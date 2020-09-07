#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer {
public:
  std::mutex mtx;

  ros::Subscriber subImuOdometry;
  ros::Subscriber subLaserOdometry;

  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  Eigen::Affine3f lidarOdomAffine;
  Eigen::Affine3f imuOdomAffineFront;
  Eigen::Affine3f imuOdomAffineBack;

  tf::TransformListener tfListener;
  tf::StampedTransform lidar2Baselink;

  double lidarOdomTime = -1;
  deque<nav_msgs::Odometry> imuOdomQueue;

  TransformFusion() {
    if(lidarFrame != baselinkFrame) {
      try {
        tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
        tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
      } catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }
    }

  subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
  subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

  pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
  pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1); 
  }

  Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
      double x, y, z, roll, pitch, yaw;
      x = odom.pose.pose.position.x;
      y = odom.pose.pose.position.y;
      z = odom.pose.pose.position.z;
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      return pcl::getTransformation(x, y, z, roll, pitch, yaw);
  }

  void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    std::lock_guard<std::mutex> lock(mtx);

    lidarOdomAffine = odom2affine(*odomMsg);

    lidarOdomTime = odomMsg->header.stamp.toSec();
  }


  void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    // static tf
    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

    std::lock_guard<std::mutex> lock(mtx);

    imuOdomQueue.push_back(*odomMsg);

    // get latest odometry (at current IMU stamp)
    if (lidarOdomTime == -1)
        return;
    while (!imuOdomQueue.empty()) {
      if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
        imuOdomQueue.pop_front();
      else
        break;
    }
    Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
    Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
    Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
    
    // publish latest odometry
    nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
    laserOdometry.pose.pose.position.x = x;
    laserOdometry.pose.pose.position.y = y;
    laserOdometry.pose.pose.position.z = z;
    laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    pubImuOdometry.publish(laserOdometry);

    // publish tf
    static tf::TransformBroadcaster tfOdom2BaseLink;
    tf::Transform tCur;
    tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
    if(lidarFrame != baselinkFrame)
      tCur = tCur * lidar2Baselink;
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
    tfOdom2BaseLink.sendTransform(odom_2_baselink);

    // publish IMU path
    static nav_msgs::Path imuPath;
    static double last_path_time = -1;
    double imuTime = imuOdomQueue.back().header.stamp.toSec();
    if (imuTime - last_path_time > 0.1) {
      last_path_time = imuTime;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
      pose_stamped.header.frame_id = odometryFrame;
      pose_stamped.pose = laserOdometry.pose.pose;
      imuPath.poses.push_back(pose_stamped);
      while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 0.1)
        imuPath.poses.erase(imuPath.poses.begin());
      if (pubImuPath.getNumSubscribers() != 0) {
        imuPath.header.stamp = imuOdomQueue.back().header.stamp;
        imuPath.header.frame_id = odometryFrame;
        pubImuPath.publish(imuPath);
      }
    }
  }
};

class IMUPreintergration : public ParamServer {
public:
  std::mutex mtx;

  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher pubImuOdometry;

  bool systemInitialized = false;

  //  一共有4个噪声模型，三个是先验，另一个correction是什么？
  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::Vector noiseModelBetweenBias;

  gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

  std::deque<sensor_msgs::Imu> imuQueOpt;
  std::deque<sensor_msgs::Imu> imuQueImu;

  // 用于保存上个时刻的状态。分别是姿态、位置、偏置以及误差 所以state是什么？
  gtsam::Pose3 prevPose_;
  gtsam::Vector3 preVel_;
  gtsam::NavState preState_;
  gtsam::imuBias::ConstantBias preBias_;

  gtsam::NavState prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool doneFirstOpt = false;
  // 用于记录积分时间，上一次的imu时间
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2 optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values graphValues;

  const double delta_t = 0;

  int key = 1;

  // lidar 和 imu 的转换矩阵
  gtsam::Pose3 imu2lidar = gtsam::Pose3(gtsam::Rot(1, 0, 0, 0), gtsam::Points(-extTrans.x(), -extTrans.y(), -extTrans.z()));
  gtsam::Pose3 lidar2imu = gtsam::Pose3(gtsam::Rot(1, 0, 0, 0), gtsam::Points(extTrans.x(), extTrans.y(), extTrans.z()));

  IMUPreintergration() {
    subImu      = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

    // imupre 因子 需要设置参数
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity); //固定用法
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    // 给噪声模型 赋值
    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1); // meter
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    // 构造函数 prior_imu_bias 是当前的对于 加速度和旋转的估计
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization   
  }

  // 更新 1.optimize的参数 2.更新graph，尤其是其中的value
  void resetOptimization() {
    // 重置ISAM2参数
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    // 重置 NonlinearFactorGraph
    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;

    // 重置 graphValues
    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
  }

  void resetParams() {
    lastImuT_imu = -1;
    doneFirstOpt = false;
    systemInitialized = false;
  }

  // 1. 速度限制，不能过快  2. 偏置不能过大
  bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30)
    {
        ROS_WARN("Large velocity, reset IMU-preintegration!");
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        ROS_WARN("Large bias, reset IMU-preintegration!");
        return true;
    }

    return false;
  }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    // 进程锁，直到本次结束
    std::lock_guard<std::mutex> lock(mtx);

    double currentCorrectionTime = ROS_TIME(odomMsg);

    // 确保之前有imu数据，可以用来做预计分
    if(imuQueOpt.empty())
      return;

    // 读取数据
    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;
    // 构建lidar odom的node
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

    // 0 系统未初始化
    if(systemInitialized == false) {
      // 1. 重置所有参数
      resetOptimization();

      // 2. 丢弃0.1s之外所有的imu数据，imu预计分 只预计分0.1s内的
      while(!imuQueOpt.empty()) {
        if(ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
          lastImuT_opt = ROS_TIME(&imuQueOpt.front());
          imuQueOpt.pop_front();
        } else
          break;
      }

      // initial pose
      // compose什么意思？ 坐标变换？ 位置插入的是lidarodm的数据
      prevPose_ = lidarPose.compose(lidar2Imu);
      gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise_);
      graphFactors.add(priorPose);
      // initial vel
      prevVel_ = gtsam::Vector3(0, 0, 0);
      gtsam::PriorFactor<gtsam::Pose3> priorVel(V(0), prevVel_, priorVelNoise_);
      graphFactors.add(priorVel);
      // initial bias
      prevBias_ = gtsam::imuBias::ConstantBias();
      gtsam::PriorFactor<gtsam::Pose3> priorBias(B(0), prevBias_, priorBiasNoise_);
      graphFactors.add(priorBias);

      // add value
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);

      // optimize once 有图 有值才能优化
      optimizer.update(graphFactors, graphValues);
      // graphFactors.resize(0); 什么意思？？
      graphFactors.resize(0);
      graphValues.clear();

      // 预计分 测量值的重置？？
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
      imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

      // 计数器 设置为1 ， 初始化ok， 开始基数了
      key = 1;
      systemInitialized = true;
      return;
    }

    // 100的时候，就需要将之前的 边缘化了 这样可以提高速度
    if(key == 100) {
      // 1. get 边缘化的 Noise  直到 key-1 ？？？
      gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
      // 2. restet graph
      resetOptimization();
      // 3. construct new graph
        // 1. add graph
      gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
      graphFactors.add(priorPose);
      gtsam::PriorFactor<gtsam::Pose3> priorVel(V(0), prevVel_, updatedVelNoise);
      graphFactors.add(priorVel);
      gtsam::PriorFactor<gtsam::Pose3> priorBias(B(0), prevBias_, updatedBiasNoise);
      graphFactors.add(priorBias);
        // 2. add value
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);
      // 4. update graph
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      key = 1;
    }

    // 1. integrate imu data and optimize
      // imuQueOpt 是待优化的队列 在根据时间处理后 将值押入因子图，而且是直到最近的 0.1s 之前的，为什么以0.1s为界限？
    while(!imuQueOpt.empty()) {
      sensor_msgs::Imu *thisImu = &imuQueOpt.front();
      double imuTime = ROS_TIME(thisImu);
      if(imuTime < currentCorrectionTime - delta_t) {
        double dt = (lastImuT_opt < 0) ? (1.0/500.0) : (imuTime - lastImuT_opt);
        // 积分间隔时间 积分是要知道时间的啊！
        imuIntegratorOpt_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
        lastImuT_opt = imuTime;
        imuQueOpt.pop_front();
      } else 
        break;
    }

    // 正常情况 add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    // https://gtsam.org/doxygen/a03467.html#a93d499891d2801915a26a344b06036bb 
      // pose_i	Previous pose key
      // vel_i	Previous velocity key
      // pose_j	Current pose key
      // vel_j	Current velocity key
      // bias	Previous bias key
    gtsam::ImuFactor imu_factor(X(key-1), V(key-1), X(key), V(key), B(key-1), preint_imu);
    // 下面两个 add 到底有什么区别？
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                     gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

    // 添加 激光odom的里程计 作为因子图
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise_);
    graphFactors.add(pose_factor);
    // 加入预测的值(迭代的初始值) imu预测的值 作为预测值
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(B(key), propState_.v());
    graphValues.insert(V(key), prevBias_);
    // optimize  ?? 为什么后面一个空的update
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step. 进行参数的更新  
    gtsam::Values result = optimizer.calculateEstimate();
    prevPose_  = result.at<gtsam::Pose3>(X(key));
    prevVel_   = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // 重置  optimization preintegration 对象
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // chaeck optimizatiom
    if(failureDetection(prevVel_, prevBias_)) {
      resetParams();
      return;
    }  

    // 2. 优化后，re-propagate imu odometry preintegration 这个是什么？
    // 保存数据 将这个数据保存下来，在imu handler中用
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;

    // 这个是干什么？ 干嘛出来一个imuQueImu？？ 只有这里用到imuQueImu 将odom 优化后的加入到imu系列
    // imuQueImu不停出来，直到currentCorrectionTime - delta_t
    double lastImuQt = -1;
    while(!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
      lastImuQt = ROS_TIME(&imuQueImu.front());
      imuQueImu.pop_front();
    }
    // re-propagate
    if(!imuQueImu.empty()) {
      // 重置对于 bias 的估计
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
      for(int i = 0; i < (int)imuQueImu.size(), ++i) {
        sensor_msgs::Imu *thisImu = &imuQueImu[i];
        double imuTime = ROS_TIME(thisImu);
        double dt = (lastImuQt < 0) ? (1.0/500.0) : (imuTime - lastImuQt);
        // https://gtsam.org/doxygen/a03527.html#af81180bb6e01ac950d29784a43e8f18d
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
        lastImuQT = imuTime;
      }
    }
    ++key;
    doneFirstOpt = true;
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw) {
    std::lock_guard<std::mutex> lock(mtx);
    // 这个地方的 imuConverter 是怎么回事？
    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

    //保存 imuQueOpt 和 imuQueImu
    imuQueOpt.push_back(thisImu); //用于 odom 的更新
    imuQueImu.push_back(thisImu);

    // 为什么要 第一次优化后？？
    if (doneFirstOpt == false)
      return;

    double imuTime = ROS_TIME(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

    // predict odometry Predict state at time j. 
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // 构造 odometry 并发送
    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = odometryFrame;
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    pubImuOdometry.publish(odometry);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, 'robot_loam');

  IMUPreintergration ImuP;

  TransformFusion TF;

  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}