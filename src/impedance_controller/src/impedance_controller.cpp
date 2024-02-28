#include <impedance_controller/impedance_controller.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    ImpedanceControl::ImpedanceControl(double weight, const QString &name, ros::NodeHandle nh) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      is_first_iter_cart_(true),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      Kp_cart_(Matrix6d::Zero()),
      Kd_cart_(Matrix6d::Zero()),
      Ki_cart_(Matrix6d::Zero()),
      init_q_goal_(Vector6d::Zero()),
      init_period_(100.0),
      delta_q_(Vector6d::Zero()),
      delta_qp_(Vector6d::Zero()),
      ns_("~impedance_ctrl"),
      nh_(nh),
      model_("ur10_model")
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("simple_effort_controller_data", 1);
      joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
      ee_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 1);
      wrench_sub_ = nh.subscribe("/schunk_netbox/raw", 10, &ImpedanceControl::wrenchCallback, this);

      // Start two Services
      move_arm_cartesian_service_ = nh_.advertiseService("move_arm_cartesian", &ImpedanceControl::moveArmCartesian, this);
      move_arm_joint_service_ = nh_.advertiseService("move_arm_joint", &ImpedanceControl::moveArmJoint, this);
      get_wrench_data_service = nh_.advertiseService("get_wrench_data", &ImpedanceControl::getWrenchData, this);

    }

    ImpedanceControl::~ImpedanceControl()
    {
    }

    void ImpedanceControl::setQInit(const JointState &q_init)
    {
      q_init_ = q_init;
    }
    void ImpedanceControl::setQHome(const JointState &q_home)
    {
      q_home_ = q_home;
    }
    void ImpedanceControl::setQPark(const JointState &q_park)
    {
      q_park_ = q_park;
    }

    void ImpedanceControl::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) 
    {
    this->latest_wrench = *msg;
    }

    bool ImpedanceControl::moveArmCartesian(impedance_controller::MoveArmCartesian::Request &req, impedance_controller::MoveArmCartesian::Response &res)
    {
      if(control_mode_ == INIT)
      {
        ROS_WARN_STREAM("ImpedanceControl: moveArmJoint: INIT mode, cannot switch to JOINT mode");
        return false;
      }
      ROS_INFO_STREAM("MoveArmCartesian: " << req.x << " " << req.y << " " << req.z << " " << req.rx << " " << req.ry << " " << req.rz);
      control_mode_ = CARTESIAN;
      cart_error_ = Vector6d::Zero();
      running_time_ = 0.0;

      // get the current joint state
      auto T_ef_0 = model_.Tef_0(joint_state_.q);
      // std::cout << "T_ef_0: " << T_ef_0.translation().transpose() << std::endl;

      ee_start_.pos().linear() = T_ef_0.translation();
      ee_start_.pos().angular() = T_ef_0.rotation();
      // ROS_INFO_STREAM("TT_ef_0.rotation(): " << T_ef_0.rotation());

      // set the goal
      ee_goal_.pos().linear() << req.x, req.y, req.z;
      // ee_goal.angular() = Eigen:Quaterniond(Eigen::AngleAxisd(req.rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(req.ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(req.rx, Eigen::Vector3d::UnitX()));
      Eigen::Matrix3d rotation_matrix = T_ef_0.rotation();;
      // rotation_matrix <<0.0,0.0, -0.999999,
      //                   -0.5, 0.0,0.0,
      //                   0.0,-0.5, 0.0;
      // for (int i = 0; i < rotation_matrix.rows(); ++i) {
      //     for (int j = 0; j < rotation_matrix.cols(); ++j) {
      //         rotation_matrix(i, i) -= 0.0000001;
      //     }
      // }
                        

      ee_goal_.pos().angular() = rotation_matrix;

      
      double dist = (ee_goal_.pos().linear() - ee_start_.pos().linear()).norm();
      if (dist < 0.3)
      {
        spline_period_ = 0.3; // avoid speed limit
      }
      else
      {
        spline_period_ = dist * 10;
      }
      // spline_period_ = dist * 30;
      
      // std::cout << "spline_period_: " << spline_period_ << std::endl;

      // std::cout << "ee_start_.pos(): " << ee_start_.pos().linear().transpose() << std::endl;
      // std::cout << "ee_goal_.pos(): " << ee_goal_.pos().linear().transpose() << std::endl;
      // std::cout << "spline_period_: " << spline_period_ << std::endl;
      // ROS_WARN_STREAM("Switching to CARTESIAN mode");
      return true;
    }

    bool ImpedanceControl::moveArmJoint(impedance_controller::MoveArmJoint::Request &req, impedance_controller::MoveArmJoint::Response &res)
    { 
      if(control_mode_ == INIT)
      {
        ROS_WARN_STREAM("ImpedanceControl: moveArmJoint: INIT mode, cannot switch to JOINT mode");
        return false;
      }
      else if(control_mode_ == CARTESIAN)
      {
        ROS_WARN_STREAM("Change to JOINT mode from CARTESIAN mode");
      }
      ROS_INFO_STREAM("MoveArmJoint: " << req.joint0 << " " << req.joint1 << " " << req.joint2 << " " << req.joint3 << " " << req.joint4 << " " << req.joint5);
      control_mode_ = JOINT;
      running_time_ = 0.0;

      joint_error_ = Vector6d::Zero();
      q_start_ = joint_state_.q;
      q_goal_(0) = req.joint0;
      q_goal_(1) = req.joint1;
      q_goal_(2) = req.joint2;
      q_goal_(3) = req.joint3;
      q_goal_(4) = req.joint4;
      q_goal_(5) = req.joint5;

      // find the max difference between the current and the goal position
      double max_diff = 0.0;
      for (int i = 0; i < STD_DOF; i++)
      {
        double diff = fabs(q_goal_(i) - joint_state_.q(i));
        if (diff > max_diff)
        {
          max_diff = diff;
        }
      }
      spline_period_ = max_diff * 3;
      return true;
    }

    bool ImpedanceControl::getWrenchData(impedance_controller::GetWrenchData::Request &req,impedance_controller::GetWrenchData::Response &res) 
    {
        res.wrench = latest_wrench;
        return true;
    }

    bool ImpedanceControl::init()
    {
      ROS_WARN_STREAM("ImpedanceControl::init");
      std::vector<double> vec;

      // check namespace
      // std::string ns_ = "~simple_effort_ctrl";
      if (!ros::param::has(ns_))
      {
        ROS_ERROR_STREAM("ImpedanceControl init(): Control gains not defined in:" << ns_);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns_ + "/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd: \n" << Kd_);

      // P GAINS
      ros::param::get(ns_ + "/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_(i, i) = vec[i] / Kd_(i, i);
      }
      ROS_WARN_STREAM("Kp: \n" << Kp_);

      // init joint position
      ros::param::get(ns_ + "/init_q", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_q: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        init_q_goal_(i) = vec[i];
      }

      // FIXME: init model
      if (!model_.is_Initialized(nh_)) 
      {
        ROS_ERROR_STREAM("ERORR: initializing model failed!");
        m_error = true;
        return false;  
      }
      theta_ = model_.Theta_function();

      ros::param::get(ns_ + "/learning_rate", gamma_);
      if (!(gamma_ > 0))
      {
        ROS_ERROR_STREAM("gamma_: is negative:" << gamma_);
        gamma_ = 0.1;
      }
      ROS_WARN_STREAM("gamma_: " << gamma_);

      // p gains for cartesian control
      ros::param::get(ns_ + "/gains_p_cart", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_p_cart: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_cart_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kp_cart_: \n" << Kp_cart_);

      // d gains for cartesian control
      ros::param::get(ns_ + "/gains_d_cart", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_d_cart: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kd_cart_(i, i) = vec[i];
      }

      // i gains for cartesian control
      ros::param::get(ns_ + "/gains_i_cart", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_i_cart: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Ki_cart_(i, i) = vec[i];
      }

      ros::param::get(ns_ + "/gains_i", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_i_cart: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Ki_(i, i) = vec[i];
      }

      // low pass filter
      ros::param::get(ns_ + "/low_pass_factor", low_pass_factor_);
      if (!(low_pass_factor_ > 0))
      {
        ROS_ERROR_STREAM("low_pass_factor_: is negative:" << low_pass_factor_);
        low_pass_factor_ = 0.1;
      }
      
      // init time
      ros::param::get(ns_ + "/init_period", init_period_);
      if (!(init_period_ > 0))
      {
        ROS_ERROR_STREAM("init_period_: is negative:" << init_period_);
        init_period_ = 10.0;
      }

      ROS_WARN_STREAM("Goal [DEG]: \n" << init_q_goal_.transpose());
      ROS_WARN_STREAM("Total Time [s]: " << init_period_);
      init_q_goal_ = DEG2RAD(init_q_goal_);
      ROS_WARN_STREAM("Goal [RAD]: \n" << init_q_goal_.transpose());
      return true;
    }

    bool ImpedanceControl::start()
    {
      ROS_WARN_STREAM("ImpedanceControl::start");
      return true;
    }

    Vector6d ImpedanceControl::update(const RobotTime &time, const JointState &state)
    {
      if (is_first_iter_)
      {
        q_start_ = state.q;
        ROS_WARN_STREAM("START [DEG]: \n" << q_start_.transpose());
        ROS_WARN_STREAM("START [CART]: \n" << model_.Tef_0(state.q).linear());

        // Init time
        time_prev_ = ros::Time::now();

        // Init mode
        control_mode_ = INIT;
        is_first_iter_ = false;
        is_check_obst_ = false;

        // Init model
        theta_ = model_.Theta_function();

        // Init error
        joint_error_ = Vector6d::Zero();
        joint_dot_error_ = Vector6d::Zero();

        // init q
        q_desired_ = getJointPVT5(q_start_, init_q_goal_, running_time_, init_period_);
      }

      //////////////////////////////
      // PARMAS UPDATE
      // This part aim to update the global parameters of the controller
      //////////////////////////////
      ros::Time time_cur = ros::Time::now();
      double dt = (time_cur - time_prev_).toSec();
      time_prev_ = time_cur;
      running_time_ += dt;
      joint_state_ = state;
      auto X_ee = model_.Tef_0(state.q);

      // publish  data
      {
        // publish joint state
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joint_state_msg.position = {state.q(0), state.q(1), state.q(2), state.q(3), state.q(4), state.q(5)};
        joint_state_pub_.publish(joint_state_msg);

        // publish ee pose
        geometry_msgs::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = ros::Time::now();
        ee_pose_msg.pose.position.x = X_ee.translation()(0);
        ee_pose_msg.pose.position.y = X_ee.translation()(1);
        ee_pose_msg.pose.position.z = X_ee.translation()(2);
        // TODO: add orientation
        // ee_pose_msg.pose.orientation.x = X_ee.rotation().x();
        // ee_pose_msg.pose.orientation.y = X_ee.rotation().y();
        // ee_pose_msg.pose.orientation.z = X_ee.rotation().z();
        // ee_pose_msg.pose.orientation.w = X_ee.rotation().w();
        ee_pose_pub_.publish(ee_pose_msg);
      }

      if(control_mode_ != CARTESIAN)
      {
        ROS_INFO_STREAM_THROTTLE(1, "EE POS: " << X_ee.translation().transpose());
        // ROS_INFO_STREAM_THROTTLE(1, "Freq: " << 1.0 / dt);
      }

      //////////////////////////////
      // CONTROL MODE SWITCH
      //////////////////////////////
      if(time.tD() > init_period_ + 5 && control_mode_ == INIT)
        {
          control_mode_ = JOINT;
          // q_start_ = state.q;
          q_goal_ = init_q_goal_;
          spline_period_ = init_period_;
          ROS_WARN_STREAM("Switching to JOINT mode");
        }
      

      //////////////////////////////
      // INIT MODE
      //////////////////////////////
      if(control_mode_ == INIT)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // poly spline
        VVector6d q_desired_cur;
        q_desired_cur = getJointPVT5(q_start_, init_q_goal_, running_time_, init_period_);

        // low pass filter
        for(int i = 0; i < 3; i++)
        {
          q_desired_[i] = low_pass_factor_ * q_desired_[i] + (1 - low_pass_factor_) * q_desired_cur[i];
        }

        // PID control
        joint_error_ += (state.q - q_desired_[0]) * dt; 
        joint_dot_error_ += (state.qp - q_desired_[1]) * dt;

        // reference
        JointState q_ref;
        q_ref = state;
        q_ref.qp = q_desired_[1] - Kp_ * (state.q - q_desired_[0]) - Ki_ * joint_error_;
        q_ref.qpp = q_desired_[2] - Kp_ * (state.qp - q_desired_[1]) - Ki_ * joint_dot_error_;

        // torque
        Vector6d Sq = state.qp - q_ref.qp;
        // Compute regressor for cartesian control
        const auto& Yr = model_.Yr_function(state.q, state.qp, q_ref.qp, q_ref.qpp);
        theta_ -= gamma_ * Yr.transpose() * Sq * dt;
        tau = -Kd_ * Sq + Yr * theta_;
        return tau;
      }

      //////////////////////////////
      // JOINT MODE
      //////////////////////////////
      else if(control_mode_ == JOINT)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // poly spline
        VVector6d q_desired_cur;
        q_desired_cur = getJointPVT5(q_start_, q_goal_, running_time_, spline_period_);

        // low pass filter
        for(int i = 0; i < 3; i++)
        {
          q_desired_[i] = low_pass_factor_ * q_desired_[i] + (1 - low_pass_factor_) * q_desired_cur[i];
        }

        // PID control
        joint_error_ += (state.q - q_desired_[0]) * dt; 
        joint_dot_error_ += (state.qp - q_desired_[1]) * dt;

        // reference
        JointState q_ref;
        q_ref = state;
        q_ref.qp = q_desired_[1] - Kp_ * (state.q - q_desired_[0])- Ki_ * joint_error_;
        q_ref.qpp = q_desired_[2] - Kp_ * (state.qp - q_desired_[1])- Ki_ * joint_dot_error_;

        // torque
        const auto& Yr = model_.Yr_function(state.q, state.qp, q_ref.qp, q_ref.qpp);
        Vector6d Sq = state.qp - q_ref.qp;
        tau = -Kd_ * Sq + Yr * theta_;
        return tau;
      }

      //////////////////////////////
      // CARTESIAN MODE
      //////////////////////////////
      else if(control_mode_ == CARTESIAN)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // get the desired cartesian state
        cc::CartesianState x_desired_cur;//VVector6d x_desired_cur;
        x_desired_cur = genTrajectoryEF(ee_start_, ee_goal_, running_time_, spline_period_,dt);

        // low pass filter
        if(is_first_iter_cart_)
        {
          x_desired_ = x_desired_cur;
          is_first_iter_cart_ = false;
        }
        x_desired_.pos() = low_pass_factor_ * x_desired_.pos() + (1 - low_pass_factor_) * x_desired_cur.pos();
        x_desired_.vel() = low_pass_factor_ * x_desired_.vel() + (1 - low_pass_factor_) * x_desired_cur.vel();
        x_desired_.acc() = low_pass_factor_ * x_desired_.acc() + (1 - low_pass_factor_) * x_desired_cur.acc();

        // get model
        auto X_ee = model_.Tef_0(state.q);
        auto Jef = model_.Jef_0(state.q);
        auto Jef_dot = model_.Jef_0_dot(state.q, state.qp);

        // current cartesian state
        cc::CartesianState x_current;
        x_current.pos().linear() = X_ee.translation();
        x_current.pos().angular() = X_ee.rotation();
        x_current.vel() = Jef * state.qp;  // 6x1

        ROS_INFO_STREAM_THROTTLE(1, " x_current: " << x_current.pos().linear().transpose());

        // error
        cart_error_.head(3) += (x_current.pos().linear() - x_desired_.pos().linear()) * dt;
        cart_dot_error_.head(3) += (x_current.vel().linear() - x_desired_.vel().linear()) * dt;

        ROS_INFO_STREAM_THROTTLE(1, " cart_error_: " << cart_error_.head(3).transpose());

        // X reference
        cc::CartesianState Xr;
        Xr.vel().linear() = x_desired_.vel().linear() - Kp_cart_.topLeftCorner(3, 3) * (x_current.pos().linear() - x_desired_.pos().linear()) - Ki_cart_.topLeftCorner(3, 3) * cart_error_.head(3);
        Xr.vel().angular() = Eigen::Vector3d::Zero();
        Xr.acc().linear() = x_desired_.acc().linear() - Kp_cart_.bottomRightCorner(3, 3) * (x_current.vel().linear() - x_desired_.vel().linear()) - Ki_cart_.bottomRightCorner(3, 3) * cart_dot_error_.head(3);
        Xr.acc().angular() = Eigen::Vector3d::Zero();

        // Jacobian pesudo inverse
        auto Jef_pinv =  Jef.transpose() * (Jef * Jef.transpose() + 0.001 * cc::Jacobian::Identity()).inverse();

        // Q reference
        Vector6d Qrp, Qrpp;
        Qrp = Jef_pinv * Xr.vel();
        Qrpp = Jef_pinv * (Xr.acc() - Jef_dot * state.qp);

        if (latest_wrench.wrench.force.z > 300)
        {
          double force_ = std::abs(latest_wrench.wrench.force.z - 485);
          double max_range_ = std::abs(470-485);
          double Level_ = (force_ / max_range_) * 5.0;
          ROS_INFO_STREAM_THROTTLE(1, "Force:"<< force_);
          ROS_INFO_STREAM_THROTTLE(1, "Level_:"<< Level_);

          // Level=0: get 1
          // Level=5: get 0
          Qrp = (-0.2*Level_ + 1) * Qrp;
          Qrpp = (-0.2*Level_ + 1) * Qrpp;
        }


        Vector6d Sq = state.qp - Qrp;
        const auto& Yr = model_.Yr_function(state.q, state.qp, Qrp, Qrpp);
        theta_ -=  0.05 * gamma_ * Yr.transpose() * Sq * dt;
        tau = -Kd_cart_ * Sq + Yr * theta_;
      //   ROS_INFO_STREAM_THROTTLE(1, "tau:"<< tau);

        // if (latest_wrench.wrench.force.z == 485)
        // {
        //   if (is_check_obst_ == false)
        //   {
        //     is_check_obst_ = true;
        //     // X_impendance_endpoint_ = x_desired_cur.pos().linear() ;
        //     // // todo
        //     // X_impendance_endpoint_(2) -= 0.1;
        //     X_impendance_endpoint_ << 0.47, -0.16, 0.51;
        //   }
          
        //     ROS_INFO_STREAM_THROTTLE(1, "Received force: [" 
        //         << latest_wrench.wrench.force.x << ", " 
        //         << latest_wrench.wrench.force.y << ", " 
        //         << latest_wrench.wrench.force.z << "] ");
        //   // Get the nullspace stuff 
        //   //Vector3d X_avoidance_point = x_desired_cur.pos().linear();
        //   auto tauTotalAvoid = computeImpedanceTau(state, X_impendance_endpoint_, 5);//Fix
        //   auto Null_sp = Matrix6d::Identity() - Jef.transpose()  *  Jef_pinv.transpose();
        //   Vector6d task2 = Null_sp * tauTotalAvoid;

        //   Vector6d tau_avoiding = task2; //cartesianAvoiding(state, x_desired_cur, dt);
          
        //   // tau_avoiding.setZero();
        //   // tau_avoiding(2) = task2(2)*100*6;

        //   // ROS_INFO_STREAM_THROTTLE(1, "tau:"<< tau);
        //   tau_avoiding = tau_avoiding*500;
        //   ROS_INFO_STREAM_THROTTLE(1, "tau_avoiding:"<< tau_avoiding);

        //   tau += tau_avoiding;

        //   ROS_INFO_STREAM_THROTTLE(1, "tau_new:"<< tau);
            
        // }

        return tau;
      }


    }

    // TODO: add orientation spline
    cc::CartesianState ImpedanceControl::genTrajectoryEF(cc::CartesianState X_start, cc::CartesianState X_goal, double running_time, double spline_period,double dt) {
        cc::CartesianState X;

        double t = running_time;
        double tf = spline_period;  
        double t0 = 0.0;

        if (t <= tf) {
            Eigen::MatrixXd T(6, 6);
            T << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
                 1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
                 0, 1,  2*t0, 3*pow(t0, 2), 4*pow(t0, 3), 5*pow(t0, 4),
                 0, 1,  2*tf, 3*pow(tf, 2), 4*pow(tf, 3), 5*pow(tf, 4),
                 0, 0,  2, 6*t0, 12*pow(t0, 2), 20*pow(t0, 3),
                 0, 0,  2, 6*tf, 12*pow(tf, 2), 20*pow(tf, 3);

            Eigen::MatrixXd X_matrix(6, 3);
            X_matrix.row(0) = X_start.pos().transpose();
            X_matrix.row(1) = X_goal.pos().transpose();
            X_matrix.row(2) = X_start.vel().transpose();
            X_matrix.row(3) = X_goal.vel().transpose();
            X_matrix.row(4) = X_start.acc().transpose();
            X_matrix.row(5) = X_goal.acc().transpose();

            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;

            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd a = T.colPivHouseholderQr().solve(X_matrix.col(i));
                double pos = a(0) + a(1)*t + a(2)*pow(t, 2) + a(3)*pow(t, 3) + a(4)*pow(t, 4) + a(5)*pow(t, 5);
                double vel = a(1) + 2*a(2)*t + 3*a(3)*pow(t, 2) + 4*a(4)*pow(t, 3) + 5*a(5)*pow(t, 4);
                double acc = 2*a(2) + 6*a(3)*t + 12*a(4)*pow(t, 2) + 20*a(5)*pow(t, 3);

                X.pos().linear()(i) = pos;
                X.vel().linear()(i) = vel;
                X.acc().linear()(i) = acc;
            }
        } else {
            X = X_goal;
        }

        // Quaternion interpolation for angular part
        double interpolation_factor = std::min(t / tf, 1.0);
        Eigen::Quaterniond Q_start(X_start.pos().angular());
        Eigen::Quaterniond Q_goal(X_goal.pos().angular());
        Eigen::Quaterniond Q_interpolated = Q_start.slerp(interpolation_factor, Q_goal);
        X.pos().angular() = Q_interpolated.toRotationMatrix();
        // X.pos().angular() = Eigen::Quaterniond(0, 0, 0, 0);
        //X.vel().angular() = Eigen::Vector3d::Zero();
        //X.acc().angular() = Eigen::Vector3d::Zero();

        // Approximate angular velocity and acceleration
        Eigen::Quaterniond Q_prev = Q_start.slerp(std::max(0.0, (t - dt) / tf), Q_goal);
        Eigen::Quaterniond Q_next = Q_start.slerp(std::min(1.0, (t + dt) / tf), Q_goal);

        // Compute the finite difference for angular velocity
        Eigen::AngleAxisd angleAxisVel(Q_prev.inverse() * Q_interpolated);
        Eigen::Vector3d angularVel = angleAxisVel.axis() * angleAxisVel.angle() /10;// dt;

        // Compute the finite difference for angular acceleration
        Eigen::AngleAxisd angleAxisPrev(Q_prev.inverse() * Q_interpolated);
        Eigen::AngleAxisd angleAxisNext(Q_interpolated.inverse() * Q_next);
        Eigen::Vector3d angularAcc = (angleAxisNext.axis() * angleAxisNext.angle() - angleAxisPrev.axis() * angleAxisPrev.angle()) /10;//(dt * dt);

        X.vel().angular() = angularVel;
        X.acc().angular() = angularAcc;
       
        return X;
    }


    bool ImpedanceControl::stop()
    {
      return true;
    }

    Vector6d ImpedanceControl::computeImpedanceTau(const JointState& state, const Vector3d& X_red_, const int j)
    {
      // See if the ball is by the robot before activate the avoidance  
      cc::HomogeneousTransformation T_i_0;
      Vector3d F_red_i;
      Vector6d tau_red_i;
      double d;
      double d_min = 10e-5;
      T_i_0 = model_.Ti_0(state.q, j);
      d = (X_red_ - T_i_0.position()).norm();  
      if(d < d_min)
        d = d_min;

      // if the ball is within the detection zone 
      Vector3d current_spingK; 
      // current_spingK << springK_[j] ,springK_[j] ,springK_[j] ; 
      double SPRING = springK_[0]; // * 10e10;

      current_spingK << SPRING, SPRING,SPRING;
      if(d < 0.5)
      {
        F_red_i.x() = current_spingK.x() /  (X_red_ - T_i_0.position()).x();
        F_red_i.y() = current_spingK.y() /  (X_red_ - T_i_0.position()).y();
        F_red_i.z() = current_spingK.z() /  (X_red_ - T_i_0.position()).z();

        tau_red_i = model_.Ji_0(state.q,j).block(0, 0, 2, 5).transpose() * F_red_i;
      }
      else    // else we can set everything to zero 
          tau_red_i.setZero(); 
        
      // ROS_INFO_STREAM("Tau Avoidance joint i " << tau_red_i.transpose());

      return tau_red_i;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
