#include <impedance_controller/impedance_controller.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>


namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    ImpedanceControl::ImpedanceControl(double weight, const QString &name, ros::NodeHandle nh) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      init_q_goal_(Vector6d::Zero()),
      init_period_(100.0),
      delta_q_(Vector6d::Zero()),
      delta_qp_(Vector6d::Zero()),
      ns_("~impedance_ctrl"),
      nh_(nh),
      model_("ur10_model")
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("simple_effort_controller_data", 1);

      // Start two Services
      move_arm_cartesian_service_ = nh_.advertiseService("move_arm_cartesian", &ImpedanceControl::moveArmCartesian, this);
      move_arm_joint_service_ = nh_.advertiseService("move_arm_joint", &ImpedanceControl::moveArmJoint, this);
      ROS_INFO_STREAM("ImpedanceControl: Services started");

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

    bool ImpedanceControl::moveArmCartesian(impedance_controller::MoveArmCartesian::Request &req, impedance_controller::MoveArmCartesian::Response &res)
    {
      if(control_mode_ == INIT)
      {
        ROS_WARN_STREAM("ImpedanceControl: moveArmJoint: INIT mode, cannot switch to JOINT mode");
        return false;
      }
      ROS_INFO_STREAM("MoveArmCartesian: " << req.x << " " << req.y << " " << req.z << " " << req.rx << " " << req.ry << " " << req.rz);
      control_mode_ = CARTESIAN;
      running_time_ = 0.0;

      // get the current joint state
      auto T_ef_0 = model_.T_ef_0(joint_state_.q);
      // std::cout << "T_ef_0: " << T_ef_0.translation().transpose() << std::endl;

      ee_start_.pos().linear() = T_ef_0.translation();
      ee_start_.pos().angular() = T_ef_0.rotation();

      // set the goal
      ee_goal_.pos().linear() << req.x, req.y, req.z;
      // ee_goal.angular() = Eigen:Quaterniond(Eigen::AngleAxisd(req.rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(req.ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(req.rx, Eigen::Vector3d::UnitX()));
      ee_goal_.pos().angular() = Eigen::Quaterniond(1, 0, 0, 0);
      
      double dist = (ee_goal_.pos().linear() - ee_start_.pos().linear()).norm();
      spline_period_ = 30 * dist;

      std::cout << "ee_start_.pos(): " << ee_start_.pos().linear().transpose() << std::endl;
      std::cout << "ee_goal_.pos(): " << ee_goal_.pos().linear().transpose() << std::endl;
      std::cout << "spline_period_: " << spline_period_ << std::endl;
      return true;
    }

    bool ImpedanceControl::moveArmJoint(impedance_controller::MoveArmJoint::Request &req, impedance_controller::MoveArmJoint::Response &res)
    { 
      if(control_mode_ == INIT)
      {
        ROS_WARN_STREAM("ImpedanceControl: moveArmJoint: INIT mode, cannot switch to JOINT mode");
        return false;
      }
      ROS_INFO_STREAM("MoveArmJoint: " << req.joint0 << " " << req.joint1 << " " << req.joint2 << " " << req.joint3 << " " << req.joint4 << " " << req.joint5);
      control_mode_ = JOINT;
      running_time_ = 0.0;
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
      // debug info
      // print current joint state
      std::cout << "Current joint state: " << joint_state_.q.transpose() << std::endl;
      // print goal joint state
      std::cout << "Goal joint state: " << q_goal_.transpose() << std::endl;

      spline_period_ = max_diff * 3;

      ROS_WARN_STREAM("Switching to JOINT mode");
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
      if (!model_.initRequest(nh_)) 
      {
        ROS_ERROR_STREAM("ERORR: initializing model failed!");
        m_error = true;
        return false;  
      }
      theta_ = model_.parameterInitalGuess();

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
        ROS_WARN_STREAM("START [CART]: \n" << model_.T_ef_0(state.q).toString());

        // Init time
        time_prev_ = ros::Time::now();

        // Init mode
        control_mode_ = INIT;
        is_first_iter_ = false;

        // Init model
        theta_ = model_.parameterInitalGuess();
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
      auto X_ee = model_.T_ef_0(state.q);
      
      if(control_mode_ != CARTESIAN)
      {
        ROS_INFO_STREAM_THROTTLE(1, "EE POS: " << X_ee.translation().transpose());
      }

      //////////////////////////////
      // CONTROL MODE SWITCH
      //////////////////////////////
      if(time.tD() > init_period_ && control_mode_ == INIT)
        {
          control_mode_ = JOINT;
          q_start_ = state.q;
          q_goal_ = init_q_goal_;
          running_time_ = 0.0;
          ROS_WARN_STREAM("Switching to JOINT mode");
        }
      
      if(running_time_ > spline_period_ && control_mode_ == CARTESIAN)
      {
        control_mode_ = JOINT;
        running_time_ = 0.0;
        q_start_ = joint_state_.q;
        q_goal_ = joint_state_.q;
        spline_period_ = 1.0;
        ROS_WARN_STREAM("Switching to JOINT mode");
      }

      //////////////////////////////
      // INIT MODE
      // FIXME: add gravity compensation
      //////////////////////////////
      if(control_mode_ == INIT)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // poly spline
        VVector6d q_desired;
        q_desired = getJointPVT5(q_start_, init_q_goal_, time.tD(), init_period_);

        // reference
        JointState q_ref;
        q_ref = state;
        q_ref.qp = q_desired[1] - Kp_ * (state.q - q_desired[0]);
        q_ref.qpp = q_desired[2] - Kp_ * (state.qp - q_desired[1]);

        // torque
        Vector6d Sq = state.qp - q_ref.qp;
        // Compute regressor for cartesian control
        const auto& Yr = model_.regressor(state.q, state.qp, q_ref.qp, q_ref.qpp);
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
        VVector6d q_desired;
        q_desired = getJointPVT5(q_start_, q_goal_, running_time_, spline_period_);

        // reference
        JointState q_ref;
        q_ref = state;
        q_ref.qp = q_desired[1] - Kp_ * (state.q - q_desired[0]);
        q_ref.qpp = q_desired[2] - Kp_ * (state.qp - q_desired[1]);
        // torque
        Vector6d Sq = state.qp - q_ref.qp;
        //const auto& Yr = model_.regressor(state.q, state.qp, q_ref.qp, q_ref.qpp);
        // theta_ -= gamma_ * Yr.transpose() * Sq * dt;
        tau = -Kd_ * Sq;// + Yr * theta_;
        return tau;
      }

      //////////////////////////////
      // TODO: CARTESIAN MODE
      //////////////////////////////
      else if(control_mode_ == CARTESIAN)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // get the desired cartesian state
        cc::CartesianState x_desired;
        x_desired = genTrajectoryEF(ee_start_, ee_goal_, running_time_, spline_period_);
        Quaterniond Q = Eigen::Quaterniond(1, 0, 0, 0);

        // get model
        auto X_ee = model_.T_ef_0(state.q);
        auto Jef = model_.J_ef_0(state.q);
        auto Jef_dot = model_.Jp_ef_0(state.q, state.qp);

        // current cartesian state
        cc::CartesianState x_current;
        x_current.pos().linear() = X_ee.translation();
        x_current.pos().angular() = X_ee.rotation();
        x_current.vel() = Jef * state.qp;  // 6x1

        // auto x_diff = x_desired.pos().linear() - x_current.pos().linear();
        // ROS_INFO_STREAM_THROTTLE(1, " x_current: " << x_current.pos().linear().transpose());
        // ROS_INFO_STREAM_THROTTLE(0.2, " x_desired: " << x_desired.pos().linear().transpose());
        // ROS_INFO_STREAM_THROTTLE(0.2, " x_diff:    " << x_diff.transpose());
        // ROS_INFO_STREAM_THROTTLE(0.2, "---");

        // X reference
        cc::CartesianState Xr;
        Xr.vel().linear() = x_desired.vel().linear() - Kp_cart_.topLeftCorner(3, 3) * (x_current.pos().linear() - x_desired.pos().linear());
        Xr.vel().angular() = x_current.vel().angular();
        Xr.acc().linear() = x_desired.acc().linear() - Kp_cart_.bottomRightCorner(3, 3) * (x_current.vel().linear() - x_desired.vel().linear());
        Xr.acc().angular() = x_current.acc().angular();
        // std::cout << "desired pos:       " << x_desired.pos().linear().transpose() << std::endl;
        // std::cout << "current pos:       " << x_current.pos().linear().transpose() << std::endl;
        // std::cout << "desired vel:       " << x_desired.vel().linear().transpose() << std::endl;
        // std::cout << "current vel:       " << x_current.vel().linear().transpose() << std::endl;  // check
        // std::cout << "desired acc:       " << x_desired.acc().linear().transpose() << std::endl;
        // std::cout << "Xr.vel().linear(): " << Xr.vel().linear().transpose() << std::endl;
        // std::cout << "Xr.acc().linear(): " << Xr.acc().linear().transpose() << std::endl;
        // std::cout << "... " << std::endl;

        // Jacobian pesudo inverse
        auto Jef_pinv =  Jef.transpose() * (Jef * Jef.transpose() + 0.001 * cc::Jacobian::Identity()).inverse();

        // Q reference
        Vector6d Qrp, Qrpp;
        Qrp = Jef_pinv * Xr.vel();
        Qrpp = Jef_pinv * (Xr.acc() - Jef_dot * state.qp);

        Vector6d Sq = state.qp - Qrp;
        const auto& Yr = model_.regressor(state.q, state.qp, Qrp, Qrpp);
        theta_ -= gamma_ * Yr.transpose() * Sq * dt;
        tau = -Kd_cart_ * Sq + Yr * theta_;
        return tau;
      }
      else
      {
        ROS_ERROR_STREAM(" Unknown control mode");
        return Vector6d::Zero();
      }

    }

    // TODO: add orientation spline
    cc::CartesianState ImpedanceControl::genTrajectoryEF(cc::CartesianState X_start, cc::CartesianState X_goal, double running_time, double spline_period) {
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

        X.pos().angular() = Eigen::Quaterniond(0, 0, 0, 0);
        X.vel().angular() = Eigen::Vector3d::Zero();
        X.acc().angular() = Eigen::Vector3d::Zero();
        return X;
    }

    bool ImpedanceControl::stop()
    {
      return true;
    }
  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
