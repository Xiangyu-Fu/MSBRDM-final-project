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
        ROS_INFO_STREAM("MoveArmCartesian: " << req.x << " " << req.y << " " << req.z << " " << req.rx << " " << req.ry << " " << req.rz);
        control_mode_ = CARTESIAN;
        running_time_ = 0.0;

        // get the current joint state
        auto T_ef_0 = model_.T_ef_0(joint_state_.q);
        std::cout << "T_ef_0: " << T_ef_0.translation().transpose() << std::endl;
        ee_start_.linear() = T_ef_0.translation();
        ee_start_.angular() = T_ef_0.rotation();

        // set the goal
        ee_goal_.linear() << req.x, req.y, req.z;
        // ee_goal.angular() = Eigen:Quaterniond(Eigen::AngleAxisd(req.rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(req.ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(req.rx, Eigen::Vector3d::UnitX()));
        ee_goal_.angular() = Eigen::Quaterniond(1, 0, 0, 0);
        
        double dist = (ee_goal_.linear() - ee_start_.linear()).norm();
        spline_period_ = 10;

        std::cout << "ee_start_: " << ee_start_.linear().transpose() << std::endl;
        std::cout << "ee_goal_: " << ee_goal_.linear().transpose() << std::endl;
        std::cout << "spline_period_: " << spline_period_ << std::endl;
        return true;
    }

    bool ImpedanceControl::moveArmJoint(impedance_controller::MoveArmJoint::Request &req, impedance_controller::MoveArmJoint::Response &res)
    {
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
      std::string ns = "~simple_effort_ctrl";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("ImpedanceControl init(): Control gains not defined in:" << ns);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns + "/gains_d", vec);
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
      ros::param::get(ns + "/gains_p", vec);
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
      ros::param::get(ns + "/init_q", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
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
      
      // init time
      ros::param::get(ns + "/init_period", init_period_);
      if (!(init_period_ > 0))
      {
        ROS_ERROR_STREAM("init_period_: is negative:" << init_period_);
        init_period_ = 100.0;
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
        // Init time
        time_prev_ = ros::Time::now();

        // Init mode
        control_mode_ = INIT;
        is_first_iter_ = false;
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
      auto T_ef_0 = model_.T_ef_0(state.q);
      auto T_B_0 = model_.T_B_0();


      // std::cout << "T_ef_0: " << T_B_0.toString() << std::endl;
      // ee_pose_.pos().linear() = T_ef_0.translation();  
      // ee_pose_.pos().angular() = T_ef_0.rotation();
      // ROS_WARN_STREAM_THROTTLE(1, "Current EE pose: " << ee_pose_.pos().linear().transpose());
      // ROS_WARN_STREAM_THROTTLE(10, "Running Time [s]: " << time.tD());

      //////////////////////////////
      // CONTROL MODE
      //////////////////////////////
      if(time.tD() > init_period_ && control_mode_ == INIT)
        {
          control_mode_ = JOINT;
          q_start_ = state.q;
          q_goal_ = state.q;
          running_time_ = 0.0;
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
        VVector6d vQd;
        vQd = getJointPVT5(q_start_, init_q_goal_, time.tD(), init_period_);

        // erros
        delta_q_ = state.q - vQd[0];
        delta_qp_ = state.qp - vQd[1];

        // reference
        JointState js_r;
        js_r = state;
        js_r.qp = vQd[1] - Kp_ * delta_q_;
        js_r.qpp = vQd[2] - Kp_ * delta_qp_;

        // torque
        Vector6d Sq = state.qp - js_r.qp;
        tau = -Kd_ * Sq;
        return tau;
      }

      // FIXME: there always a joint error between the goal and the current position

      //////////////////////////////
      // JOINT MODE
      //////////////////////////////
      else if(control_mode_ == JOINT)
      {
        // control torque
        Vector6d tau;
        tau.setZero();
        // poly spline
        VVector6d vQd;
        vQd = getJointPVT5(q_start_, q_goal_, running_time_, spline_period_);

        delta_q_ = state.q - vQd[0];
        delta_qp_ = state.qp - vQd[1];

        // reference
        JointState js_r;
        js_r = state;
        js_r.qp = vQd[1] - Kp_ * delta_q_;
        js_r.qpp = vQd[2] - Kp_ * delta_qp_;

        // torque
        Vector6d Sq = state.qp - js_r.qp;
        tau = -Kd_ * Sq;
        return tau;
      }

      //////////////////////////////
      // FIXME: CARTESIAN MODE
      //////////////////////////////
      else if(control_mode_ == CARTESIAN)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // poly spline
        Vector6d x_start, x_goal;
        VVector6d vQd;
        x_start.head(3) = ee_start_.linear();
        x_goal.head(3) = ee_goal_.linear();
        vQd = getJointPVT5(x_start, x_goal, running_time_, spline_period_);
        Quaterniond Q = Quaterniond::Identity();

        std::cout << "vQd[0]: " << vQd[0].transpose() << std::endl;
        std::cout << "vQd[1]: " << vQd[1].transpose() << std::endl;
        std::cout << "vQd[2]: " << vQd[2].transpose() << std::endl;

        x_state_des_.pos() << vQd[0].head(3), Q.coeffs();

        tau = cartesianSpaceControl(state, x_state_des_, dt);
        return tau;
      }
      else
      {
        ROS_ERROR_STREAM("ImpedanceControl::update: Unknown control mode");
        return Vector6d::Zero();
      }

    }

    Vector6d ImpedanceControl::cartesianSpaceControl(
      const JointState &cur, const cc::CartesianState &des, double dt)
    {
      auto T_ef_0 = model_.T_ef_0(cur.q);
      auto J_ef_0 = model_.J_ef_0(cur.q);
      auto Jp_ef_0 = model_.Jp_ef_0(cur.q, cur.qp);

      // current cartesian state
      x_state_cur_.pos().linear() = T_ef_0.translation();
      x_state_cur_.pos().angular() = T_ef_0.rotation();
      x_state_cur_.vel() = J_ef_0*cur.qp;

      // control errors
      cc::CartesianVector X_delta;
      X_delta.linear() = x_state_cur_.pos().linear() -  x_state_des_.pos().linear();
      Eigen::AngleAxisd aa(x_state_cur_.pos().angular()*des.pos().angular().inverse());
      X_delta.angular() = aa.angle()*aa.axis();

      // ROS_INFO_STREAM_THROTTLE(1.0, "setting X_delta to: " << X_delta.transpose());

      cc::CartesianVector Xp_delta;
      Xp_delta = x_state_cur_.vel() - x_state_des_.vel();

      // references
      x_state_ref_.vel() = des.vel() - Kp_cart_*X_delta;
      x_state_ref_.acc() = des.acc() - Kp_cart_*Xp_delta;

      // conversion back to jointspace
      auto J_ef_0_pinv = computeDampenedJacobianInverse(J_ef_0);      
      q_state_ref_.qp = J_ef_0_pinv*x_state_ref_.vel();
      q_state_ref_.qpp = J_ef_0_pinv*(x_state_ref_.acc() - Jp_ef_0*cur.qp);

      // control action
      Vector6d S_q = cur.qp - q_state_ref_.qp;
      return -Kd_ * S_q + computeYrTh(S_q, cur, q_state_ref_, dt);
    }

    Vector6d ImpedanceControl::computeYrTh(const Vector6d &S_q, const JointState &cur, const JointState &ref, double dt)
    {
      const auto& Yr = model_.regressor(cur.q, cur.qp, ref.q, ref.qp);
      theta_ -= gamma_ * Yr.transpose() * S_q * dt;
      return Yr * theta_;
    }

    Matrix6d ImpedanceControl::computeDampenedJacobianInverse(const cc::Jacobian& J, double lambda) 
    {
      return J.transpose() * (J * J.transpose() + lambda * cc::Jacobian::Identity()).inverse();
    }

    bool ImpedanceControl::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
