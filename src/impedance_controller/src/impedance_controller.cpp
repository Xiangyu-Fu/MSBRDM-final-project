#include <impedance_controller/impedance_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    ImpedanceControl::ImpedanceControl(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      init_q_goal_(Vector6d::Zero()),
      init_period_(100.0),
      delta_q_(Vector6d::Zero()),
      delta_qp_(Vector6d::Zero())
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("simple_effort_controller_data", 1);
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
      // TIME UPDATE
      //////////////////////////////
      ros::Time time_cur = ros::Time::now();
      double dt = (time_cur - time_prev_).toSec();
      time_prev_ = time_cur;
      running_time_ += dt;
      ROS_WARN_STREAM_THROTTLE(10, "Running Time [s]: " << time.tD());

      //////////////////////////////
      // INIT MODE
      //////////////////////////////
      if(control_mode_ == INIT)
      {
        if(time.tD() > init_period_)
        {
          control_mode_ = JOINT;
          q_start_ = state.q;
          q_goal_ = state.q;
          running_time_ = 0.0;
          ROS_WARN_STREAM("Switching to JOINT mode");
        }
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
        vQd = getJointPVT5(q_start_, q_goal_, running_time_, 5.0);

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

      //////////////////////////////
      // CARTESIAN MODE
      //////////////////////////////
      else if(control_mode_ == CARTESIAN)
      {
        // control torque
        Vector6d tau;
        tau.setZero();

        // poly spline
        VVector6d vQd;
        vQd = getJointPVT5(q_start_, q_goal_, running_time_, spline_period_);

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
      else
      {
        ROS_ERROR_STREAM("ImpedanceControl::update: Unknown control mode");
        return Vector6d::Zero();
      }

    }

    bool ImpedanceControl::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
