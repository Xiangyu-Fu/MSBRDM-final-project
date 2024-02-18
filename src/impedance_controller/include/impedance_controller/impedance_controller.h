#ifndef UR_ROBOT_LLI_IMPEDANCECONTROL_H
#define UR_ROBOT_LLI_IMPEDANCECONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include "impedance_controller/MoveArmCartesian.h"
#include "impedance_controller/MoveArmJoint.h"
#include<ros/ros.h>

// FIXME: use blue_controller/BlueControl.h as a template
#include <ur_model/ur_model.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class ImpedanceControl : public ControlEffort
    {
    public:
      enum Mode {INIT, JOINT, CARTESIAN};
      
    private:
      bool is_first_iter_;

      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;

      Matrix6d Kp_;
      Matrix6d Kd_;
      Matrix6d Ki_;

      Vector6d q_goal_;
      double spline_period_;

      Vector6d delta_q_;
      Vector6d delta_qp_;

      // rosservices
      ros::ServiceServer move_arm_cartesian_service_;
      ros::ServiceServer move_arm_joint_service_;

      // global params
      Mode control_mode_;
      ros::Time time_prev_;
      double running_time_;

      // params for init procedure
      double init_period_;
      Vector6d init_q_goal_;

      // params for joint control
      JointState joint_state_;

      // FIXME: params for cartesian control
      ur::URModel model_; 
      cc::CartesianPosition ee_start_, ee_goal_;
      cc::CartesianState ee_pose_, x_state_des_;
      VectorXd theta_; 
      
    public:
      ImpedanceControl(double weight = 1.0, const QString &name = "ImpedanceControl");
      
      ImpedanceControl(double weight, const QString &name, ros::NodeHandle nh);
    
      ~ImpedanceControl();

      void setQInit(const JointState &q_init);

      void setQHome(const JointState &q_home);

      void setQPark(const JointState &q_park);

      // Two services
      bool moveArmCartesian(impedance_controller::MoveArmCartesian::Request &req, impedance_controller::MoveArmCartesian::Response &res);
      bool moveArmJoint(impedance_controller::MoveArmJoint::Request &req, impedance_controller::MoveArmJoint::Response &res);

    private:
      bool init();

      bool start();

      Vector6d update(const RobotTime &time, const JointState &state);

      Vector6d cartesianSpaceControl(const JointState &cur, const cc::CartesianState &des, double dt);

      Vector6d computeYrTh(const Vector6d &S_q, const JointState &cur, const JointState &ref, double dt);

      Matrix6d computeDampenedJacobianInverse(const cc::Jacobian& J, double lambda);

      bool stop();
    };

  } // namespace RobotControllers
} // namespace tuics_ur_robot_lli

#endif // UR_ROBOT_LLI_IMPEDANCECONTROL_H
