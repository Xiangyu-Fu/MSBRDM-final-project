#include <ur_model/ur_model.h>

int main(int argc, char **argv)
{
  // setup ros stuff
  ros::init(argc,argv, "ur10_robot_model_test");
  ros::NodeHandle nh("~");

  // initalize the model (only done once!)
  tum_ics_ur_model::URModel model("ur10_model");

  // load the intial parameters from the parameter server
  if(!model.initRequest(nh))
  {
    ROS_ERROR_STREAM("Error: initalizing model failed!");
    return -1;
  }

  //----------------------------------------------------------------------------
  // test some function calls

  cc::VectorDof q, qp, qrp, qrpp;
  q = M_PI*cc::VectorDof::Random();
  qp.setZero();
  qrp.setZero();
  qrpp.setZero();
  
  // Initalize the robot regressor matrix
  tum_ics_ur_model::URModel::Regressor_Yr Yr = model.Yr_function(q, qp, qrp, qrpp);
  
  // Initalize the robots parameter vector
  tum_ics_ur_model::URModel::Regressor_Theta Theta = model.Theta_function(); 

  // get the end-effector transformation wrt 0 frame
  Eigen::Affine3d Tef_0 = model.Tef_0(q);

  // get the end-effector jacobain wrt 0 frame
  Eigen::Matrix<double,6,6> Jef_0 = model.Jef_0(q);

  //----------------------------------------------------------------------------
  // print some stuff

  ROS_WARN_STREAM("q=\n" << q.transpose() << "\n");
  ROS_WARN_STREAM("Tef_0=\n" << Tef_0.matrix() << "\n");
  ROS_WARN_STREAM("Jef_0=\n" << Jef_0 << "\n");
  ROS_WARN_STREAM("Theta=\n" << Theta.transpose() << "\n");
  ROS_WARN_STREAM("Yr=\n" << Yr << "\n");

  return 0;
}