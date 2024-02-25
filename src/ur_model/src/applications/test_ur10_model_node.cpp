#include <ur_model/ur_model.h>

int main(int argc, char **argv)
{
  // setup ros stuff
  ros::init(argc,argv, "ur10_robot_model_test");
  ros::NodeHandle nh("~");

  // initalize the model (only done once!)
  ur::URModel model("ur10_model");

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
  ur::URModel::Regressor Y = model.Regressor_Yr(q, qp, qrp, qrpp);
  
  // Initalize the robots parameter vector
  ur::URModel::Parameters th = model.Regressor_Theta(); 

  // get the end-effector transformation wrt 0 frame
  Eigen::Affine3d T_ef_0 = model.Tef_0(q);

  // get the end-effector jacobain wrt 0 frame
  Eigen::Matrix<double,6,6> J_ef_0 = model.Jef_0(q);

  //----------------------------------------------------------------------------
  // print some stuff

  ROS_WARN_STREAM("q=\n" << q.transpose() << "\n");
  ROS_WARN_STREAM("T_ef_0=\n" << T_ef_0.matrix() << "\n");
  ROS_WARN_STREAM("J_ef_0=\n" << J_ef_0 << "\n");
  ROS_WARN_STREAM("th=\n" << th.transpose() << "\n");
  ROS_WARN_STREAM("Y=\n" << Y << "\n");

  return 0;
}