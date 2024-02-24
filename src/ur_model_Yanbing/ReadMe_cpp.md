# [C++] ur_model

## 待办事项
**现在**
- 用新的DH表计算矩阵表达式，并且替换
- 需要添加Joint广播部分
**未来**
- 修改control中使用的model_函数名
- 需要重写model_base.h



## 可能会出错的地方
- 省略的部分 请看`Back_UP.cpp`
- 重写了`Ti_0`函数，可能有问题


## 我的（名称）
**变量**
model_interface::ModelBase Base;  
typedef cc::MatrixX Regressor_theta;  
typedef cc::VectorX Regressor_Yr;  

Regressor_theta Theta_;
Regressor_Yr Yr_;

**函数**
**public 用于返回**
URModel(const std::string& name = "ur_model");  
virtual ~URModel();  
**virtual bool init(ros::NodeHandle& nh) override;**  

const cc::MatrixDof &Inertia_Matrix  
const cc::MatrixDof &Coriolis_Matrix  
const cc::MatrixDof &Gravity_Matrix  
const Regressor_Yr &regressor_Yr  
const Regressor_Theta &regressor_Theta();  
cc::HomogeneousTransformation Ti_0  
cc::HomogeneousTransformation Tcmi_0  
cc::Jacobian Ji_0  
cc::Jacobian Jcmi_0  
cc::Jacobian Ji_0_dot  
cc::Jacobian Jcmi_0_dot  




**private 用于更新**



