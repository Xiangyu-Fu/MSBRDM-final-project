# Matlab文件用途

**caculet.m** 矩阵的自动计算 
  - Transformation Matrix
  - Jacobian, Jacobian_dot
  - M, C, G  

**caculet_save.m** 用于保存M,C,G（计算时间过长）
**Regressor_6DOF.m** Theta, Yr计算器
**Y.mat, Yr.mat, Theta.mat** 计算好的矩阵
**auto_save.m** 自动将matlab矩阵转为c++形式，并保存在.txt文件中
**check.m** 用于检查是否符合M*Qpp + C*Qp + G = Yr*Theta