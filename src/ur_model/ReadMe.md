# [C++] ur_model

## 待办事项
- 修改cmake, package, model_base文件
- 加上速度限制
- 写新的node
- 移动到ur_6dof

## 可能会出错的地方
- 



# [Matlab] ur_model
> **如果公式太长，VS Code自动切割为多行，按ALT+Z即可取消**


## 其他注意事项
- Y*Theta = M*Qpp + C*Qp + G  
  通过验证√
- 数组q, qp, qpp, qrp, qrpp
- cc::初始化
  - M, C, G, Theta, Y: 
  在ur_model.cpp初始化时已是全零矩阵  
  因此`auto_save.m`时`skipZeros = true;`
  - Transformation Matrix:  
  没初始化  
  因此`skipZeros = false;`  
  - Jacobian:  
  可以通过`cc::Jacobian::Zero();`初始化