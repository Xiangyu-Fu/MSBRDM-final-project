# [Matlab] ur_model
> **如果公式太长，VS Code自动切割为多行，按ALT+Z即可取消**

## 已完成
- **自动Matlab Symbolic转换C++表达式函数**  
  （那一年，所有的工人都在抵制珍妮纺纱机×
- FK(Transformation Matrix)
- Jacobian
- M, C, G
- Regressor
- Jacobian_dot

## 代办
- 切换DH
- IK
- J_time(可能需要)


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