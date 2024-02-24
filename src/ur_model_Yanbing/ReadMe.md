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
- 将函数都写入ur_model.h和.cpp中
- 修改control中使用的model_函数名
- IK
- J_time(可能需要)


## 其他注意事项
- Y*Theta = M*Qpp + C*Qp + G  
  通过验证√
- 数组q, qp, qpp, qrp, qrpp
- cc::初始化
  - 也许需要在ur_model.cpp就对全部初始化？（提高效率）
  - Transformation Matrix: 3*3单位矩阵+