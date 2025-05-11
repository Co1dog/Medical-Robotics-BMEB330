# Medical Robot - Chapter 2: Kinematics - 1

## 1. 平面和机构自由度

自由度计算公式：

$$
F = 3(n - 1) - 2l - h
$$

其中：

- $n$：构件总数（含基座）  
- $l$：lower pairs（低副）  
- $h$：higher pairs（高副）  

### 示例：

1. $$F = 3(4-1) - 2 \times 4 - 0 = 9 - 8 = 1$$  
2. $$F = 3(6-1) - 2 \times 7 = 15 - 14 = 1$$  
3. $$F = 3(5-1) - 2 \times 5 = 12 - 10 = 2$$  
4. $$F = 3(5-1) - 2 \times 5 = 12 - 10 = 2$$  
5. $$F = 3(4-1) - 2 \times 4 = 9 - 8 = 1$$  

---

## 2. 正/逆运动学基本定义

- **正运动学**：已知关节变量 $\theta$，求末端位置 $x$，即 $x = f(\theta)$  
- **逆运动学**：已知末端位姿 $x$，反解关节变量 $\theta$，即 $\theta = f^{-1}(x)$  

---

## 3. 速度映射与 Jacobian

对正运动学微分：

$$
\dot{x} = J(\theta) \cdot \dot{\theta}
$$

其中：

- $J(\theta)$：Jacobian 矩阵  
- $\dot{x}$：末端速度  
- $\dot{\theta}$：关节速度  

---

### 示例：2R 平面机械臂

末端位置：

 $$
\begin{cases}
x = L_1 \cos \theta_1 + L_2 \cos(\theta_1 + \theta_2) \\
y = L_1 \sin \theta_1 + L_2 \sin(\theta_1 + \theta_2)
\end{cases}
 $$

### Jacobian 矩阵

$$
\begin{aligned}
J(\theta_1,\theta_2)
&=
\begin{bmatrix}
\dfrac{\partial x}{\partial \theta_1} & \dfrac{\partial x}{\partial \theta_2}\\
\dfrac{\partial y}{\partial \theta_1} & \dfrac{\partial y}{\partial \theta_2}
\end{bmatrix}
&=
\begin{bmatrix}
-L_1\sin\theta_1 - L_2\sin(\theta_1+\theta_2) & -L_2\sin(\theta_1+\theta_2)\\
L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) & L_2\cos(\theta_1+\theta_2)
\end{bmatrix}
\end{aligned}
$$


### 速度关系

$$
\begin{bmatrix}
\dot{x}\\\dot{y}
\end{bmatrix}
&=
J(\theta_1,\theta_2)\,
\begin{bmatrix}
\dot{\theta}_1\\\dot{\theta}_2
\end{bmatrix}
$$


---

## 4. 奇异位形（Singularities）

当 $\det(J)=0$，Jacobian 不可逆，表示：

- 无法反解 $\dot{\theta}$，系统丧失某些方向的运动能力  
- 存在控制死区或刚体退化  

具体到 2R 系统：

 $$
\det(J)=L_1L_2\sin\theta_2
 $$

当 $\theta_2=0$ 或 $\pi$ 时， $\det(J)=0$，系统奇异。


---

## 5. 力-力矩关系（虚功原理）

由虚功原理推导出：

$$
\tau = J^T F
$$

推导步骤：

- 虚功守恒： $F^T \delta x = \tau^T \delta \theta$ 
- 代入 $\delta x = J \delta \theta$ 得：

$$
\tau = J^T F
$$

---

## 📌 符号说明

- $J(\theta)$：Jacobian 矩阵  
- $\dot{x}$：末端速度  
- $\dot{\theta}$：关节速度  
- $\tau$：关节力矩  
- $F$：末端力  
