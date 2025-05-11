# Medical Robot - Chapter 2: Kinematics

## 1. 平面和机构自由度

自由度计算公式（Gruebler–Kutzbach equation）：

$\[
F = 3(n - 1) - 2l - h
\]$

其中：

- \( n \)：构件总数（包含基座）
- \( l \)：lower pairs（低副，2D 约束 2 个自由度）
- \( h \)：higher pairs（高副，2D 约束 1 个自由度）

### 示例计算：

1. $\( F = 3(4-1) - 2 \times 4 - 0 = 9 - 8 = 1 \)$
2. $\( F = 3(6-1) - 2 \times 7 = 15 - 14 = 1 \)$
3. $\( F = 3(5-1) - 2 \times 5 = 12 - 10 = 2 \)$
4. $\( F = 3(5-1) - 2 \times 5 = 12 - 10 = 2 \)$
5. $\( F = 3(4-1) - 2 \times 4 = 9 - 8 = 1 \)$

---

## 2. 正运动学与逆运动学

- 正解：给定关节变量，计算末端位姿 $\( x = f(\theta) \)$
- 逆解：给定末端目标位姿，反解关节变量 $\( \theta = f^{-1}(x) \)$

---

## 3. 速度运动学与 Jacobian 矩阵

对正运动学函数两边求导：

$\[
x = f(\theta) \Rightarrow \dot{x} = J(\theta) \cdot \dot{\theta}
\]$

其中：

- $\( J(\theta) \)$：Jacobian 矩阵（表示关节速度对末端速度的映射）
- $\( \dot{x} \)$：末端速度
- $\( \dot{\theta} \)$：关节速度

---

### 示例：2R 平面机械臂

末端位置：

$\[
\begin{cases}
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) \\
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
\end{cases}
\]$

Jacobian 矩阵：

$\[
J(\theta_1, \theta_2) =
\begin{bmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} \\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2}
\end{bmatrix}$
=
$\begin{bmatrix}
- L_1 \sin\theta_1 - L_2 \sin(\theta_1 + \theta_2) & -L_2 \sin(\theta_1 + \theta_2) \\
L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) & L_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
\]$

速度变换：

$\[
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}$
=
$J(\theta_1, \theta_2)
\begin{bmatrix}
\dot{\theta}_1 \\
\dot{\theta}_2
\end{bmatrix}
\]$

---

## 4. 奇异位形（Singularities）

当 Jacobian 矩阵行列式为 0 时，矩阵不可逆 ⇒ 无法从末端速度反解出关节速度，系统丧失某些方向的运动能力。

\[
\dot{\theta} = J^{-1} \dot{x},\quad \det(J) = 0 \Rightarrow J^{-1} \text{ 不存在}
\]

例如：

\[
\det(J) = -L_1 \sin\theta_1 L_2 \cos(\theta_1 + \theta_2) + L_1 \cos\theta_1 L_2 \sin(\theta_1 + \theta_2)
\]

进一步化简：

\[
\det(J) = L_1 L_2 \sin\theta_2
\]

当 \( \theta_2 = 0 \) 或 \( \pi \) 时，行列式为 0，系统奇异。

---

## 5. Jacobian 也用于力映射

通过虚功原理，末端受力 \( F \) 与关节力矩 \( \tau \) 满足：

\[
\tau = J^T F
\]

推导依据：

- \( \delta x = J \cdot \delta \theta \)
- 虚功：\( F^T \delta x = \tau^T \delta \theta \)
- 推得：\( J^T F = \tau \)

---

## 6. 末端作用力的映射与控制

通过 Jacobian 可将：

- 末端力转为关节扭矩：\( \tau = J^T F \)
- 关节速度映射到末端速度：\( \dot{x} = J \dot{\theta} \)

---
