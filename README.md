# mechanical-week-3-task-2
smart methods internship week 3

# 3-DoF Robotic Arm Kinematics

A mathematical model of a three-degree-of-freedom robotic arm, focusing on both forward and inverse kinematics. 
Detailed transformation matrices and equations to understand and compute the arm's movements. 

# Forward Kinematics in 3D

Forward kinematics in 3D involves determining the position \((x, y, z)\) and orientation of the end-effector given the joint parameters.

## Assumptions
- The robot has three rotational joints.
- The links are defined by lengths L1, L2, and L3.
- The joint angles are θ1, θ2, and θ3.

## Denavit-Hartenberg (DH) Parameters
The DH convention simplifies the process of modeling robotic arms by assigning a coordinate frame to each joint and defining a set of four parameters that describe the relative positions of these frames. The four DH parameters are:

**𝑎𝑖 (Link Length):**
- The distance between the \(z_{i-1}\) and \(z_i\) axes measured along the \(x_i\) axis.
- This parameter represents the length of the common normal between the two successive joint axes.

**αi (Link Twist):**
- The angle between the \(z_{i-1}\) and \(z_i\) axes measured around the \(x_i\) axis.
- This parameter represents the twist angle between the two successive joint axes.

**di (Link Offset):**
- The distance between the \(x_{i-1}\) and \(x_i\) axes measured along the \(z_i\) axis.
- This parameter represents the offset along the previous \(z\)-axis to the common normal.

**θi (Joint Angle):**
- The angle between the \(x_{i-1}\) and \(x_i\) axes measured around the \(z_i\) axis.
- This parameter represents the rotation around the previous \(z\)-axis to align the \(x\)-axes.

## Defining DH Parameters for a 3-DoF Robot
Let's define the DH parameters for a 3-DoF robotic arm, which typically has three revolute joints. We'll explain how each parameter is derived based on the arm's configuration.

**Joint 1 (Base to Link 1)** 
- a1=0: Since the base is fixed and there is no offset along the \(x_1\) axis.
- 𝛼1=0: There is no twist angle between the base and the first joint.
- d1: This is the distance from the base to the first joint along the \(z_1\) axis, which is usually a fixed length.
- θ1: This is the rotation around the \(z_1\) axis, which is the first joint angle.

**Joint 2 (Link 1 to Link 2)**
- a2=L1: The length of the first link along the \(x_2\) axis.
- 𝛼2=0: There is no twist angle between the first and second joints.
- d2=0: There is no offset along the \(z_2\) axis for a revolute joint.
- θ2: This is the rotation around the \(z_2\) axis, which is the second joint angle.

**Joint 3 (Link 2 to Link 3)**
- a3=L2: The length of the second link along the \(x_3\) axis.
- 𝛼3=0: There is no twist angle between the second and third joints.
- d3=0: There is no offset along the \(z_3\) axis for a revolute joint.
- θ3: This is the rotation around the \(z_3\) axis, which is the third joint angle.

For a 3-DoF robot, the DH parameters might look like:

| Joint | ai | 	𝛼i | di | θi |
|-------|--------|--------------|--------|-------------|
| 1     | 0      | 0            | d1 | θ1 |
| 2     | L1 | 0            | 0      | θ2 |
| 3     | L2 | 0            | 0      | θ3 |

Using these parameters, the transformation matrices for each joint are:

$$ 
T_1 = \begin{bmatrix}
\cos(θ1) & -\sin(θ1) & 0 & 0 \\
\sin(θ1) & \cos(θ1) & 0 & 0 \\
0 & 0 & 1 & d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$ 
T_2 = \begin{bmatrix}
\cos(θ2) & -\sin(θ2) & 0 & L_1 \cos(θ2) \\
\sin(θ2) & \cos(θ2) & 0 & L_1 \sin(θ2) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$ 
T_3 = \begin{bmatrix}
\cos(θ3) & -\sin(θ3) & 0 & L_2 \cos(θ3) \\
\sin(θ3) & \cos(θ3) & 0 & L_2 \sin(θ3) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## Composite Transformation

The composite transformation matrix \(T\) from the base to the end-effector is:

$$ 
T = T1 \cdot T2 \cdot T3
$$

To find the position \((x, y, z)\) of the end-effector, we extract the translation components from the final transformation matrix:

$$ 
\begin{aligned}
x &= L1 \cos(θ1) + L2 \cos(θ1 + θ2) + L3 \cos(θ1 + θ2 + θ3) \\
y &= L1 \sin(θ1) + L2 \sin(θ1 + θ2) + L3 \sin(θ1 + θ2 + θ3) \\
z &= d_1
\end{aligned}
$$

# 3-DoF Robotic Arm Kinematics

## Forward Kinematics

The diagram below illustrates the forward kinematics process, where we determine the position of the end-effector given specific joint angles:

<div style="text-align: center;">
  <h2>Forward Kinematics</h2>
  <svg width="400" height="300">
    <line x1="200" y1="200" x2="200" y2="100" style="stroke:black;stroke-width:2" />
    <line x1="200" y1="100" x2="300" y2="100" style="stroke:black;stroke-width:2" />
    <line x1="300" y1="100" x2="350" y2="50" style="stroke:black;stroke-width:2" />
    <circle cx="200" cy="200" r="4" fill="red" />
    <circle cx="200" cy="100" r="4" fill="red" />
    <circle cx="300" cy="100" r="4" fill="red" />
    <circle cx="350" cy="50" r="4" fill="red" />
    <text x="180" y="220" font-family="Verdana" font-size="12">Base</text>
    <text x="180" y="95" font-family="Verdana" font-size="12">Joint 1</text>
    <text x="310" y="95" font-family="Verdana" font-size="12">Joint 2</text>
    <text x="360" y="50" font-family="Verdana" font-size="12">End-Effector</text>
  </svg>
</div>


# Inverse Kinematics in 3D

Inverse kinematics involves finding the joint angles θ1, θ2, and θ3 given the position \((x, y, z)\) of the end-effector.

## Steps to Solve Inverse Kinematics

1. **Calculate θ1:**

   The first joint angle θ1 can be found using:

   $θ1 = \arctan2(y, x)$

2. **Calculate the Position of the Wrist Center:**

   The wrist center is the position where the third link starts. This can be found as:

   $x_w = x - L_3 \cos(θ1 + θ2 + θ3)$
   
   $y_w = y - L_3 \sin(θ1 + θ2 + θ3)$
   
   $z_w = z - d_1$

4. **Calculate θ2 and θ3:**

   Using the wrist center coordinates, we can form a triangle with sides L1, L2, and the distance from the base to the wrist center \((r)\):

   $r = \sqrt{x_w^2 + y_w^2}$

   The angle θ2 can be found using the law of cosines:

   $\cos(θ2) = \frac{r^2 + L1^2 - L2^2}{2 L1 r}$
   
   $θ2 = \arccos\left(\frac{r^2 + L1^2 - L2^2}{2 L1 r}\right)$

   The angle θ3 can be determined as:

   $θ3 = \arctan2(z_w, \sqrt{x_w^2 + y_w^2}) - θ2$
