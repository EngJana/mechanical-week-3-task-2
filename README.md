# mechanical-week-3-task-2

# Forward Kinematics in 3D

Forward kinematics in 3D involves determining the position \((x, y, z)\) and orientation of the end-effector given the joint parameters.

## Assumptions
- The robot has three rotational joints.
- The links are defined by lengths \(L_1\), \(L_2\), and \(L_3\).
- The joint angles are \(\theta_1\), \(\theta_2\), and \(\theta_3\).

## Denavit-Hartenberg (DH) Parameters
The DH convention simplifies the process of modeling robotic arms by assigning a coordinate frame to each joint and defining a set of four parameters that describe the relative positions of these frames. The four DH parameters are:

**\(a_i\) (Link Length):**
- The distance between the \(z_{i-1}\) and \(z_i\) axes measured along the \(x_i\) axis.
- This parameter represents the length of the common normal between the two successive joint axes.

**\(\alpha_i\) (Link Twist):**
- The angle between the \(z_{i-1}\) and \(z_i\) axes measured around the \(x_i\) axis.
- This parameter represents the twist angle between the two successive joint axes.

**\(d_i\) (Link Offset):**
- The distance between the \(x_{i-1}\) and \(x_i\) axes measured along the \(z_i\) axis.
- This parameter represents the offset along the previous \(z\)-axis to the common normal.

**\(\theta_i\) (Joint Angle):**
- The angle between the \(x_{i-1}\) and \(x_i\) axes measured around the \(z_i\) axis.
- This parameter represents the rotation around the previous \(z\)-axis to align the \(x\)-axes.

## Defining DH Parameters for a 3-DoF Robot
Let's define the DH parameters for a 3-DoF robotic arm, which typically has three revolute joints. We'll explain how each parameter is derived based on the arm's configuration.

**Joint 1 (Base to Link 1)** 
- \(a_1=0\): Since the base is fixed and there is no offset along the \(x_1\) axis.
- \(\alpha_1=0\): There is no twist angle between the base and the first joint.
- \(d_1\): This is the distance from the base to the first joint along the \(z_1\) axis, which is usually a fixed length.
- \(\theta_1\): This is the rotation around the \(z_1\) axis, which is the first joint angle.

**Joint 2 (Link 1 to Link 2)**
- \(a_2=L_1\): The length of the first link along the \(x_2\) axis.
- \(\alpha_2=0\): There is no twist angle between the first and second joints.
- \(d_2=0\): There is no offset along the \(z_2\) axis for a revolute joint.
- \(\theta_2\): This is the rotation around the \(z_2\) axis, which is the second joint angle.

**Joint 3 (Link 2 to Link 3)**
- \(a_3=L_2\): The length of the second link along the \(x_3\) axis.
- \(\alpha_3=0\): There is no twist angle between the second and third joints.
- \(d_3=0\): There is no offset along the \(z_3\) axis for a revolute joint.
- \(\theta_3\): This is the rotation around the \(z_3\) axis, which is the third joint angle.

For a 3-DoF robot, the DH parameters might look like:

| Joint | \(a_i\) | \(\alpha_i\) | \(d_i\) | \(\theta_i\) |
|-------|--------|--------------|--------|-------------|
| 1     | 0      | 0            | \(d_1\) | \(\theta_1\) |
| 2     | \(L_1\) | 0            | 0      | \(\theta_2\) |
| 3     | \(L_2\) | 0            | 0      | \(\theta_3\) |

Using these parameters, the transformation matrices for each joint are:

\[ 
T_1 = \begin{bmatrix}
\cos(\theta_1) & -\sin(\theta_1) & 0 & 0 \\
\sin(\theta_1) & \cos(\theta_1) & 0 & 0 \\
0 & 0 & 1 & d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

\[ 
T_2 = \begin{bmatrix}
\cos(\theta_2) & -\sin(\theta_2) & 0 & L_1 \cos(\theta_2) \\
\sin(\theta_2) & \cos(\theta_2) & 0 & L_1 \sin(\theta_2) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

\[ 
T_3 = \begin{bmatrix}
\cos(\theta_3) & -\sin(\theta_3) & 0 & L_2 \cos(\theta_3) \\
\sin(\theta_3) & \cos(\theta_3) & 0 & L_2 \sin(\theta_3) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

## Composite Transformation

The composite transformation matrix \(T\) from the base to the end-effector is:

\[ 
T = T_1 \cdot T_2 \cdot T_3
\]

To find the position \((x, y, z)\) of the end-effector, we extract the translation components from the final transformation matrix:

\[ 
\begin{aligned}
x &= L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3) \\
y &= L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3) \\
z &= d_1
\end{aligned}
\]

# Inverse Kinematics in 3D

Inverse kinematics involves finding the joint angles \(\theta_1\), \(\theta_2\), and \(\theta_3\) given the position \((x, y, z)\) of the end-effector.

## Steps to Solve Inverse Kinematics

1. **Calculate \(\theta_1\):**

   The first joint angle \(\theta_1\) can be found using:

   \[
   \theta_1 = \arctan2(y, x)
   \]

2. **Calculate the Position of the Wrist Center:**

   The wrist center is the position where the third link starts. This can be found as:

   \[
   x_w = x - L_3 \cos(\theta_1 + \theta_2 + \theta_3)
   \]
   \[
   y_w = y - L_3 \sin(\theta_1 + \theta_2 + \theta_3)
   \]
   \[
   z_w = z - d_1
   \]

3. **Calculate \(\theta_2\) and \(\theta_3\):**

   Using the wrist center coordinates, we can form a triangle with sides \(L_1\), \(L_2\), and the distance from the base to the wrist center \((r)\):

   \[
   r = \sqrt{x_w^2 + y_w^2}
   \]

   The angle \(\theta_2\) can be found using the law of cosines:

   \[
   \cos(\theta_2) = \frac{r^2 + L_1^2 - L_2^2}{2 L_1 r}
   \]
   \[
   \theta_2 = \arccos\left(\frac{r^2 + L_1^2 - L_2^2}{2 L_1 r}\right)
   \]

   The angle \(\theta_3\) can be determined as:

   \[
   \theta_3 = \arctan2(z_w, \sqrt{x_w^2 + y_w^2}) - \theta_2
   \]
