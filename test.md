Abstract
========

This document presents a model prective control – MPC – algorithm aiming
to autonoumously drive an Unmanned Surface Vehicle – USV – towards a set
of waypoints. The design of the algorithm has been thinked to be robust
to environmental disturbances encountered in the marine environment.

The modelling of the USV and disturbances have been simplified as this
work is aimed to be a proof of concept: a more accurate modelling should
be considered for a real world implementation.

Introduction
============

This work follows a previous one where the goal was to autonomously
control an USV over a set of waypoints. A MPC algorithm with constant
position reference - i.e. the current waypoint - tracking was
implemented but this control scheme was not achieving good tracking
performances. The problem could come from the optimization that was done
over the position variables in the global frame which expressions are
non-linear. Thus it was decided to recast the MPC strategy to make the
optimization simpler using angular and forward speed variables – which
expression are fully linear in the boat’s frame - into the optimization
problem.

In this new version, the algorithm tries to track both angle and forward
speed references over a constant time horizon. The boat model was
rewritten in a simpler but yet accurate way and an "intelligent"
reference providing algorithm was written to make the system more robust
to disturbances.

The first point that will be addressed is the modelling of the USV,
followed by the tracking strategy used. Then the reference providing
algorithm - including dealing with disturbances - will be explained and
finally the MPC algorithm and the results obtained.

USV modelling
=============

This part explains the physics used to describe the USV behavior and how
this was modeled so it can be used into an MPC scheme.

Physical description
--------------------

The USV studied here has a catamaran-like structure with two static
propellers at the end of each hull.

![](media/image1.png){width="4.253472222222222in"
height="2.686111111111111in"}The figure below provides a graphic view of
the boat and the forces that applies to him.

Modelling
---------

Using Newton's second law in translation and rotation the linear and
angular accelerations can be written in the boat local frame:

$$\left\{ \begin{matrix}
m\ddot{x} = U_{2} + U_{2} - k\dot{x} \\
I\ddot{\theta} = \frac{D}{2}(U_{1} - U_{2}) \\
\end{matrix} \right.\ $$

The MPC algorithm will need to track a forward-speed and an angle
reference, thus a model where those are available is required.

The state-space formalism will be used to describe the system and the
state and input vectors are stated as follow[^1]:

$$X = \begin{pmatrix}
\ddot{\theta} \\
\dot{\theta} \\
\begin{matrix}
\theta \\
\dot{x} \\
\ddot{x} \\
\end{matrix} \\
\end{pmatrix},\ U = \begin{pmatrix}
U_{1} \\
U_{2} \\
\end{pmatrix}$$

Then a simple discrete-time state-space system is written:

$$X_{k + 1} = \begin{pmatrix}
\begin{matrix}
1 \\
T_{e} \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
1 \\
\begin{matrix}
T_{e} \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
1 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
1 \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
1 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}X_{k} + \begin{pmatrix}
\begin{matrix}
\frac{D}{2I} \\
0 \\
\begin{matrix}
0 \\
0 \\
\frac{1}{m} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\frac{- D}{2I} \\
0 \\
\begin{matrix}
0 \\
0 \\
\frac{1}{m} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}U_{k} = AX_{k} + BU_{k}$$

As explained in an optimization made on input increments will be more
tractable for tracking problems. So the system was extended as follow:

$$X = \begin{pmatrix}
\ddot{\theta} \\
\dot{\theta} \\
\begin{matrix}
\theta \\
\dot{x} \\
\ddot{\begin{matrix}
x \\
U_{1} \\
U_{2} \\
\end{matrix}} \\
\end{matrix} \\
\end{pmatrix},\ U = \begin{pmatrix}
{U}_{1} \\
{U}_{2} \\
\end{pmatrix}$$

$$X_{k + 1} = \begin{pmatrix}
A & B \\
0 & I \\
\end{pmatrix}X_{k} + \begin{pmatrix}
0 \\
I \\
\end{pmatrix}{U}_{k}$$

Tracking
========

The past section dealt with variables described in the boat local frame.
But the boat position needs to be estimated - or measured - in order to
drive it towards the waypoint. To achieve this a technique known as
"dead reckoning" has been used.

Dead reckoning
--------------

Technically dead reckoning is equivalent to integrating the boat's speed
over one time step with the actual position as initial condition.

As the velocity of the boat is obtained in its own local frame, its
position in the global frame depends on the angle of the boat.

Again, a state-space system will be used to implement this. However,
this system will be non-linear[^2]:

$$X = \begin{pmatrix}
X \\
Y \\
\begin{matrix}
\theta \\
\dot{\theta} \\
\begin{matrix}
\ddot{\theta} \\
\dot{x} \\
\ddot{x} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix},\ U = \begin{pmatrix}
U_{1} \\
U_{2} \\
\end{pmatrix}$$

$$f\left( X,U \right) = \begin{pmatrix}
X + T_{e}\dot{x}\cos\theta \\
Y + T_{e}\dot{x}\sin\theta \\
\begin{matrix}
\theta + T_{e}\dot{\theta} \\
\dot{\theta} + T_{e}\ddot{\theta} \\
\begin{matrix}
\ddot{\theta} + \frac{D}{2I}(U_{1} - U_{2}) \\
\dot{x} + T_{e}\ddot{x} \\
\ddot{x} + \frac{1}{m}(U_{1} + U_{2} - k\dot{x}) \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

This set of state equations are linearized around the boat current angle
to obtain:

$$X_{k + 1} = \begin{pmatrix}
\begin{matrix}
1 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
1 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
1 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
T_{e} \\
1 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
\begin{matrix}
1 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
T_{e}\cos\theta \\
T_{e}\sin\theta \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
1 \\
 - k/m \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
1 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}X_{k} + \begin{pmatrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
\frac{D}{2I} \\
0 \\
\frac{1}{m} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
 - \frac{D}{2I} \\
0 \\
\frac{1}{m} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}U_{k}$$

Kalman filters
--------------

The previous paragraph provides a model to estimate the boat's position.
However in real world applications a GPS and a compass could be used to
obtain this data.

A Kalman filter could be developed to compute a better estimate of the
position using both model and sensors data. This is known as sensor
fusion.

Reference providing algorithm
=============================

When using a MPC algorithm, a reference over the prediction horizon is
needed. The goal of this algorithm is to provide such a reference and to
anticipate the environmental disturbances that could be encountered. In
this algorithm an angle reference is needed and the speed reference will
simply be a constant.

The algorithm goal is to provide an angle reference, knowing the USV
position and the environmental disturbances. It works on a fairly simple
way:

-   $\ While\ i\  < \ n_{y}:$

    -   $\text{\ Compute\ the\ angle\ between\ the\ USV\ position\ and\ the\ next\ waypoint.}$

    -   $\text{\ Create\ a\ }\mathrm{\text{virtual\ }}\text{state\ space\ model\ linearized\ around\ the\ angle\ previously\ computed.}$

    -   $\ Input\ to\ the\ ``virtual"\ model\ two\ equal\ thrusts\ computed\ to\ have\ the\ desired\ forward\ speed.$

    -   $\text{\ Add\ the\ angle\ to\ the\ reference\ vector.}$

    -   $\ If\ the\ boat\ arrived\ to\ the\ waypoint,\ consider\ the\ next\ waypoint.$

    -   $\ \text{Increment\ i}$

-   $\text{\ End}$

Where $n_{y}$ is the MPC prediction horizon.

An illustrated example is provided below to help the understanding.

1.  The USV is in a random position.

2.  Compute the angle between the USV and the next waypoint and create a
    linearized model of the USV – represented with dashed lines.

3.  Input to this model constant and equal thrusts until it arrives to
    the waypoint and then recompute the angle to the next waypoint and
    so on.

This algorithm outputs an angle between minus pi and pi, thus, when the
difference between the current angle and the next computed is greater
than pi, this means that there is a shorter path to go than what the
algorithm outputs. A second algorithm is here to correct this.

It works as follow:

-   $Difference\  = \ abs(angle\ \ next\_ angle)$

-   $If\ difference\  > \ pi$

    -   $If\ angle\  < \ 0$

        -   $Next\_ angle\  = \ angle\ \ (2*pi\ \ difference)$

    -   $\text{Else}$

        -   $Next\_ angle\  = \ angle\  + \ (2*pi\ \ difference)$

    -   $\text{End}$

-   $\text{End}$

Disturbances
------------

In order to see the algorithm robustness to external disturbances, some
were added to the model.

It was assumed that the disturbances were constant and that they could
be measured with an arbitrary precision, which is obviously false in a
real scenario but more convenient here.

The disturbances in the global frame can be expressed as:

$$D_{k} = \begin{pmatrix}
D_{X} \\
D_{Y} \\
\end{pmatrix}$$

Let $\alpha$ be the angle of the $D_{k}$ vector from the $X$ axis[^3].
So in the boat frame this vector become:

$$D_{k} = \ \begin{pmatrix}
D_{x} \\
D_{y} \\
\end{pmatrix} = \begin{pmatrix}
D_{X}cos(\alpha - \theta) \\
D_{Y}sin(\alpha - \theta) \\
\end{pmatrix}$$

$y$ axis velocity and acceleration had to be add to the state-space
system used to provide the reference; so it became:

$$X = \begin{pmatrix}
X \\
Y \\
\begin{matrix}
\theta \\
\dot{\theta} \\
\begin{matrix}
\ddot{\theta} \\
\dot{x} \\
\begin{matrix}
\ddot{x} \\
\dot{y} \\
\ddot{y} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix},\ U = \begin{pmatrix}
U_{1} \\
U_{2} \\
\end{pmatrix}$$

With this state vector, the equations became:

$$f\left( X,U \right) = \begin{pmatrix}
X + T_{e}\dot{x}\cos\left( \theta \right) - T_{e}\dot{y}\sin\left( \theta \right) \\
Y + T_{e}\dot{x}\sin\left( \theta \right) + T_{e}\dot{y}\cos\left( \theta \right) \\
\begin{matrix}
\theta + T_{e}\dot{\theta} \\
\dot{\theta} + T_{e}\ddot{\theta} \\
\begin{matrix}
\ddot{\theta} + \frac{D}{2I}(U_{1} - U_{2}) \\
\dot{x} + T_{e}\ddot{x} \\
\begin{matrix}
\ddot{x} + \frac{1}{m}(U_{1} + U_{2} - k\dot{x} + D_{x}) \\
\dot{y} + T_{e}\ddot{y} \\
\ddot{y} + \frac{1}{m}(D_{y} - k\dot{y}) \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

And linearizing around $\theta$ gives the following Jacobians:

$$F_{X} = \begin{pmatrix}
\begin{matrix}
1 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
1 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
1 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
T_{e} \\
1 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
\begin{matrix}
1 \\
0 \\
\begin{matrix}
0 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
T_{e}cos(\theta) \\
T_{e}sin(\theta) \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
1 \\
\begin{matrix}
 - k/m \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
\begin{matrix}
1 \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
 - T_{e}sin(\theta) \\
T_{e}cos(\theta) \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
1 \\
 - k/m \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
T_{e} \\
1 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

$$F_{U} = \begin{pmatrix}
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
\frac{D}{2I} \\
0 \\
\begin{matrix}
\frac{1}{m} \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
 - \frac{D}{2I} \\
0 \\
\begin{matrix}
\frac{1}{m} \\
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

So the following recursive equation is obtained:

$$X_{k + 1} = F_{X}X_{k} + F_{U}U_{k} + \frac{1}{m}\begin{pmatrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
0 \\
0 \\
\begin{matrix}
D_{x} \\
0 \\
D_{y} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

Using this system into the reference providing algorithm will
automatically take disturbances into account as far as they are
measured.

MPC algorithm
=============

This part will describe the MPC algorithm and how it has been
implemented in Matlab.

The previous MPC control scheme was optimizing the boat’s position in
the plane to drive it to the next waypoint. This method has not given
good results as the trajectory was not optimal in any sense.

It is probable that this was produced by a bad optimization problem, as
the boat position in the global plane was given by a non-linear system –
due to the referential change between the boat frame and the global
frame.

Thus it has been decided to use variables easily expressed in the boat’s
frame. It has been noticed that boat angle and cruise speed should be
enough to control the boat. And as they can be expressed easily in the
boat’s frame they should give a more tractable optimization problem.

The extended state-space system described in the modelling part will be
used for the MPC algorithm – with the $D_{x}$ term of the disturbance
added. The standard $A,\ B$ notations will be used for its matrixes even
if they were already used for the non-extended system.

Also, the output matrix has been chosen to match the optimized
variables:

$$C = \begin{pmatrix}
\begin{matrix}
0 & 0 & \begin{matrix}
1 & 0 & \begin{matrix}
0 & 0 & 0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\begin{matrix}
0 & 0 & \begin{matrix}
0 & 1 & \begin{matrix}
0 & 0 & 0 \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

Cost function
-------------

The cost function used is the same than the one used in for the tracking
problem:

$$J = \ \frac{1}{2}\left( r_{k + N} - y_{k + N} \right)^{T}S\left( r_{k + N} - y_{k + N} \right) + \frac{1}{2}\sum_{n = 0}^{N - 1}\left( \left( r_{k + n} - y_{k + n} \right)^{T}Q\left( r_{k + n} - y_{k + n} \right) + {u}_{k + n}^{T}R{u}_{k + n}^{\ } \right)$$

Replacing the output term with its equation from the state-space
representation gives:

$$J = \frac{1}{2}\left( r_{k{+ n}_{y}} - CX_{k + n_{y}} \right)^{T}S\left( r_{k{+ n}_{y}} - CX_{k + n_{y}} \right) + \frac{1}{2}\sum_{n = 0}^{n_{y} - 1}\left\lbrack \left( r_{k + n} - CX_{k + n} \right)^{T}Q\left( r_{k + n} - CX_{k + n} \right) + {U}_{k + n}^{T}R{U}_{k + n} \right\rbrack$$

With $S,\ Q\ \mathrm{\text{and}}\mathrm{\ }R$ gain matrices.

It is show in that the cost function can be expressed in a form that can
be used into the $\text{quadprog}$ function of Matlab:

$$J = \frac{1}{2}x^{T}Hx + f^{T}x$$

Where $x$ is the optimization variable – i.e. the $n_{u}$ future input
increments written as$\ \overrightarrow{U}$.

It is also shown in that $H$ and $f$ can be expressed as follow:

$$\left\{ \begin{matrix}
H = \ {\overline{C}}^{T}\overline{Q}\ \overline{C} + \overline{R} \\
f^{T} = \left\lbrack {X_{k}}^{T}\ r^{T} \right\rbrack\begin{bmatrix}
{\overline{A}}^{T}\overline{Q}\ \overline{C} \\
 - \overline{T}\ \overline{C} \\
\end{bmatrix} \\
\end{matrix} \right.\ $$

With:

$$\overline{Q} = \begin{pmatrix}
\begin{matrix}
C^{T}\text{QC} \\
0 \\
\begin{matrix}
 \vdots \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \ddots \\
\begin{matrix}
 \ddots \\
\cdots \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
\cdots \\
 \ddots \\
\begin{matrix}
C^{T}\text{QC} \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \vdots \\
\begin{matrix}
0 \\
C^{T}\text{SC} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

$$\overline{C} = \begin{pmatrix}
\begin{matrix}
B \\
\text{AB} \\
\begin{matrix}
 \vdots \\
A^{n_{y} - 1}B \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
B \\
\begin{matrix}
 \ddots \\
\cdots \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
\cdots \\
 \ddots \\
\begin{matrix}
 \ddots \\
\text{AB} \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \vdots \\
\begin{matrix}
0 \\
B \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

$$\overline{R} = \begin{pmatrix}
\begin{matrix}
R \\
0 \\
\begin{matrix}
 \vdots \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
R \\
\begin{matrix}
 \ddots \\
\cdots \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
\cdots \\
 \ddots \\
\begin{matrix}
 \ddots \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \vdots \\
\begin{matrix}
0 \\
R \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

$$\overline{A} = \begin{pmatrix}
A \\
A^{2} \\
\begin{matrix}
 \vdots \\
A^{n_{y}} \\
\end{matrix} \\
\end{pmatrix}$$

$$\overline{T} = \begin{pmatrix}
\begin{matrix}
\text{QC} \\
0 \\
\begin{matrix}
 \vdots \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \ddots \\
\begin{matrix}
 \ddots \\
\cdots \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
\begin{matrix}
\cdots \\
 \ddots \\
\begin{matrix}
\text{QC} \\
0 \\
\end{matrix} \\
\end{matrix} & \begin{matrix}
0 \\
 \vdots \\
\begin{matrix}
0 \\
\text{SC} \\
\end{matrix} \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

And $X_{k}$ the current state vector and $r$ the reference vector.

Constraints
-----------

Constraints can be easily added to an MPC scheme as they are often
included into the optimization software. The $\text{quadprog}$ function
of Matlab has been used to solve the optimization problem, so
constraints of the following form have been added:

$$A\overrightarrow{U} \leq B$$

As shown in , let’s pose:

$$\left\{ \begin{matrix}
X \leq \overline{X} \\
X \geq \\
\end{matrix} \right.\ $$

So

$$\left\{ \begin{matrix}
X \leq \overline{X} \\
 - X \leq - \\
\end{matrix} \right.\ $$

What can be written as:

$$\begin{pmatrix}
I_{n_{y}} \\
 - I_{n_{y}} \\
\end{pmatrix}\overrightarrow{X} \leq \begin{pmatrix}
\overline{X} \\
 \vdots \\
\begin{matrix}
\overline{X} \\
 - \\
\begin{matrix}
 - \\
 \vdots \\
 - \\
\end{matrix} \\
\end{matrix} \\
\end{pmatrix}$$

Where $\overrightarrow{X}$ represents the vector of the $n_{y}$ future
states.

The following notation will be used:

$$C_{X}\overrightarrow{X} \leq d_{X}$$

And, as state predictions are given by:

$$\overrightarrow{X} = \overline{A}X_{k} + \overline{C}\ \overrightarrow{U}$$

The previous inequality can be written as:

$$C_{X}(\overline{A}X_{k} + \overline{C}\ \overrightarrow{U}) \leq d_{X}$$

Which gives:

$$C_{X}\overline{C}\ \overrightarrow{U} \leq d_{X} - C_{X}\overline{A}X_{k}$$

So in our constraint equation:

$$A = C_{X}\overline{C}$$

$$B = d_{X} - C_{X}\overline{A}X_{k}$$

Results
=======

This part will present the results obtained with the MPC algorithm as
described in this document.

The simulations used to validate this algorithm are based on a benchmark
trajectory of 10 waypoints. The USV has to reach the 10 waypoints in a
defined order. This test has been made with and without disturbances –
on the second figure the pink arrow represents the direction of the
disturbance.

In the figures below are represented – from left to right and up to
down: the USV trajectory with the waypoints, the thrust applied on each
motor, the forward speed and its reference, the angle from the state
vector used in the reference providing algorithm, the angle from the
state vector used in the MPC algorithm and the $y$ axis boat speed.

![](media/image3.png){width="5.188976377952756in"
height="2.6023622047244093in"}In the first figure below – simulation
without disturbances – the main objective is clearly reached: the
algorithm drives the boat around all the waypoints in a fairly optimized
way. However an offset is observed on the forward speed of the boat –
adding an integral gain may solve the problem. And an unexplained angle
drift arises around the 1000^th^ time step.

![](media/image4.png){width="5.224409448818897in"
height="2.6181102362204722in"}In the second figure – simulation with
disturbances – the main objective is also clearly reached. However,
besides the two kinks pointed out in the previous paragraph, two more
issued arises: as the distance from the “optimal” path is not optimized,
the MPC algorithm doesn’t care of its trajectory as long as all the
waypoints are reached thus leading to a non-optimal trajectory. Also,
the $y$ speed of the boat is not an optimization criterion so all the
disturbances pass through the MPC and directly affects the boat. The
problem is that the two motors are not controllable in angle, making the
correction of the $y$ axis speed difficult.

Appendices
==========

The code is available on GitHub:

https://github.com/ccalas/mpc/blob/master/final\_mpc.m

[^1]: The indices used for discrete time variables have been ignored in
    order to simplify the equations.

[^2]: The indices used for discrete time variables have been ignored in
    order to simplify the equations.

[^3]: See the USV figure in the « USV modelling » part for details.
