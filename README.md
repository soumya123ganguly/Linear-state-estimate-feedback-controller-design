# Linear-state-estimate-feedback-controller-design
We design our own linear state-estimate feedback controller for a Mobile Inverted Pendulum (MiP). We shall be considering the control of the Mobile Inverted Pendulum (MiP) as developed by UCSD Professor
Tom Bewley, his Coordinated Robotics Lab, and commercialized by WowWee Robotics.

# Files needed for this project : 

1. hw4 2015.pdf – Homework 4 for MAE280A in 2015. Part II of this homework, which starts out as above, introduces
the MiP by providing equations of motion for the device, their linearization, simplified analysis, motor equations,
measurement equations etc. Please read this document to familiarize yourself with the electromechanical set-up of
the MiP control problem and the notation that we use.

2. mipDerivations.pdf – a document commencing from the nonlinear MiP equations of motion and manipulating them to
yield a form suited to simulation in MATLAB.

3. mIpModel.mat – a matlab data file containing the parameters of a continuous-time linearized state-space model (ALCmIp,
BLCmIp,CLCmIp,DLCmIp]) and its discrete-time counterpart (ALDmIp, BLDmIp,CLDmIp,DLDmIp]) produced
via the MATLAB command c2d with 200Hz sampling rate and zero-order-hold methodology. Use MATLAB
command load mIpModel.mat in the command window to load these matrices with those names. Note that
the linearization is done about the zero state of the nonlinear system, which is stationary and balanced (unstably)
vertically.

4. MiP control.mat – a MATLAB data file containing Neeraj Dhole’s state feedback gain K and his observer gain L. These
are the gains in operation in the MiP which has been attending MAE280A lectures.

5. mIpParameters.m – MATLAB script defining the parameters of the MiP model for running the closed-loop nonlinear
simulation.

6. mIpCLwindy.slx – a closed-loop Simulink model of the nonlinear dynamics of the continuous-time MiP — inside the
grayed box— with
(i) input voltage to the motor,
(ii) reference additive alteration to the MiP input voltage to create interesting closed-loop signals,
(iii) wind torque disturbance to the MiP adding to $\ddot{\theta}$ directly,
(iv) outputs $\theta_t$ and $\phi_t$,
(v) a scope to show you (but not allow you to use) the entire MiP state.

# Tasks performed : 

0. Start analysis with the reference and wind generators yielding zero signals.
   
1. Using the linear discrete-time system model, show that it is both reachable and observable. Then use pole positioning
– first in the choice of linear state-variable feedback gain matrix K and then in linear output injection gain
matrix L – to design a linear state-estimate feedback controller for the linearized model.

2. Verify, using MATLAB’s feedback and lsim commands, that the discrete-time linear controller works as expected
on the discrete-time linear system. The discrete-time system has a sample rate of 200Hz.

3. Use the d2c command with the zero-order-hold method and feedback to verify that the continuous-time version
works similarly.. One can note that the nonlinear Simulink model includes a z-o-h.

4. Try the K and L to compute new matrices KLDmIp, LLDmIp, ALCBKLDmIP as in mIpParameters.m
and run the nonlinear Simulink closed-loop system.

(i) Make sure that the equilibrium is as expected. To do this:
i. Double-click (or Right-click and select Properties) on the reference and wind boxes and set the amplitudes
and biases to zero.
ii. Double-click on the Theta Noise and Phi Noise blocks and set the noise power to zero.
iii. Double-click on the digital controller box and ensure that the initial condition is zero.
iv. Double click on the gray mIp box to look inside.
v. There are five integrators inside – one for each state variable. Double-Click on each and set the initial
condition to zero.
vi. Run the simulation by clicking the run arrow. Inspect the state variables (which normally are not known
to us) and their estimates, the state of the controller. They should all stay at zero.

(ii) Start by changing the initial condition on theta (i.e. the theta integrator) – it should be measured in radians –
to see how well the linearized model captures the nonlinear behavior. Try to determine a range of allowable
theta initial conditions for stability of the nonlinear system.

(iii) Now include some reference driving signal by choosing non-zero amplitude and/or non-zero bias. Examine
the quality of the state estimates versus the actual state values as the signals increase in amplitude; firstly
within the linearization range, then beyond that range, then beyond the saturation range of the control signal.

(iv) Reset the theta initial condition to zero and turn up the theta noise power. This is the noise in the measurement
of theta but not in theta itself. We find the limiting value of this noise power before we lose stability.

(v) Vary the observer design to manage the response to the large noise power.

5. It is easy to test many different controller designs using this setup. Try to develop one that looks better than the one given.
   
6. Document, with plots and brief design explanation, our progress to our favorite controller paying particular
attention to the state estimator.

7. Examine and explain the effect of the wind torque on the behavior of the MiP, especially the change in equilibrium
for a constant (bias is the constant offset value inside the sinusoidal source) disturbance.

8. Explain how one would alter the design to return $\phi$ to zero at equilibrium – i.e. a homing MiP – first with
a constant wind and secondly with a constant plus sinusoidal wind disturbance.


# How to run the code to fulfill the tasks : 

1. First run the command 'load mIpModel.mat' in the command window of MATLAB to load the model of MiP.
2. Run TASK_CODE.m file to execute the tasks 0-8 (except task 7)
3. For executing task 7, open the Simulink model files.  


