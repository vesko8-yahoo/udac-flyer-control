# The C++ Control Project Writeup #

This is the writeup for the C++ control project.

## The Tasks ##

For this project, you will be building a controller in C++.  You will be implementing and tuning this controller in several steps.

You may find it helpful to consult the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py) as a reference when you build out this controller in C++.

#### Notes on Parameter Tuning
1. **Comparison to Python**: Note that the vehicle you'll be controlling in this portion of the project has different parameters than the vehicle that's controlled by the Python code linked to above. **The tuning parameters that work for the Python controller will not work for this controller**

2. **Parameter Ranges**: You can find the vehicle's control parameters in a file called `QuadControlParams.txt`. The default values for these parameters are all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: In this [one-page document](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) you can find a derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

### Body rate and roll/pitch control (scenario 2) ###

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, you will:

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
 - implement the code in the function `BodyRateControl()`
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

If successful, you should see the rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.  Note that the vehicle will keep flying off quite quickly, since the angle is not yet being controlled back to 0.  Also note that some overshoot will happen due to motor dynamics!.

If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting `QuadControlParams.kpBank = 0`.

2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

The `GenerateMotorCommands()` implementation is from line 78 till 103. It kaes in the total thrust and the moments for each axis and converts those to commanded thrusts for each individual motor. It uses the drone's arm length to do this calculation. In the realw-rld each motor can produce a max thrust and also there is min thrust so I needed to make sure that the commaneded thrusts are within this range. I also played with the the `mass` parameter in the `QuadControlParams.txt`.

Next I implemented `BodyRateControl()` - see line 124 till 135 in `QuadControl.cpp`. The controller takes the p,q and r velocities and also the commanded p, q and r and converts them to commanded moments taking into account the error. This is essentially a P-conroller. 

Next I implemented the `RollPitchControl()` - see lines 163 till 200.
It takes in a commanded thrust, a desired acceleration for x, a desired acceleration for y and the attitude of the vehicle in Quaternions and uses them to compute the commaned velocity for x and the commanded velocity for y direction. It is essentially a P-Controller. It is using the `kpBank` parameter which I set to 16. Also, I had to play with the `kpPQR` parameters and eventually arrived at the following values `kpPQR = 81, 40, 10`. If negative thrust command is received then the x and y velocities are set to 0 since we cannot achieve negative thrust in the vehicle. Maybe producing negative thrust is possible if our vehicle had variable pitch propellers (propellers with different angles of their blades). For now we set the x and y velocities are set to 0 if negative commanded thrust is received. 

<p align="center">
<img src="animations/scenario2.gif" width="500"/>
</p>


### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 - implement the code in the function `AltitudeControl()`
 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

I implemented `AltitudeControl()` see lines 225 till 245. This takes in verticle postition and velocity and commanded posistion and velocity and also the commaned vertical acceleration and the vehicle's attitude and also a dt and computes the commanded thrust. This is a PD-Controller With Feedforward. 

I implemented `LateralPositionControl()`. For the x and y directions we are given position, velocity, and the corresponding commanded position, velocity and also the commaned acceleration. We use those to computed the commaned accelarations in the x and y directions. This is essentially a PID-Controller With Feed Forward. 

I also implemented YawControl - see lines 313 till 333. This is essentially a P-Controller. You have to account for the fact here there the bounds are between -pi and pi.

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

I implemented `AltitudeControl()` see lines 225 till 245. This takes in verticle postition and velocity and commanded posistion and velocity and also the commaned vertical acceleration and the vehicle's attitude and also a dt and computes the commanded thrust. We are using dt parameter now and that makes this controller and Integral Controller. 

Also, I had to play with the Kp parameters used  by the above contorllers namely `kpPosXY`, `kpPosZ`, `kiPosZ`, `kpVelXY`, `kpVelZ`, and `kpYaw`. These are what I settled on: `kpPosXY=2.5`, `kpPosZ=8`, `kiPosZ=10`, `kpVelXY=15`, `kpVelZ=18`,  and `kpYaw=2.3`.

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>
