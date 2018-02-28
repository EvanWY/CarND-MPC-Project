## The Model
The model take in vehicle pose (position x,y, yaw psi) and speed, rotational speed, and generate control for next time frame.
## Timestep Length and Elapsed Duration (N & dt)
0.17 second of timestamp length with N=6, which is tune so that the prediction length will be within 1 second, which will perform well in this track (can't be too long because the trajectory given by the simulator is not long enough)
## MPC Preprocessing
All data are transform into local coordinate of the vehicle, where the vehicle faces to the direction (1, 0). Which make it much easier to compute cte (just compare y value) and psi (just use atan(dy/dx) )
## Latency
The control data (a and delta) of the first frame are feed into the system, and I take the result from the second frame in the optimization result as control for this frame. So that the control will be delay and match the correct time frame.
