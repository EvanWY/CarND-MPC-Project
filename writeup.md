## The Model
The model take in vehicle pose (position x,y, yaw psi) and speed, rotational speed, and generate control for next time frame.

``` c++
fg[0] += cte1*cte1 // meter
    + (10*epsi1) * (10*epsi1) // 
    //+ (v1-50)*(v1-50) // optimize speed to as close to 50 as possible
    - v1 // optimize speed to as fast as possible
    //+ (60*delta0) * (60*delta0) // delta : [-0.49, 0.49] // optimize delta0
    //+ (2*a0) * (2*a0); // a: [-1, 1] // optimize a0
    + ((v0*v0 / Lf * delta0 * v0*v0 / Lf * delta0) + a0*a0/100) / 100 // instad of seperating delta0 with a0, I decided to combine both acceleration and limit the combined force.
    ;

fg[0] += (delta1 - delta0) * (delta1 - delta0) * 4 // optimize turning rate changing rate
    + (a1 - a0) * (a1 - a0) * 0.02; // optimize acceleration changing rate
```

Parameters:
1. **cte ^ 2** (cte square is used as standard unit to compare to other paremeters)
1. **(10 * epsi) ^ 2** (epsi is multiply by a fector of 10, so match with cte)
1. **-v** (initially, I used _(v-50)^2_, however I found it possible to optimize with "as fast as possible" as the goal with proper constrains)
1. **(v^2 * delta / Lf)^2 + (a/10)^2** (This is the Total Force combining thruttle and turning. This simulate the physics that cause us to hit the break before enter a coner)

## Timestep Length and Elapsed Duration (N & dt)
0.015 second of timestamp length with N=60, which is tune so that the prediction length will be within 1 second, which will perform well in this track (can't be too long because the trajectory given by the simulator is not long enough)
## MPC Preprocessing
All data are transform into local coordinate of the vehicle, where the vehicle faces to the direction (1, 0). Which make it much easier to compute cte (just compare y value) and psi (just use atan(dy/dx) )
## Latency
The control data (a and delta) of the first frame are feed into the system, and I take the result from the second frame in the optimization result as control for this frame. So that the control will be delay and match the correct time frame.
