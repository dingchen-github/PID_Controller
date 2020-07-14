# CarND-Controls-PID
A project in the Self-Driving Car Engineer Nanodegree Program

---

## Project Goal

The goal of this project is to implement a PID controller for the steering angle to drive the car around a track autonomously. The PID parameters can be tuned manually or automatically, e.g. with twiddle.

The twiddle algorithm was taught in the lecture. My code implements it.

```
def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    while sum(dp) > tol:
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2*dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p, best_err
```

---

## Code Implementation

### PID.cpp

A class called PID is already provided, but not fully defined. I need to define the functions.

1. Init: Pass the variables to the Kp, Ki and Kd, and initialise the PID errors to be 0.
2. UpdateError: First calculate the d error, which is the difference between the current cte and the last one. Then pass the current cte to the p error. At last update the i error.
3. TotalError: Calculate the total error using p, i and d errors. This total error is the output for the variable to be controlled, in this project the steering angle.
4. PrintParam: Print Kp, Ki and Kd after finishing twiddle.

### main.cpp

#### Variable Definition (line 40-51)

Define the variables to be passed to h.onMessage.

|Variable|Meaning|
|:-------------:|:-------------:|
|p[3]|A list of Kp, Ki and Kd. If we want to use twiddle, initialise all three values to be zero. Otherwise define p with optimised parameters.|
|dp[3]|A list of twiddle parameters.|
|n|Number of simulation cycles in twiddle.|
|twiddle|A bool variable as a flag indicating using twiddle mode or not.|
|tol|Twiddle tolerance. The sum of dp's three values. If this tolerance is reached, then twiddle is completed.|
|err|Twiddle error. Sum of squares of cte (to be divided by n).|
|best_err|Best twiddle error.|
|it|Iterator for p and dp index.|
|new_run|Flag indicating whether we will begin a new simulation cycle for the next p and dp index.|
|minus|Flag indicating whether we need to subtract dp from p.|

#### Twiddle (line 76-150)

* If n is 0, initialise pid parameters to the new p values and pid errors to be zero. This is similar to the "call `make_robot` before every call of `run`" part in the twiddle algorithm.

* Use pid functions to update the errors and calculate the total error as the steering angle.

* Adjust throttle based on the speed and steering angle. When both speed and steering angle are big, brake the car. The steering angle is between -1 and 1, and I have observed that in most of the times, the car only need a steering value between -0.1 and 0.1, so we can consider an absolute angle bigger than 0.1 represents a sharp turn. For the speed part, we can adjust the threshold. When the PID parameters are not optimised, a threshld of 20 or even 15 can help keep the car on the track. After we have got the optimised parameters, the speed part doesn't matter any more, the car stays on the track driving between 32-35 MPH.

* If the car goes off track, reset the simulator. Through trial and error I found out the cte threshold of 4 to be good.

* Accumulate the twiddle error and the cycle number.

* When a certain cycle number is reached, we begin the twiddle part (line 96-137). Otherwise, just pass the steering value and speed to the simulator (line 138-143).

* Twiddle part:
  * Calculate the average error.
  * If this is a new simulation cycle, as flagged by new_run, we add dp[it] to p[it], and set new_run to false. The cout part helps us track p during the simulation.
  * If this is not a new simulation cycle, we compare the current twiddle error to the best error.
  * If we find a new best error, we update the best error, incease dp[it] by 10%, move on the next index, set new_run to true and set the minus flag to true.
  * If we didn't find a new best error, we need to check whether we need to subtract dp from p, with the help of the minus flag. If minus is false, we bring p[it] to the original value, reduce dp[it] by 10% and go on to the next cycle.
  * Last but not the lease, reset the twiddle error and cycle number. If the car stays on the track, then we don't need to reset the simulator, just let the car run further.


* Check whether we have reached the twiddle tolerance. If yes, set twiddle flag to be false, print the PID parameters and close the simulation session.

#### No Twiddle (line 151-164)

If twiddle flag is false, we enter the normal driving mode, with the PID parameters initialised before h.onMesage or after twiddle.

---

## Parameter Tuning

In the twiddle mode, I initialise p to be zero. But I don't need to set dp all to 1, because after "driving" on the track, I found out that:

* Kp is proportional to cte. If Kp is too big, the car drives from left to right to left with big oscillations.
* Kd is related to cte differences. It helps to reduce the oscillations.
* Ki is related to the accumlated errors. Ki needs to be much smaller than Kp and Kd, because accumlated errors would be bigger and bigger as the car drives along.

So I found the the most efficient way being to tune Kp in 0.1 step, Ki in 0.001 and Kd in 1. These are my initial values of dp.

Then I run the twiddle mode. With different speed limitations (see "Adjust throttle" part), I got the following results:

* speed limit 20, best error 0.00355002, p = {0.100, 0.00018, 1.0}
* speed limit 30, best error 0.00353408, p = {0.100, 0.00019, 1.8}
* speed limit 100, best error 0.00628974, p = {0.147, 0.00001, 1.8}

Since I want to let the car drive as fast as possible, I choose the last one. I set p to {0.147, 0.00001, 1.8} and enter the normal driving mode, and the car can drive a full lap.
