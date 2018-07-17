# Project Writeup CarND PID Control
## Author: Bugra Turan, 22.06.2018

### Introduction
This is the writeup for the PID controller Project of the Udacity CarND term 2. First, we will go through the implementation step-by-step. Afterwards, the parameter search will be discussed and possible ways to improve will be shown.

#### 1) Implementation
As it was shown in the lecture the required steps for the PID control process were implemented. This means that in the defined PID class the init was added as well as the update method. The init method sets the given PID parameters Kp, Ki, and Kd. In the update function the given crosstrack error value cte is used to update the class members p_error, i_error, and d_error. These values are then used in the method TotalError() to calculate the next steering_angle value that gives the best value to reduce the error.

In the main.cpp the cte value is read from the simulator and used in the PID controller class object pid. This object return the appropriate steering angle and if it is between -1 and 1 passed back to the simulator. The steering angle is clipped to these min and max values as well. 

Lastly, on top of the main() function the determined PID coefficients are defined. In the next step I will describe how these values were determined.

#### 2) PID Controller Automation
The original main.cpp was heavily edited to fit the required behavior for my parameter search. The defined PID coefficients are only used if there are no execute arguments are given. Otherwise a boolean called study is set to true. This bool is passed to the PID class as well. More specifically the update function of this class UpdateError() checks this boolean and constantly checks if the crosstrack error is lower than a constant called MAX_CTE and the simulation time is bigger than a certain margin. If it is too big then another boolean of the PID class called dead is set to true indicating that the car is too far of the track and that the simulation needs to stop. Please note that during runtime also the average crosstrack error avg_cte is calculated as well.

Back in the main.cpp this value avg_cte is used as the result when the simulation ends. The abort criteria is checked within the main() function. Here, we check if the boolean pid.dead is true, indicating that the car is stuck, or if there are more than 2000 timesteps which corresponds to a whole lap. If the condition is met the simulation is reset back to its beginning, the average crosstrack error divided by the timesteps, and the investigated PID coefficients are saved into the study_output.txt file. Finally the program closes itself by closing the connection and return.

#### 3) Parameter Search
Now we have a completely automated PID controller simulation which can be started by passing the desired PID coefficients Kp, Ki, and Kd. The controller runs until it either reaches one lap or the crosstrack error is too big. The average CTE is then saved into a output file and everything is set back. For a parameter search I have implemented a Python script that triggers the PID controller binary pid with the appropriate arguments. I have implemented the Twiddle method from the lectures and tried many coefficients in the same manner.

So how it works:
1. Run the simulator ->
2. Execute the python script run_study.py ->
3. The coefficient ranges will be sweeped according to the Twiddle method.

#### 3b) Impact of PID Coefficients
The impact of the different coefficients are as followed. The parameter Kp ist often called the P-Part of the PID controller or proportional part. This means that the controller tries to reduce the CTE by steering proportional in the reverse direction. The strength is determined by the Kp value. The problem is that if the parameter is too big there will be an overshoot and the CTE will oscillate.

The next parameter is the value Kd. This coefficient is related to the differential of the error and called D-Part. Therefore, a damping effect can be observed and the oscillations by the Kp value can be reduced. However, if the value is too big the dynamics of the system will be reduced due to the strong damping.

Lastly, the value of Ki is called the integral or I-Part of the controller. This is helpful is there is an systematic error like a constant drift of the wheels. The bigger the steady state error is the stronger the error reduction plays into the CTE reduction. A drawback is that, like the P-part, if the coefficient is too big the oscillations of the CTE will increase as well.


#### 4) Results
I have put a lot of extra work into the project to automatically determine the best PID parameters but found out that there is still some coarse tuning required. My best results were achieved with the values set as default in my programm, namely:

* Kp = 0.12;
* Ki = 0.001;
* Kd = 3.5;

If we use the Twiddle method from 0, 0, 0 and a starting delta of 1, 1, 1 there is no proper metric to optimize into the correct direction that can be used over such a large range. Furthermore, the I part is very sensitive even for very small numbers. I have decied to set it to 0.0 and optimize only P and D. The coefficients can then be optimizied nicely by an evaluation of the results file.

#### 5) Summary
This project was quite fun. Especially the part to automate to simulator and PID controller for parameter optimization. For the parameter exploration part I have to say that Twiddle is not really suitable if the simulation loop is too big and that more sophisticated global optimizers could perform better. My idea would be a genetic algorithm or even a neural network.





