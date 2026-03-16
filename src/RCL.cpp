/*
Overview:
This file will contain the RCL class, which will implement the RCL algorithm as follows:
1. Get a sample from each distance sensor
2. Get the estimated position of the robot
3. Raycast from the estimated position, getting what the distance sensors should be reading
4. Compare the raycasted values to the actual distance sensor values, and calculate the error
5. Use that error to update the estimated position of the robot


specifics:
- to raycast, we will use line intersection, to find where the ray and to borders/obstacles intersect
- We will assume that the IMU is correct


things to figure out
-> How to use the error to transform the est. position
	Math for finding sensor pos:
        sp.x=botX+cos(θ)⋅offset
        sp.y=botY+sin(θ)⋅offset
    Then raycasts to all walls, to find the closest wall and uses it.
	    If the sensor is looking at a North/South wall (Y-axis), it calculates: 
            ResultY=WallY−(sin(Angle)⋅val)
        If the sensor is looking at an East/West wall (X-axis), it calculates:
            ResultX=WallX−(cos(Angle)⋅val)
    This gives the robot a "suggested" coordinate. It then subtracts the sensor's 
    offset one last time to find where the center of the robot is.
    Finally it filters if trust is to low:
        - bad angle
        - bad distance reading (too far, or too close)
        - bad intersection (obstacle in the way, or at a corner)


Code layout:
We will have a RCL class, with the following methods:
* Constructor() - Initializes the class, and takes in the 4 distance sensors, the IMU, as well as a vector of coordinates that makes up the obstacles.
* dynamicUpdate() - this will be called every loop, and will perform the RCL algorithm
* staticUpdate(int samples) - A function for getting non-moving distance sensor updates, will take samples number of readings, and will average them to get a more accurate reading
* isValid() - this will check if the reading of the distance sensors is valid

*/

