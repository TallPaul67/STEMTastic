# STEMTastic
This is code intended for First Lego League (FLL) Robotics code hosted on the Spike Prime.  This is Python, actually Spike Python, which has a reduced set of functions and features.  

This code performed well for us, and got us to the state tournament in 2024.  

Requirements:
The wheel motors are configured in row 9.  Our code assumes Left is in port.A and right is in port.E.  The wheel is assumed to have a circumference of 7 inches (which is the smaller of the two Spike Prime wheels).  This is configured in row 12.  
The arm motors are configured in rows 30 and 31.  


It includes the following funtions:
* resetGyro().  We found we couldn't wait for the motion_sensor to get to a stable state, so made our own.  This allows for more prescise robot movement and turns.  
* gyroStraight().  Use this to drive the robot straight.  Forwards or backwards.  It is designed for smooth acceleration up to a given max speed, then smoothly decelerates to stop at the given distance.
* gyroStraightTime().  Instead of distance, this function moves the robot by time.  Smoothly accelerating and decelerating.  We use this in areas we are worried the robot will get stuck.
* gyroTurn().  Use this to turn a precise number of degrees to the left or right.  It is designed to smoothly accelerated for the first third of the turn reaching max turning speed, then decelerate smoothly for the last third of the turn.
* gyroTurnTime().  Instead of turning by degrees, it turns by time.  We use this in turns we are worried the robot will get stuck.  
* MoveArm().  Rotates left or right motor.  Up or down.  We move our arm by time, to lessen the chance that it will get stuck.
* initializeRobot().  Always run this function first to initialize all the robot motors and variables.

None of the line following functions are functional in this release.  

