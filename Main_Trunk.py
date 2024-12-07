from hub import motion_sensor, port, sound, light_matrix
import motor, motor_pair
import runloop
import time, sys, math
import color_sensor 

def initialize_robot():  # Be sure to run this function first in main to initialize the robot
    # WHEEL MOTOR SETUP
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E) # set motor and sensor ports here.  First is Left motor, second is Right motor.  
    global minRobotSpeedInchesPerSecond;    minRobotSpeedInchesPerSecond = 3.0 # inches per second, this is the lowest reliable speed
    global desiredRobotAccel;               desiredRobotAccel = 24.0
    global wheelRevDistance;                wheelRevDistance = 7.0 # wheel travels this many inches with one revolution.  10.9 for Brainiac and 5.45 for Achoo
    global minRobotSpeedDegreesPerSec;      minRobotSpeedDegreesPerSec = minRobotSpeedInchesPerSecond * 360 / wheelRevDistance
    # set default robot max velocity here
    global defaultRobotMaxSpeed;            defaultRobotMaxSpeed = 30 # default robot maximum speed in in per second
    global FORWARDS;                        FORWARDS = 1
    global BACKWARDS;                       BACKWARDS = 0
    global LEFT;                            LEFT = 1
    global RIGHT;                           RIGHT = 0
    global HIGH;                            HIGH =1
    global MEDIUM;                          MEDIUM = 0
    global LOW;                             LOW = -1
    global stuckWheelCounter;               stuckWheelCounter = 0        
    global oldDistanceTraveled
    # ARM MOTOR SETUP
    global ARM_LEFT;                        ARM_LEFT = 1
    global ARM_RIGHT;                       ARM_RIGHT = -1
    global ARM_UP;                          ARM_UP = 1
    global ARM_DOWN;                        ARM_DOWN = -1
    global LEFT_MOTOR_PORT;                 LEFT_MOTOR_PORT = port.F
    global RIGHT_MOTOR_PORT;                RIGHT_MOTOR_PORT = port.B   

    # GYRO SETUP
    motion_sensor.set_yaw_face(motion_sensor.FRONT)    # set motion sensor up face as the face with the button and light matrix
    global gyroOffset;              gyroOffset = 0.0
    global gyroYawAngle;            gyroYawAngle = 0
    global gyroProportionalGain;    gyroProportionalGain = 0.0 # default value
    global gyroIntegralGain;        gyroIntegralGain=  0.0
    global gyroDerivativeGain;      gyroDerivativeGain = 0.0

    # LINE FOLLOW SETUP
    # Set light sensor ports
    # set line follow proportional gain
    # set light sensor sensitivity values here
    global BLACK;   BLACK = 50
    global WHITE;   WHITE = 90

    return

def resetGyro():
    waitCount = 0
    while not motion_sensor.stable():
        time.sleep_ms(200)
        waitCount = waitCount + 1
        if waitCount > 5:
            waitCount = 0
            light_matrix.write("G!")
            print("motion sensor is not stable")
            #time.sleep_ms(6000)
            #sys.exit("Robot not stable.  Unable to calibrate gyro!")
    motion_sensor.reset_yaw(0)
    time.sleep_ms(20) # This is needed to allow completion of the gyro sensor reset.  Much sweat was spent in learning this lesson!   
    gyroOffset = motion_sensor.tilt_angles()[0] * 0.1 
    if abs( gyroOffset ) > 0.4:
        print("Gyro reset unsuccessful!  Yaw angle is not zero and equals ", gyroOffset," degrees. ")
    print("getGyroOffset running!  gyroOffset = ", gyroOffset, "degrees.")    
    return

def getYawAngle():
    gyroYawAngle = motion_sensor.tilt_angles()[0] * 0.1 - gyroOffset # gyroOffset is in degrees.  Need to convert tilt angles to degrees.
    return gyroYawAngle   # gyroYawAngle is in degrees.  

def getGyroProportionalGain(robotSpeedInchesPerSec):
    gyroProportionalGain = .4
    return gyroProportionalGain

def getGyroIntegralGain(robotSpeedInchesPerSec):
    #gyroIntegralGain = getGyroProportionalGain(robotSpeedInchesPerSec) * 0.001
    return 0.0001

def getGyroDerivativeGain(robotSpeedInchesPerSec):
    gyroDerivativeGain = getGyroProportionalGain(robotSpeedInchesPerSec) * 1.0
    return 0.0

def checkSteering(steering):
    if steering > 3.0: # check to ensure steering value isn't too big
        sound.beep(1000, 100, 75)
        #print("Steering gain too high, 100")
        steering = 3.0
    elif steering < -3.0: # or too negative
        sound.beep(1000, 100, 75)
        #print("Steering gain too high, -100")
        steering = -3.0
    return steering

def getSpeed(accelSpeed, maxSpeed, decelSpeed):
    desiredSpeed = 0
    # Always take the minimum speed of accel, max, and decel
    # First, determine if the robot should be decelerating (line three)
    if decelSpeed <= accelSpeed:
        if decelSpeed <= maxSpeed:
            if decelSpeed < minRobotSpeedInchesPerSecond:
                decelSpeed = minRobotSpeedInchesPerSecond
            desiredSpeed = decelSpeed
        else:
            desiredSpeed = maxSpeed
    #Next, check to see if the robot should be at max speed (line two)
    elif maxSpeed <= accelSpeed:
        if maxSpeed <= decelSpeed:
            desiredSpeed = maxSpeed
        else: print("Lost in 2 logic error!")
    #Otherwise, the robot should be accelerating
    elif accelSpeed <= maxSpeed:
        # if lineOneSpeed <= lineThreeSpeed:
        if accelSpeed < minRobotSpeedInchesPerSecond:
            accelSpeed = minRobotSpeedInchesPerSecond
        desiredSpeed = accelSpeed
    else:
        #We have an error somewhere.  Stop the robot!  
        runloop.sleep_ms(2000)
        sound.beep(1400, 1000, 80)# Problem!  This shouldn't happen!  Ding the warning bell!
        print("No man's land!!")
        print("lineonespeed = ", accelSpeed, ".linetwospeed= ", maxSpeed, "lineThreespeed=", decelSpeed)
        desiredSpeed = 0
        motor_pair.move(motor_pair.PAIR_1, 0, velocity = 0) # stop the robot and reflect on what you have done!
        runloop.sleep_ms(2000)
        return
    return desiredSpeed

def convertInchesPerSecToPWM(inputSpeed):
    pwm = 0
    if inputSpeed >= 0:
        pwm = max(0, math.floor( 280 * (inputSpeed + 2.55) ))
    elif inputSpeed < 0:
        pwm = min(0, math.floor( 280 * (inputSpeed - 2.55) ))
    else:
        print("Warning!  Error!  ")
    if pwm < -10000:    pwm = -10000
    if pwm >  10000:    pwm =  10000
    #note there is "deadspace" between pwm -1300 and 1300.  Need to fix that here in the future.  
    #if (pwm > 0 and pwm < 1300):  pwm = 1300
    #if (pwm < 0 and pwm > -1300): pwm = -1300
    return pwm

def gyroStraight(forwardOrBackward, desiredDistanceInches, maxSpeedInchesPerSecond, accelHighLow):# Drive straight for a specified distance, at a specified maximum speed in inches per second
    # check and fix inputs
    if desiredDistanceInches < 0.:    desiredDistanceInches = 0.
    if desiredDistanceInches > 96.:    desiredDistanceInches = 96.

    straightAccel = desiredRobotAccel
    if accelHighLow == HIGH: 
        straightAccel = 2 * desiredRobotAccel 
        if maxSpeedInchesPerSecond > 3 * defaultRobotMaxSpeed:    maxSpeedInchesPerSecond = 3 * defaultRobotMaxSpeed
        if maxSpeedInchesPerSecond < minRobotSpeedInchesPerSecond:    maxSpeedInchesPerSecond = minRobotSpeedInchesPerSecond        
    else:
        if maxSpeedInchesPerSecond > defaultRobotMaxSpeed:    maxSpeedInchesPerSecond = defaultRobotMaxSpeed
        if maxSpeedInchesPerSecond < minRobotSpeedInchesPerSecond:    maxSpeedInchesPerSecond = minRobotSpeedInchesPerSecond

    #initialize gyro
    resetGyro()
    currentYawAngle = getYawAngle()  # Note angles are in decidegrees which means tenths of degrees!Yes, you read that correctly...
    targetYawAngle = currentYawAngle # TargetYawAngle is essentially zero, but just in case we want to change it in the future...
    yawError = 0
    sumOfYawErrors = 0

    #initialize distance and speed
    distanceTraveledInches = 0.
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    distanceRemainingInches = desiredDistanceInches # initially the distance remaining equals the desired distance
    currentSpeed = 0
    oldDistanceTraveledInches = 0
    stuckWheelCounter = 0

    #initialize time
    oldTime = time.ticks_ms() * 0.001# need to check this source of time
    currentTime = oldTime 
    loopTime = 0.
    timer = 0
    lineOneSpeed = 0
    lineTwoSpeed = maxSpeedInchesPerSecond
    lineThreeSpeed = maxSpeedInchesPerSecond
    i = 0

    while distanceTraveledInches < desiredDistanceInches:
        # determine remaining distance
        distanceRemainingInches = desiredDistanceInches - distanceTraveledInches

        # determine time increment since the last while loop
        oldTime = currentTime
        currentTime = time.ticks_ms() * 0.001# convert to sec from ms
        loopTime = ( currentTime - oldTime )
        timer = timer + loopTime

        #determine desired robot speed
        lineOneSpeed = straightAccel * timer
        currentSpeed = 0
        lineThreeSpeed = math.sqrt( 2 * straightAccel * ( distanceRemainingInches ))
        currentSpeed = getSpeed(lineOneSpeed, lineTwoSpeed, lineThreeSpeed)

        # determine steering
        currentYawAngle = motion_sensor.tilt_angles()[0] * 0.1 # get current gyro angle
        oldYawError = yawError
        yawError = targetYawAngle - currentYawAngle
        sumOfYawErrors = sumOfYawErrors + yawError
        changeOfError = yawError - oldYawError
        currentSteering = (yawError) * getGyroProportionalGain(currentSpeed) + sumOfYawErrors * getGyroIntegralGain(currentSpeed) + changeOfError * getGyroDerivativeGain(currentSpeed)
        #currentSteering = checkSteering(currentSteering)# Check that steering is within reasonable bounds

        # MOVE ROBOT!MOVE ROBOT!FINALLY!
        if forwardOrBackward == FORWARDS:
            motor.set_duty_cycle(port.A, convertInchesPerSecToPWM(  0.95*currentSpeed + currentSteering ) ) # 0.95 came from testing, helps the robot stay straight
            motor.set_duty_cycle(port.E, convertInchesPerSecToPWM( -1.06*currentSpeed + currentSteering ) ) # 1.06 came from testing, helps the robot stay straight
        elif forwardOrBackward == BACKWARDS:
            motor.set_duty_cycle(port.A, convertInchesPerSecToPWM( -0.95 * currentSpeed + currentSteering ))
            motor.set_duty_cycle(port.E, convertInchesPerSecToPWM( +1.06 * currentSpeed + currentSteering ))
        # determine new distance traveled by gatheringmotor relative positions
        oldDistanceTraveledInches = distanceTraveledInches
        distanceTraveledInches = abs( ( motor.relative_position(port.A) - motor.relative_position(port.E) )/2. ) * wheelRevDistance / 360.
        if oldDistanceTraveledInches == distanceTraveledInches: 
            stuckWheelCounter += 1
        if stuckWheelCounter >= 100000:
           sound.beep(1200, 200)
           print("Wheel stuck!")
           motor_pair.stop(motor_pair.PAIR_1, stop = motor.BRAKE)
           stuckWheelCounter = 0
           return # exit due to stuck wheel

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.BRAKE) # we should have reached our destination, so let's stop and apply the brakes
    print("End Distance Traveled = ", distanceTraveledInches, ".currentYawAngle = ", currentYawAngle, " degrees.")
    return

def gyroStraightTime(forwardOrBackward, desiredTime, maxSpeedInchesPerSecond, accelHighLow):# Drive straight for a specified distance, at a specified maximum speed in inches per second
    # check and fix inputs
    desiredDistanceInches = 96.  # Need a big number so distance isn't the problem, and we can focus on time

    straightAccel = desiredRobotAccel
    if accelHighLow == HIGH:
        straightAccel = 2 * desiredRobotAccel
        if maxSpeedInchesPerSecond > 3 * defaultRobotMaxSpeed:    maxSpeedInchesPerSecond = 3 * defaultRobotMaxSpeed
        if maxSpeedInchesPerSecond < minRobotSpeedInchesPerSecond:    maxSpeedInchesPerSecond = minRobotSpeedInchesPerSecond
    else:
        if maxSpeedInchesPerSecond > defaultRobotMaxSpeed:    maxSpeedInchesPerSecond = defaultRobotMaxSpeed
        if maxSpeedInchesPerSecond < minRobotSpeedInchesPerSecond:    maxSpeedInchesPerSecond = minRobotSpeedInchesPerSecond

    #initialize gyro
    resetGyro()
    currentYawAngle = getYawAngle()# Note angles are in decidegrees which means tenths of degrees!Yes, you read that correctly...
    targetYawAngle = currentYawAngle # TargetYawAngle is essentially zero, but just in case we want to change it in the future...
    yawError = 0
    sumOfYawErrors = 0

    #initialize distance and speed
    distanceTraveledInches = 0.
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    distanceRemainingInches = desiredDistanceInches # initially the distance remaining equals the desired distance
    currentSpeed = 0
    oldDistanceTraveledInches = 0
    stuckWheelCounter = 0

    #initialize time
    oldTime = time.ticks_ms() * 0.001# need to check this source of time
    currentTime = oldTime
    loopTime = 0.
    timer = 0
    lineOneSpeed = 0
    lineTwoSpeed = maxSpeedInchesPerSecond
    lineThreeSpeed = maxSpeedInchesPerSecond

    while timer < desiredTime:
        # determine remaining distance
        distanceRemainingInches = desiredDistanceInches - distanceTraveledInches

        # determine time increment since the last while loop
        oldTime = currentTime
        currentTime = time.ticks_ms() * 0.001# convert to sec from ms
        loopTime = ( currentTime - oldTime )
        timer = timer + loopTime

        #determine desired robot speed
        lineOneSpeed = straightAccel * timer
        currentSpeed = 0
        lineThreeSpeed = math.sqrt( 2 * straightAccel * ( distanceRemainingInches ))
        currentSpeed = getSpeed(lineOneSpeed, lineTwoSpeed, lineThreeSpeed)

        # determine steering
        currentYawAngle = motion_sensor.tilt_angles()[0] * 0.1 # get current gyro angle
        oldYawError = yawError
        yawError = targetYawAngle - currentYawAngle
        sumOfYawErrors = sumOfYawErrors + yawError
        changeOfError = yawError - oldYawError
        currentSteering = (yawError) * getGyroProportionalGain(currentSpeed) + sumOfYawErrors * getGyroIntegralGain(currentSpeed) + changeOfError * getGyroDerivativeGain(currentSpeed)
        #currentSteering = checkSteering(currentSteering)# Check that steering is within reasonable bounds

        # MOVE ROBOT!MOVE ROBOT!FINALLY!
        if forwardOrBackward == FORWARDS:
            motor.set_duty_cycle(port.A, convertInchesPerSecToPWM(0.95*currentSpeed + currentSteering ) ) # 0.95 came from testing, helps the robot stay straight
            motor.set_duty_cycle(port.E, convertInchesPerSecToPWM( -1.06*currentSpeed + currentSteering ) ) # 1.06 came from testing, helps the robot stay straight
        elif forwardOrBackward == BACKWARDS:
            motor.set_duty_cycle(port.A, convertInchesPerSecToPWM( -0.95 * currentSpeed + currentSteering ))
            motor.set_duty_cycle(port.E, convertInchesPerSecToPWM( +1.06 * currentSpeed + currentSteering ))
        # determine new distance traveled by gatheringmotor relative positions
        oldDistanceTraveledInches = distanceTraveledInches
        distanceTraveledInches = abs( ( motor.relative_position(port.A) - motor.relative_position(port.E) )/2. ) * wheelRevDistance / 360.
        if oldDistanceTraveledInches == distanceTraveledInches:
            stuckWheelCounter += 1
        if stuckWheelCounter >= 100000:
            sound.beep(1200, 200)
            print("Wheel stuck!")
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.BRAKE)
            stuckWheelCounter = 0
            return # exit due to stuck wheel

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.BRAKE) # we should have reached our destination, so let's stop and apply the brakes
    print("End Distance Traveled = ", distanceTraveledInches, ".currentYawAngle = ", currentYawAngle, " degrees.")
    return


def gyroTurn(leftOrRight, desiredAngle, maxSpeedInchesPerSecond): #Turn left a specified number of degrees at a specified max speed
    # check and fix inputs
    if desiredAngle < 0:                desiredAngle = 0
    if desiredAngle > 170:              desiredAngle = 170 # We heard from other teams that the gyro isn't reliable above 180 degree turns so lets stay away from that
    if maxSpeedInchesPerSecond > 20:    maxSpeedInchesPerSecond = 20.
    if maxSpeedInchesPerSecond < 4:     maxSpeedInchesPerSecond = 4.  

    #initialize gyro.  Attempt to reset.  Note: Spike App 3.4 has a bug where this reset is only successful if followed by a wait.
    resetGyro()
    initialYawAngle = getYawAngle()
    currentYawAngle = initialYawAngle
    print("About to start turn.currentYawAngle = ", currentYawAngle, "degrees.  gyroOffset = ", gyroOffset, "degrees.  desiredAngle = ", desiredAngle, "degrees.")
    distanceTraveledInches = 0
    oldDistanceTraveled = 0
    stuckWheelCounter = 0

    while abs( currentYawAngle ) < ( abs ( desiredAngle ) ):
        # MOVE ROBOT!MOVE ROBOT!YEA!
        if leftOrRight == LEFT:
            #motor.set_duty_cycle( port.A, convertInchesPerSecToPWM(maxSpeedInchesPerSecond) )
            #motor.set_duty_cycle( port.E, convertInchesPerSecToPWM(maxSpeedInchesPerSecond) )
            motor.set_duty_cycle( port.A, convertInchesPerSecToPWM(getSmoothControllerSpeed( minRobotSpeedInchesPerSecond, maxSpeedInchesPerSecond, initialYawAngle, currentYawAngle, desiredAngle) ) )
            motor.set_duty_cycle( port.E, convertInchesPerSecToPWM(getSmoothControllerSpeed( minRobotSpeedInchesPerSecond, maxSpeedInchesPerSecond, initialYawAngle, currentYawAngle, desiredAngle) ) )
        elif leftOrRight == RIGHT:
            #motor.set_duty_cycle( port.A, convertInchesPerSecToPWM(-maxSpeedInchesPerSecond) )
            #motor.set_duty_cycle( port.E, convertInchesPerSecToPWM(-maxSpeedInchesPerSecond) )
            motor.set_duty_cycle( port.A, convertInchesPerSecToPWM(-getSmoothControllerSpeed( minRobotSpeedInchesPerSecond, maxSpeedInchesPerSecond, initialYawAngle, currentYawAngle, desiredAngle) ) )
            motor.set_duty_cycle( port.E, convertInchesPerSecToPWM(-getSmoothControllerSpeed( minRobotSpeedInchesPerSecond, maxSpeedInchesPerSecond, initialYawAngle, currentYawAngle, desiredAngle) ) )

        # Get our updated robot yaw angle now that it has moved a bit.
        currentYawAngle = getYawAngle()
        oldDistanceTraveled = distanceTraveledInches
        distanceTraveledInches = abs( ( motor.relative_position(port.A) + motor.relative_position(port.E) )/2. ) * wheelRevDistance / 360.
        if oldDistanceTraveled == distanceTraveledInches:
            stuckWheelCounter += 1
        if stuckWheelCounter >= 500000:
            sound.beep(1200, 200)
            print("Wheel stuck!")
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.BRAKE)
            stuckWheelCounter = 0
            return # exit due to stuck wheel

    #Desired angle should be reached now.  Stop the motors!
    motor.stop( port.A, stop = motor.BRAKE )
    motor.stop( port.E, stop = motor.BRAKE )

    #So what exactly is the resulting yaw angle?How close are we?  Note this approach only works for slow turns.
    print( "Current Yaw Angle = ", currentYawAngle, " degrees." )

def convertInchesToDegrees(inches):
    return math.floor(inches*360./wheelRevDistance)

def getSmoothControllerSpeed( minSpeed, maxSpeedInchesPerSecond, initialYawAngle, currentYawAngle, finalYawAngle ):
    # This assumes the first 30% of the turn is the speed ramp up zone, and the last 30% is the ramp down zone
    initialYawAngle = abs( initialYawAngle )
    currentYawAngle = abs( currentYawAngle )
    finalYawAngle   = abs( finalYawAngle )
    rampUpZone = 0.33 * ( finalYawAngle - initialYawAngle )
    if currentYawAngle < rampUpZone:
        controllerSpeed = (maxSpeedInchesPerSecond - minSpeed) / (rampUpZone)*currentYawAngle + minSpeed
    elif currentYawAngle > (0.67 * ( finalYawAngle - initialYawAngle )):
        controllerSpeed = (minSpeed - maxSpeedInchesPerSecond)/(finalYawAngle - 2*rampUpZone)*(currentYawAngle - 2*rampUpZone) + maxSpeedInchesPerSecond
    else: controllerSpeed = maxSpeedInchesPerSecond
    if controllerSpeed < minSpeed:                  controllerSpeed = minSpeed
    if controllerSpeed > maxSpeedInchesPerSecond:   controllerSpeed = maxSpeedInchesPerSecond
    return math.floor(controllerSpeed)

def moveLeftArmOld(upOrDown = 1, Seconds= 1):
    motor.run_for_time(LEFT_MOTOR_PORT, Seconds * 1000, upOrDown*1200)

def moveRightArmOld(upOrDown = 1, Seconds = 1):
    motor.run_for_time(RIGHT_MOTOR_PORT, Seconds * 1000, upOrDown*1500)

async def moveArm(leftOrRightMotor = port.B, upOrDown = 1, moveArmSeconds= 1., armPower = 400):
    #initialize
    currentTime = 0.  #initialize time
    oldTime = currentTime
    loopTime = 0.
    stuckArmCounter = 0
    stuckArmCountWarningLimit = 8 
    timer = 0.
    if armPower > 1000: armPower = 1000
    motor.reset_relative_position(leftOrRightMotor, 0)
    oldArmMotorAngle = abs( motor.relative_position(leftOrRightMotor) )
    armMotorAngle = oldArmMotorAngle

    currentTime = time.ticks_ms() * 0.001 # convert milliseconds to seconds

    while (timer < moveArmSeconds):
        #Run motor for a bit
        motor.run(leftOrRightMotor, upOrDown * armPower)       
        await runloop.sleep_ms(10)

        #Update time
        oldTime = currentTime
        currentTime = time.ticks_ms() * 0.001   # convert to sec from ms
        loopTime = ( currentTime - oldTime )
        timer = timer + loopTime 

        #Detect stuck motor
        armMotorAngle = abs( motor.relative_position(leftOrRightMotor) )
        #print("Left Arm Relative Angle = ", armMotorAngle)
        if armMotorAngle == oldArmMotorAngle: 
            stuckArmCounter = stuckArmCounter + 1
            print("stuckArmCounter = ", stuckArmCounter)
        else:
            oldArmMotorAngle = armMotorAngle

        if stuckArmCounter > stuckArmCountWarningLimit:
            sound.beep(1200, 200)
            print("Warning!  Stuck arm!!")
            motor.stop(leftOrRightMotor)
            stuckArmCounter = 0
            return #Exit this function prematurely

    motor.stop(leftOrRightMotor, stop = motor.BRAKE)
    return

def moveRightArm(upOrDown = 1, Seconds = 1):
    motor.run_for_time(RIGHT_MOTOR_PORT, Seconds * 1000, upOrDown*1500)

#def moveStraightUntilColor(colorSpeed = 1.0, color = WHITE, DistanceAfterColorDetected = 0):
    #initialize variables

    #move robot!
    #while ( color_sensor.reflection(port.C) <= WHITE ):
        #motor.set_duty_cycle(port.A, convertInchesPerSecToPWM( 0.95 * colorSpeed ) ) # 0.95 came from testing, helps the robot stay straight
        #motor.set_duty_cycle(port.E, convertInchesPerSecToPWM( -1.06 * colorSpeed ) ) # 1.06 came from testing, helps the robot stay straight

    #if (DistanceAfterColorDetected > 0.2):
        #gyroStraight(FORWARDS, DistanceAfterColorDetected, colorSpeed)

    #return

async def main():  # This is our main program! 
    initialize_robot()  # Initialize all the robot number values
    await runloop.sleep_ms(250) # let robot rest for a split second
    
    #await runloop.sleep_ms(100)
    #gyroStraight (FORWARDS, 5,3)
    #await runloop.sleep_ms (2000)
    
    gyroStraightTime(FORWARDS,  1, 10, 0)
    gyroStraightTime(BACKWARDS, 1, 10, 0)
    
    sys.exit("End of program")  
    

runloop.run(main())
