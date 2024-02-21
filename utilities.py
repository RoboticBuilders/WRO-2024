from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media import *
from pybricks.iodevices import Ev3devSensor
import ColorSensorCalibration

import math, time

_MM_PER_INCH = 25.4
def convertInchesToMM(distanceInInches):
    """Convert distance in inches to MM

    distanceInInches:
        measurement in inches that you want converted to MM
    """
    return _MM_PER_INCH * distanceInInches

ev3 = EV3Brick()
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)

robot = DriveBase(leftMotor, rightMotor, 55, convertInchesToMM(4.2))

leftMedMotor = Motor(Port.D)
rightMedMotor = Motor(Port.A)

gyro = GyroSensor(Port.S1)
leftColorSensor = ColorSensor(Port.S4)
rightColorSensor = ColorSensor(Port.S3)

leftColorSensorDevSensor = Ev3devSensor(Port.S4)
rightColorSensorDevSensor = Ev3devSensor(Port.S3)

def resetRobot():
    gyro.reset_angle(0)

# Will correct Robot position so gyro reading is 
# within specified range
def gyroCorrectRobotPos(gTarget, minTurnDegrees=1):
    gLower = gTarget-2
    gUpper = gTarget+2
    # Validate that lower range is ACTUALLY lower than upper
    if gUpper < gLower:
        raise ValueError("Illegal Arguments: gLower (" + str(gLower) + ") is not lower than gUpper (" + str(gUpper) + ")")
    origSpeed, origAccel, origTurnRate, origTurnAccel = robot.settings()
    target = (gLower + gUpper) / 2
    g = gyro.angle()
    while g < gLower or g > gUpper:
        if abs(target - g) <= 20:
            robot.stop()
            robot.settings(straight_speed=10, 
                            straight_acceleration=10,
                            turn_rate=30, turn_acceleration=300)
            if g > gUpper:
                robot.turn(-1*minTurnDegrees)
            else:
                robot.turn(minTurnDegrees)
        else:
            robot.turn(target - g)
        # time.sleep(0.1)
        g = gyro.angle()
        print("Utilities.gyroCorrectRobotPos: Current Gyro (" + str(g) + ") ---- Target Angle (" + str(target) + ")" )

    robot.stop()    
    #reset back to original settings
    robot.settings(origSpeed, origAccel, origTurnRate, origTurnAccel)

# Stops when intensityCheck function returns true. 
# If initialDistanceBeforeSensing is specified, robot will first move that distance before it starts sensing
def moveRobotUntilTrue(intensityChecker, sensor, intensity, sensorName="",
    initialDistanceBeforeSensing=0, stepSize = 0.5, forward=True, speed=100):
    """Moves robot straight forward as long as some condition is true
    
    intensityChecker :
        function that should return true/false. If true, robot moves forward. If false, robot stops and function returns. Function should take a sensor, intensity value and a sensor name (for logging)
    sensor : 
        sensor reference to pass to the checker function above
    intensity : 
        reflection intensity threshold that the checker function is given
    sensorName : 
        name of the sensor being used
    initialDistanceBeforeSensing : 
        distance in inches robot should travel before checking for sensor value. This is to move faster
    stepSize : 
        distance in inches that robot travels before checking sensor condition again
    forward :
        True or False. If True the robot will move forward.
    """
    multiplier = 1.0
    if (forward == False):
        multiplier = -1.0
    robot.stop()
    # Capture the settings before this method is called.
    prevSpeed, prevStraightAcc, prevTurnSpeed, prevTurnAcc = robot.settings()

    robot.settings(straight_speed=speed, straight_acceleration=1000, 
        turn_rate=300, turn_acceleration=500)
    robot.straight(initialDistanceBeforeSensing * _MM_PER_INCH * multiplier)
    robot.drive(100 * multiplier, 0)
    while True:
        if (intensityChecker(sensor, intensity, sensorName) == False):
            robot.stop()
            # Reset the settings to the old values.
            robot.settings(prevSpeed, prevStraightAcc, prevTurnSpeed, prevTurnAcc)
            break

def trueIfIntensityUnder(sensor, intensity, sensorName=""):
    _sensorValue = sensor.reflection()
    #print(sensorName + ": (target < " + str(intensity) + ") ***** Current value: " + str(_sensorValue))
    return _sensorValue < intensity

def trueIfIntensityOver(sensor, intensity, sensorName=""):
    _sensorValue = sensor.reflection()
    print(sensorName + ": (target > " + str(intensity) + ") ***** Current value: " + str(_sensorValue))
    return _sensorValue > intensity

def lineFollow():
    BLACK = 4 #9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.
    PROPORTIONAL_GAIN = 1.2
    print("Waiting for debugger attach")
    print(robot.distance())

    # Start following the line endlessly.
    while robot.distance() <= 200:
    #for i in range(5):
        # Calculate the deviation from the threshold.
        colorReflection = leftColorSensor.reflection()
        if leftColorSensor.reflection() > 25:
            colorReflection = colorReflection * 3

        deviation = colorReflection - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        print(turn_rate)

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)
    print(robot.distance())

def moveArm(distanceInDegrees, speed, arm):
    if distanceInDegrees > 2200 or distanceInDegrees < -2200:
        print("distanceInDegrees should be between -2200 and 2200")
        return
    else:
        if arm == "left":
            leftMedMotor.run_angle(speed, distanceInDegrees)
        if arm == "right":
            rightMedMotor.run_angle(speed, distanceInDegrees)
        if arm != "right" and arm != "left":    
            print("arm should be either left or right(r and l lowercase)")

def turnToAngle(targetAngle, speed=200, forceTurn="None", slowTurnRatio=0.2):
    """Turns the robot the specified angle.
    It calculates if the right or the left turn is the closest
    way to get to the target angle. Can handle both negative 
    targetAngle and negative gyro readings.

    targetAngle -- the final gyro angle to turn the robot to
    speed -- the speed to turn.
    forceTurn -- Can be "None", "Right" or "Left" strings, forcing
    the robot to turn left or right independent of the shortest 
    path.
    slowTurnRatio -- A number between 0.1 and 1.0. Controls the 
    amount of slow turn. If set to 1.0 the entire turn is a slow turn
    the default value is 0.2, or 20% of the turn is slow.
    """
    robot.stop()
    currentAngle = gyro.angle()
    print("TurnToAngle currentAngle: " + str(currentAngle) 
        + " targetAngle: " + str(targetAngle))

    # First make the current angle between 0-360.
    if (currentAngle < 0):
        currentAngle = currentAngle + 360

    # Next make the targetAngle between 0-360
    if (targetAngle < 0):
        targetAngle = targetAngle + 360

    # Compute whether the left or the right
    # turn is smaller.
    degreesToTurnRight = 0
    degreesToTurnLeft = 0
    if (targetAngle > currentAngle):
        degreesToTurnRight = targetAngle - currentAngle
        degreesToTurnLeft = (360-targetAngle) + currentAngle

    else:
        degreesToTurnLeft = currentAngle - targetAngle
        degreesToTurnRight = (360-currentAngle) + targetAngle


    degreesToTurn = 0
    if (forceTurn == "None"):
        if (degreesToTurnLeft < degreesToTurnRight):
            degreesToTurn = degreesToTurnLeft * -1
        else:
            degreesToTurn = degreesToTurnRight
    elif (forceTurn == "Right"):
        degreesToTurn = degreesToTurnRight
    elif (forceTurn == "Left"):
        degreesToTurn = degreesToTurnLeft * -1

    print("degreesToTurn: " + str(degreesToTurn))
    #_turnRobot1degreeAtATime(degreesToTurn, speed, slowTurnRatio)
    _turnRobot(degreesToTurn, speed, slowTurnRatio)

# This is an internal function, do not call this directly.
def _turnRobot(angleInDegrees, speed, slowTurnRatio):
    """Turns the Robot accurately using robot.drive.
    This method first turns the robot with
    speed using robot.turn for a % of the angleInDegrees
    determined by slowTurnRatio. For the remainder of the turn
    i.e. slowturnRatio * angleInDegrees it turns using robot.drive
    with a speed of 30.

    angleInDegrees -- Angle in degrees to turn. Can be +ve or -ve.
    speed -- Fast turn speed. 
    slowTurnRatio -- This is the % of the turn that we want to slow turn.
                     For example 0.2 means that 20% of the turn we want
                     to slow turn.
    """
    slowTurnDegrees = slowTurnRatio * abs(angleInDegrees)
    robot.stop()
    initialAngle = gyro.angle()
    finalAngle = initialAngle + angleInDegrees
    print("Initial gyro angle: " + str(initialAngle) + " TargetAngle: " + str(finalAngle))
    initialTurn = 0

    if angleInDegrees > 0:
        initialTurn = angleInDegrees - slowTurnDegrees
    else:
        initialTurn = angleInDegrees + slowTurnDegrees

    robot.settings (turn_rate=speed, turn_acceleration=500)
    robot.turn(initialTurn)
    robot.stop()

    # Correct the orientation of the robot.
    initialSlowSpeed = 20
    # angleAfterFastTurn = gyro.angle()
    # if (angleAfterFastTurn > finalAngle):
    #     robot.drive(0, -1*initialSlowSpeed)
    # else:
    #     robot.drive(0, initialSlowSpeed)

    gyroAngle = gyro.angle()    
    print("Gyro angle before slow turn: " + str(gyroAngle))
    while (abs(gyroAngle - finalAngle) > 1):
        #proportionalSpeed = ((finalAngle - gyroAngle) / (slowTurnDegrees)) * initialSlowSpeed           
        #print("Gyro angle in slow turn: " + str(gyroAngle) + 
        #    " abs diff: " + str(abs(gyroAngle - finalAngle)) + " speed; " + str(proportionalSpeed))
        #robot.drive(0, proportionalSpeed)

        if (gyroAngle > finalAngle):
            robot.drive(0,initialSlowSpeed * -1)
        else:
            robot.drive(0,initialSlowSpeed)

        gyroAngle = gyro.angle()

    robot.stop()
    print("Final Gyro angle: " + str(gyro.angle()))

# This is an internal function, do not call this directly.
def _turnRobot1degreeAtATime(angleInDegrees, speed, slowTurnRatio=0.2):
    """Turns the robot accurately.
    This method has been superseeded, by a turnRobot that uses
    robot.drive, please use that.

    Turns the robot using robot.turn for most of the turn
    and then slow turns using the gyro. The slow turn is done using 
    1degree at a time turns.

    angleInDegrees -- the angle to turn
    speed -- the speed to turn.
    """

    #slowTurnDegrees = 20
    #if(abs(angleInDegrees) < 20):
    #    slowTurnDegrees = 5

    slowTurnDegrees = slowTurnRatio * abs(angleInDegrees)

    robot.stop()
    initialAngle = gyro.angle()
    print("Initial gyro angle: " + str(initialAngle))
    finalAngle = initialAngle + angleInDegrees
    initialTurn = 0

    if angleInDegrees > 0:
        initialTurn = angleInDegrees - slowTurnDegrees
    else:
        initialTurn = angleInDegrees + slowTurnDegrees

    robot.settings (turn_rate=speed, turn_acceleration=500)
    robot.turn(initialTurn)
    robot.stop()

    # Correct the orientation of the robot.
    angleAfterFastTurn = gyro.angle()
    turnAngle = 1
    if (angleAfterFastTurn > finalAngle):
        turnAngle = -1
    else:
        turnAngle = 1

    gyroAngle = gyro.angle()    
    print("Gyro angle before slow turn: " + str(gyroAngle))
    while (abs(gyroAngle - finalAngle) > 1):
        robot.turn(turnAngle)
        gyroAngle = gyro.angle()
        #print("Gyro angle in slow turn: " + str(gyroAngle) + 
        #    " abs diff: " + str(abs(gyroAngle - finalAngle)) + 
        #    " turnAngle: " + str(turnAngle))
        turnAngle = 1
        if (gyroAngle > finalAngle):
            turnAngle = -1
        else:
            turnAngle = 1
    robot.stop()
    wait(100)
    print("Final Gyro angle: " + str(gyro.angle()))

def isBatteryGood():
    if (ev3.battery.voltage() < 7600):
        ev3.speaker.say("Please recharge EV3")
        print("Please reset gyro")
        return False

    return True

def isGyroGood():
    """Returns if the Gyro is working correctly.
    Can be used to stop the program when the gyro is not working
    correctly.

    The robot should be stationary when this method is called.
    """
    angle1 = gyro.angle()
    wait(5000)
    angle2= gyro.angle()

    if (abs(angle1 - angle2) > 2):
        ev3.speaker.say("Please reset gyro")
        print("Please reset gyro")
        return False

    return True

def smoothGyroProportionalStraight(speed, distanceInMM, target_angle, gain=1):
    """Drive forward in a straight line,
    uses smooth acceleration and deceleration.
    Use only when the distance and speed are meaningful

    Algorithm:
    1. Start with a smooth acceleration.
    2. Use Gyro correction to drive forward.
    3. Run at speed until 80% of distance.
    4. Drive straight to use the deceleration of the robot.
    6. Stop.

    speed -- speed to drive the robot at (0-1000)
    distanceInMM -- distance to drive forward in mm
    target_angle -- The target angle that we want to use for the straight line
    (0-180)
    """
    # reset the robot distance
    robot.reset()
    robot.stop()
    origSpeed, origAccel, origTurnRate, origTurnAccel = robot.settings()
    robot.settings(speed, 500, 1, 500)
    print("smoothGyroPropotional for distance: " + str(distanceInMM)
        + " currentAngle: " + str(target_angle))

    distance80 = min(convertInchesToMM(2), distanceInMM * 0.80)
    _driveStraight(distance80, speed, target_angle, gain)
    #print(distanceInMM)
    #print(distance80)
    #print(distanceInMM - distance80)

    robot.straight(distanceInMM - distance80)

    robot.stop()
    # Restore settings back to what they were
    robot.settings(origSpeed, origAccel, origTurnRate, origTurnAccel)

# Internal function please do not call this directly.
# use one of the public functions
def _driveStraight(distance, speed, target_angle, gain):
    while robot.distance() <= distance :
        #print("Robot distance: " + str(robot.distance()))
        correction = target_angle - gyro.angle()
        #print("gyro angle: " + str(gyro.angle()) + "  correction: " + str(correction))
        turn_rate = correction * gain
        robot.drive(speed, turn_rate)

def waitForAligningRobot(waitSeconds):
    print("Please align robot.")
    ev3.speaker.say("Please align robot.")
    wait(waitSeconds * 1000)
    print("Running code now.")

# Waits till the specific button is pressed on the
# EV3 device
def waitTillButtonPress(button):
    """ waits till the specified button is pressed

    button -- The button to be pressed, it can be
    a combination of buttons. as expressed in
    https://pybricks.com/ev3-micropython/parameters.html#pybricks.parameters.Button
    
    Returns true when the button is pressed."""
    while(True):
        if (button in ev3.buttons.pressed()):
            return True

def colorSensorProportionalLineFollower(speed, intensity, colorSensor, distance, turnMultiplier):
    robot.reset()
    targetIntensity = intensity
    offset = 0 
    driveDistance = distance
    driveSpeed = speed
    multiplier = turnMultiplier

    if colorSensor == "right":
        while robot.distance() < driveDistance:
            offset = targetIntensity - rightColorSensor.reflection()
            offset = offset * multiplier
            robot.drive(driveSpeed, offset)

    if colorSensor == "left":
        while robot.distance() < driveDistance:
            offset = targetIntensity - leftColorSensor.reflection()
            offset = offset * multiplier
            robot.drive(driveSpeed, offset)

def lineSquare():
    """DO NOT USE. NOT WORKING YET.
    Squares the robot against the white line.

    The Robot is square when
    Right: 35/34; Left: 31
    """
    robot.stop()

    print("Starting drive forward to determine orientation")
    robot.drive(10, 0)
    avgLeftColor = 0
    avgRightColor = 0
    counter = 1
    while(True):
        leftColor = leftColorSensor.reflection()
        avgLeftColor += leftColor
        rightColor = rightColorSensor.reflection()
        avgRightColor += rightColor
        print("leftColor : " + str(leftColor) + " rightColor: " + str(rightColor))
        if (leftColor >= 35) or (rightColor >= 35):
            avgLeftColor /= counter
            avgRightColor /= counter
            print("avgLeftColor : " + str(avgLeftColor) + " avgRightColor: " + str(avgRightColor))
            break

    if (avgRightColor > avgLeftColor):
        # Pointing Left, means we need to turn right.
        print("We are pointing left, need to turn right")
        robot.drive(0, 10)
    elif (avgRightColor < avgLeftColor):
        # Pointing right, means we need to turn left.
        print("We are pointing right, need to turn left")
        robot.drive(0, -10)
    else:
        # We are done
        print("We are pointing straight. We are done.")
        return

    print("Starting the turn loop")
    while(True):
        leftColor = leftColorSensor.reflection() + 3
        rightColor = rightColorSensor.reflection()
        print("leftColor : " + str(leftColor) + " rightColor: " + str(rightColor))
        if (abs(leftColor - rightColor) == 0): 
            break

    robot.stop()
    # Reset the settings to the values before the method call.
    #robot.settings(prevSpeed, prevStraightAcc, prevTurnSpeed, prevTurnAcc)