########### Now: this is the code from the tutorial https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python#program-a-controller
##### TODO: make it such that 24 readings are transmitted to the robot from the supervisor
###### TODO: try to import our CNN model and run an infrence with it
####### TODO: make this a emitter and the supervisor a received (on a sepparate channel?) and send back the classification label to the supervisor

from controller import Robot, DistanceSensor, Motor, Receiver
import struct  # Import struct for binary data unpacking

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# initialize motors 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# initialize receiver
receiver = robot.getDevice('receiver')
#method sets the time step for the receiver, 
#allowing it to receive data at each simulation step.
receiver.enable(TIME_STEP)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # check for data from the supervisor
    if receiver.getQueueLength() > 0:
        # receive string data from supervisor
        data = receiver.getString().replace('\x00', '').strip()
        
        # Split the comma-separated string into individual values
        attenuated_values = list(map(float, data.split(',')))
        
        # print received attenuated acceleration data
        print(f"Received data: Attenuated Acceleration X: {attenuated_values[0]}, Y: {attenuated_values[1]}, Z: {attenuated_values[2]}")
        # clear the receiver queue
        receiver.nextPacket()

    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)