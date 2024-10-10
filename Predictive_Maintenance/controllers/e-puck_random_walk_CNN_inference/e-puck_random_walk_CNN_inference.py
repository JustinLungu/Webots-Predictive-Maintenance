########### Now: this is the code from the tutorial:
# https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python#program-a-controller
from controller import Robot, DistanceSensor, Motor, Receiver, Emitter
import numpy as np
import tflite_runtime.interpreter as tflite  # Import TFLite runtime for inference
import struct  # For binary data packing

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28


################ CNN MODEL INITIALIZATION ######################

# Load the TFLite model and allocate tensors
model_path = '../../models/cnn_model.tflite'
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Get input and output tensors
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

################################################################

################ ROBOT INITIALIZATION ##########################
# create the Robot instance.
robot = Robot()

# initialize distance sensors
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

# initialize receiver to receive data from the supervisor
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)

# initialize emitter to send classification label back to the supervisor
emitter = robot.getDevice('emitter')

#################################################################


#################### ROBOT LOGIC + INFERENCE ####################

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:

    ############## GET INPUT DATA + DO INFERENCE #################

    # check for data from the supervisor
    if receiver.getQueueLength() > 0:
        # receive string data from supervisor
        data = receiver.getString().replace('\x00', '').strip()
        
        # Split the data into individual readings
        readings = data.split(';')
        
        # Collect all the 24 readings into a single numpy array for inference
        input_data = []
        for reading in readings:
            attenuated_values = list(map(float, reading.split(',')))
            input_data.extend(attenuated_values)  # Flatten into a single list
        
        # Convert to a numpy array and reshape based on model's expected input shape
        input_data = np.array(input_data, dtype=np.float32).reshape(input_details[0]['shape'])
        
        # Set the input tensor
        interpreter.set_tensor(input_details[0]['index'], input_data)
        
        # Run inference
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        
        # Extract classification label (assuming output_data is a single value or list of probabilities)
        classification_label = np.argmax(output_data)
        print(f"Inference result: {classification_label}")
        
        # Send classification label back to the supervisor via emitter
        emitter.send(struct.pack('i', classification_label))  # Sending the label as an integer

        # clear the receiver queue
        receiver.nextPacket()
    
    ####################################################################
    
    ################### RANDOM WALK CONTROLLER #########################

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
    
    ######################################################################
