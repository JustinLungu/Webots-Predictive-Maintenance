# Overview of Emitter-Receiver Communication
In Webots, the Emitter and Receiver classes allow different components (e.g., robots, supervisors) to communicate with each other within the simulation. The Emitter sends data, while the Receiver receives data. This mechanism is particularly useful when you need real-time interaction or data sharing between components.

## Emitter in the code:

**Preparing Data to Send:**
- The attenuated accelerometer values (attenuatedX, attenuatedY, attenuatedZ) are converted into a comma-separated string:
```
ostringstream dataStream;
dataStream << attenuatedX << "," << attenuatedY << "," << attenuatedZ;
string dataString = dataStream.str();
```

**Sending Data:**
The supervisor sends this string to the robot using:
'''
emitter->send(dataString.c_str(), dataString.length() + 1);
'''
- emitter->send transmits the data to any receivers that are tuned to the same communication channel.
- The +1 accounts for the null terminator, ensuring that the entire string is transmitted.


## Receiver in the code:

checks how many packets are in the receiver's queue. If the queue is not empty, the robot proceeds to receive data.
'''
if receiver.getQueueLength() > 0:
'''

**Receiving and Processing Data:**
- The robot retrieves the string data using:
'''
data = receiver.getString().replace('\x00', '').strip()
'''
    - receiver.getString() retrieves the incoming data as a string.
    - .replace('\x00', '') removes any null characters (which can be present in C++ strings).
    - .strip() removes any leading or trailing whitespace.
- The robot splits the received string into individual values:

'''attenuated_values = list(map(float, data.split(',')))'''
    - data.split(',') breaks the string into a list of substrings based on commas.
    - map(float, ...) converts these substrings to floating-point numbers.