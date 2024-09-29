// File: supervisor_controller.cpp
// Date:
// Description: Supervisor script to read and save accelerometer data with attenuation calculations based on robot position
// Author:
// Modifications:

/*
Are you recreating the vibration map in this code? Cause the values do no match with the ones from the csv.
The simulation runs for approx 19 minutes before the supervisor runs out of data. Why 19 and not 5 minutes? idk

*/

#include <webots/Supervisor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>  // For sqrt and pow

#include <webots/Emitter.hpp>


// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// Function to read accelerometer data from a file
vector<vector<double>> readAccelerometerData(const string &filename) {
  vector<vector<double>> data;
  ifstream file(filename);
  string line;

  // Check if the file opened successfully
  if (!file.is_open()) {
    cerr << "Error: Could not open the file " << filename << endl;
    return data;
  }

  // Read the file line by line
  while (getline(file, line)) {
    stringstream ss(line);
    string value;
    vector<double> row;

    // Split the line by tabs and convert to doubles
    while (getline(ss, value, '\t')) {
      row.push_back(stod(value));
    }

    // Add the row to the data vector
    data.push_back(row);
  }

  file.close();
  return data;
}

// Function to calculate the distance between two points in 3D space
double calculateDistance(const double *position1, const vector<double> &position2) {
  return sqrt(pow(position1[0] - position2[0], 2) + pow(position1[1] - position2[1], 2));
}

// Function to calculate attenuation based on distance
double calculateAttenuation(double distance) {
  return 1 / (1 + distance); // change to + distance
}

// Main program of your controller
int main(int argc, char **argv) {
  // Create the Supervisor instance
  Supervisor *supervisor = new Supervisor();
  //used for sending data to other robots in the simulation
  Emitter *emitter = supervisor->getEmitter("emitter");

  
  Node *rootNode = supervisor->getRoot();
  Field *childrenField = rootNode->getField("children");
  
  // Import a robot node
  childrenField->importMFNodeFromString(-1, "DEF E-PUCK E-puck { translation 0 0 0, controller \"e-puck_random_walk_CNN_inference\" }");
  Node *epuckNode = supervisor->getFromDef("E-PUCK");
  //Not used yet: uncomment when used
  //Field *translationField = epuckNode->getField("translation");

  // Get the time step of the current world
  int timeStep = (int)supervisor->getBasicTimeStep();

  string filePath = "data/capture1_60hz_30vol.txt"; 

  // Read accelerometer data from file
  vector<vector<double>> accelerometerData = readAccelerometerData(filePath);

  // Check if data was successfully read
  if (!accelerometerData.empty()) {
    cout << "Successfully read accelerometer data with " << accelerometerData.size() << " entries." << endl;
  } else {
    cout << "No data read from the file." << endl;
  }

  vector<double> vibrationSource = {0.0, 0.0, 0.0}; // change to actual starting coordinates

  // Main loop: perform simulation steps until Webots stops the controller
  size_t i = 0;
  while (supervisor->step(timeStep) != -1) {
    // Get robot position
    const double *position = epuckNode->getPosition();
    double coordinates[2];
    coordinates[0] = floor(position[0]);
    coordinates[1] = floor(position[1]);
    
    cout << "Robot position: " << position[0] << " " << position[1] << " " << position[2] << endl;
    cout << "Closest rounded point: " << coordinates[0] << " " << coordinates[1] << endl;

    //based on distance from vibration source we apply a certain attenuation level
    double distance = calculateDistance(coordinates, vibrationSource);
    double attenuation = calculateAttenuation(distance);

    // Print attenuation for debug purposes
    // cout << "Attenuation: " << attenuation << endl;

    // Calculate and print attenuated accelerometer data at current step
    if (i < accelerometerData.size()) {
       double attenuatedX = accelerometerData[i][0] * attenuation;
       double attenuatedY = accelerometerData[i][1] * attenuation;
       double attenuatedZ = accelerometerData[i][2] * attenuation;
       
       
       // Convert the attenuated values to a comma-separated string
       ostringstream dataStream;
       dataStream << attenuatedX << "," << attenuatedY << "," << attenuatedZ;
       string dataString = dataStream.str();
        
       // Send the string data
       emitter->send(dataString.c_str(), dataString.length() + 1); // +1 to include the null terminator
               
               
       
       //vector<double> dataToSend = {attenuatedX, attenuatedY, attenuatedZ};
       //Send data as binary using reinterpret_cast
       //emitter->send(reinterpret_cast<const char*>(dataToSend.data()), dataToSend.size() * sizeof(double));
       //emitter->send(dataToSend.data(), dataToSend.size() * sizeof(double)); // Send data to the robot
    
       cout << "Sent data: Attenuated Acceleration X: " << attenuatedX 
         << " Y: " << attenuatedY << " Z: " << attenuatedZ << endl;
    } else {
      cout << "Out of data" << endl;
    }
    i++;
  };
  
  delete emitter;
  delete supervisor;
  return 0;
}
