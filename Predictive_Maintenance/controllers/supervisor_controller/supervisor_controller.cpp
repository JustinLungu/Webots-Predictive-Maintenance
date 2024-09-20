// File:          supervisor_controller.cpp
// Date:
// Description:   Supervisor script to read and save accelerometer data
// Author:
// Modifications:

#include <webots/Supervisor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

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

// This is the main program of your controller.
int main(int argc, char **argv) {
  // Create the Supervisor instance.
  Supervisor *supervisor = new Supervisor();
  
  Node *rootNode = supervisor->getRoot();
  Field *childrenField = rootNode->getField("children");
  
  childrenField->importMFNodeFromString(-1, "DEF E-PUCK E-puck { translation 0 0 0, controller \"\" }");
  Node *epuckNode = supervisor->getFromDef("E-PUCK");
  Field *translationField = epuckNode->getField("translation");
  
  
  
  

  // Get the time step of the current world.
  int timeStep = (int)supervisor->getBasicTimeStep();

  // File path to the accelerometer data
  string filePath = "data/capture1_60hz_30vol.txt";  // Adjust path as necessary

  // Read and save the accelerometer data
  vector<vector<double>> accelerometerData = readAccelerometerData(filePath);

  // Check if data was successfully read
  if (!accelerometerData.empty()) {
    cout << "Successfully read accelerometer data with " << accelerometerData.size() << " entries." << endl;

    // Print the data to verify it was read correctly
    for (size_t i = 0; i < accelerometerData.size(); ++i) {
      cout << "Entry " << i + 1 << ": ";
      for (size_t j = 0; j < accelerometerData[i].size(); ++j) {
        cout << accelerometerData[i][j] << " ";
      }
      cout << endl;
    }
  } else {
    cout << "No data read from the file." << endl;
  }


  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  int i = 0;
  while (supervisor->step(timeStep) != -1) {
    // Simulation step logic can be added here.
    if(i < int(accelerometerData.size())){
      cout << "Entry " << i + 1 << ": ";
      for (size_t j = 0; j < accelerometerData[i].size(); ++j) {
        cout << accelerometerData[i][j] << " ";
      }
      cout << endl;
    } else {
      cout << "Out of data";
    }
    i++;

  };

  // Clean up
  delete supervisor;
  return 0;
}
