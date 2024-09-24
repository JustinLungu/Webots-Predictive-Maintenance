"""
File: generate_map_vibration_data.py
Description:
    Writes a CSV file in which the x, y, z acceleration values can be retrieved
    for each given (x,y) coordinates at time t
"""

# note: alternatively, the calculations can (I think) be one in WeBots

import pandas as pd
import numpy as np
from scipy.spatial.distance import cdist

INPUT_DATA_PATH = "capture1_60hz_30vol.txt"
MAP_SIZE = 10
VIBRATION_SOURCE = (5, 5)


def read_raw_data(file_path):
    """
    Reads raw data file.

    :param file_path: path to txt file containing the data
    :return: dataframe containing the values
    """ 
    data = pd.read_csv(file_path, sep='\t', header=None, names=['Vibration_X', 'Vibration_Y', 'Vibration_Z'])
    return data.iloc[:1000,:] # keep only 1000 data entries, so WeBots does not crash when you import the map


def create_populated_map(source_vibration):
    """
    Creates a dataframe containing the acceleration data for each combination of 
    (x, y) coordinates and time t.

    :param source_vibration: dataframe containing x,y,z accelaration values at source point
    :return: vibration map
    """
    coordinates = np.array([(x, y) for x in range(MAP_SIZE) for y in range(MAP_SIZE)])
    distances = cdist([VIBRATION_SOURCE], coordinates).flatten() # distance to source
    attenuation = 1 / (1 + distances) # inverse distances for attenuation 
    time_points = len(source_vibration)
    times = np.arange(time_points)

    repeated_attenuation = np.tile(attenuation, (time_points, 1))
    repeated_attenuation = repeated_attenuation.T
    
    # Apply the attenuation to each vibration axis data over time
    vibration_x = (source_vibration['Vibration_X'].values * repeated_attenuation).T
    vibration_y = (source_vibration['Vibration_Y'].values * repeated_attenuation).T
    vibration_z = (source_vibration['Vibration_Z'].values * repeated_attenuation).T

    # Create a DataFrame that includes time, coordinates, and vibrations
    vibration_map = pd.DataFrame({
        'Time': np.repeat(times, MAP_SIZE * MAP_SIZE),
        'X': np.tile(coordinates[:, 0], time_points),
        'Y': np.tile(coordinates[:, 1], time_points),
        'Vibration_X': vibration_x.flatten(),
        'Vibration_Y': vibration_y.flatten(),
        'Vibration_Z': vibration_z.flatten()
    })

    return vibration_map


def query_vibration(x, y, time, vibration_map):
    """
    Perform a query in the vibration map.

    :param x: the x coordinate for the point in space
    :param y: the y coordinate 
    :param time: the timestamp
    :param vibration_map: the datafram containing the map information
    :return: the x, y, z acceleration values for the given point in space and time
    """
    query = vibration_map[(vibration_map['X'] == x) & (vibration_map['Y'] == y) & (vibration_map['Time'] == time)]
    if not query.empty:
        return query[['Vibration_X', 'Vibration_Y', 'Vibration_Z']].iloc[0].to_dict()
    else:
        return "No data available for these coordinates and time."     
    

def write_map_file(map_data):
    """
    Writes the map data into a CSV file for future use.
    :param map_data: the dataframe containing inforation about the map
    """
    output_file = 'vibration_map.csv'
    map_data.to_csv(output_file, index=False)
    print(f"Vibration map successfully saved to {output_file}")


raw_data = read_raw_data(INPUT_DATA_PATH)
print(raw_data.shape)
data_map = create_populated_map(raw_data)

x_query, y_query, time_query = 5, 5, 11
result = query_vibration(x_query, y_query, time_query, data_map)
print(f"Acceleration at ({x_query}, {y_query}) at time {time_query}: {result}")

write_map_file(data_map)



