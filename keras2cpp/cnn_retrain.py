
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPooling2D, Flatten, Dropout
from keras import optimizers

import random
import tensorflow as tf

def readData(filePath):
    # attributes of the dataset
    columnNames = ['x-axis','y-axis','z-axis']
    #read the specified file using pandas function and return the data
    data = pd.read_csv(filePath, header=None, names=columnNames, sep='\t', na_values=';')
    return data
     

dataset_1 = readData("../capture1_60hz_30vol.txt")
dataset_2 = readData("../capture2_40hz_60vol.txt")
dataset_3 = readData("../capture3_80HZ_20vol.txt")

print("the first dataset has a shape of:", dataset_1.shape, "the second dataset has:", dataset_2.shape, "and the last one has:", dataset_3.shape)


# Add a new column for sequential timestamps
dataset_1['timestamp'] = np.arange(1, len(dataset_1) + 1)
dataset_2['timestamp'] = np.arange(1, len(dataset_2) + 1)
dataset_3['timestamp'] = np.arange(1, len(dataset_3) + 1)

# defining the function to plot a single axis data
# setup color, title, limit and add grid.
def plotAxis(axis, x, y, title):
    axis.plot(x, y, color='green', linewidth=1)
    axis.set_title(title)
    axis.xaxis.set_visible(False)
    axis.set_ylim([min(y)-np.std(y), max(y)+np.std(y)])
    axis.set_xlim([min(x), max(x)])
    axis.grid(True)

# defining a function to plot the data for a given vibration pattern
def plotVibPattern(data, hz, vol):
    # make subplots of x, y, z over timestamp
    fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(20,10), sharex=True)
    plotAxis(ax0, data['timestamp'], data['x-axis'], 'X-AXIS')
    plotAxis(ax1, data['timestamp'], data['y-axis'], 'Y-AXIS')
    plotAxis(ax2, data['timestamp'], data['z-axis'], 'Z-AXIS')
    # set the size and title
    plt.subplots_adjust(hspace=0.2)
    title = "Accelerometer data for " + hz + "HZ and " + vol + " volume"
    fig.suptitle(title)
    plt.subplots_adjust(top=0.9)
    plt.show()


#select a subset of 2000 samples for plotting
subset_1 = dataset_1[:200]
#plotVibPattern(subset_1, "60", "30")
subset_2 = dataset_2[:200]
#plotVibPattern(subset_2, "40", "60")
subset_3 = dataset_3[:200]
#plotVibPattern(subset_3, "80", "20")


# Bring all values in the [0, 1] interval by  +2 /4

normalized_data1 = (dataset_1 + 2)/4
subset_1 = normalized_data1[:200]
#plotVibPattern(subset_1, "60", "30")

normalized_data2 = (dataset_2 + 2)/4
subset_2 = normalized_data2[:200]
#plotVibPattern(subset_2, "40", "60")

normalized_data3 = (dataset_3 + 2)/4
subset_3 = normalized_data3[:200]
#plotVibPattern(subset_3, "80", "20")


test_data_1 = normalized_data1[:int(normalized_data1.shape[0] * 10 / 100)]
test_data_2 = normalized_data2[:int(normalized_data2.shape[0] * 10 / 100)]
test_data_3 = normalized_data3[:int(normalized_data3.shape[0] * 10 / 100)]

train_data_1 = normalized_data1[int(normalized_data1.shape[0] * 10 / 100):]
train_data_2 = normalized_data2[int(normalized_data2.shape[0] * 10 / 100):]
train_data_3 = normalized_data3[int(normalized_data3.shape[0] * 10 / 100):]



print(train_data_1.shape[0] + test_data_1.shape[0] == dataset_1.shape[0])
print(train_data_2.shape[0] + test_data_2.shape[0] == dataset_2.shape[0])
print(train_data_3.shape[0] + test_data_3.shape[0] == dataset_3.shape[0])


def windows(data,size):
    start = 0
    while start< data.count():
        yield int(start), int(start + size)
        start+= (size/2)

def split_data(data, window_size = 24):
    segments = np.empty((0,window_size,3))
    for (start, end) in windows(data['timestamp'],window_size):
        x = data['x-axis'][start:end]
        y = data['y-axis'][start:end]
        z = data['z-axis'][start:end]

        if(len(data['timestamp'][start:end])==window_size):
            segments = np.vstack([segments,np.dstack([x,y,z])])
    return segments


segments_1 = split_data(train_data_1)
segments_2 = split_data(train_data_2)
segments_3 = split_data(train_data_3)


print("First class has a shape of ", segments_1.shape, "Second class has ", segments_2.shape, "and third ", segments_3.shape)

merged_normalized_segments = np.concatenate([segments_1, segments_2, segments_3], axis=0)

labels_1 = np.full(segments_1.shape[0], 0)
labels_2 = np.full(segments_2.shape[0], 1)
labels_3 = np.full(segments_3.shape[0], 2)

all_labels = np.concatenate((labels_1, labels_2, labels_3))
one_hot_encoded = np.asarray(pd.get_dummies(all_labels),dtype = np.int8)

one_hot_encoded_1 = one_hot_encoded[:segments_1.shape[0]]
one_hot_encoded_2 = one_hot_encoded[segments_1.shape[0]:(segments_1.shape[0] + segments_2.shape[0])]
one_hot_encoded_3 = one_hot_encoded[(segments_1.shape[0] + segments_2.shape[0]):]

labels = np.concatenate((one_hot_encoded_1, one_hot_encoded_2, one_hot_encoded_3))



#Convolutional Neural network model to train on the vibrational pattern
def TrainingModel():
    model = Sequential()
    # adding the first convolutionial layer with 24 filters and 2 by 1 kernal size, using the rectifier as the activation function
    model.add(Conv2D(8, (4,1),input_shape=(24, 3,1),activation='relu'))
    model.add(MaxPooling2D(pool_size=(2,1),strides=(2,1),padding='valid'))
    model.add(Flatten())
    # adding the dropout layer to avoid overfitting
    model.add(Dropout(0.3))
    # adding softmax layer for the classification
    model.add(Dense(3, activation='softmax'))
    # Compiling the model to generate a model
    adam = optimizers.Adam()
    model.compile(loss='categorical_crossentropy', optimizer=adam, metrics=['accuracy'])
    model.summary()
    return model

# shuffle the data a bit: merged_normalized_segments and labels need shuffling in unison

# Shuffle two lists with same order
# Using zip() + * operator + shuffle()
temp = list(zip(merged_normalized_segments, labels))
random.shuffle(temp)
res1, res2 = zip(*temp)
# res1 and res2 come out as tuples, and so must be converted to lists.
res1, res2 = np.array(res1), np.array(res2)

res1 = np.expand_dims(res1, axis=-1)

model = TrainingModel()
history = model.fit(res1, res2, validation_split=0.2,epochs=10,batch_size=5, shuffle=True)


plt.plot(history.history['loss'], color='orange')
plt.plot(history.history['val_loss'], 'b')


# Create segments from the test data
def windows(data,size):
    start = 0
    while start< data.count():
        yield int(start), int(start + size)
        start+= size

def split_data(data, window_size = 24):
    segments = np.empty((0,window_size,3))
    #labels= np.empty((0))
    for (start, end) in windows(data['timestamp'],window_size):
        x = data['x-axis'][start:end]
        y = data['y-axis'][start:end]
        z = data['z-axis'][start:end]

        if(len(data['timestamp'][start:end])==window_size):
            segments = np.vstack([segments,np.dstack([x,y,z])])
            #labels = np.append(labels,l)
    return segments#, labels

test_segments_1 = split_data(test_data_1)
test_segments_2 = split_data(test_data_2)
test_segments_3 = split_data(test_data_3)
test_normalized_segments = np.concatenate((test_segments_1, test_segments_2, test_segments_3))

test_normalized_segments = np.expand_dims(test_normalized_segments, axis=-1)


test_labels_1 = np.full(test_segments_1.shape[0], 0)
test_labels_2 = np.full(test_segments_2.shape[0], 1)
test_labels_3 = np.full(test_segments_3.shape[0], 2)

test_all_labels = np.concatenate((test_labels_1, test_labels_2, test_labels_3))
test_one_hot_encoded = np.asarray(pd.get_dummies(test_all_labels),dtype = np.int8)

test_one_hot_encoded_1 = test_one_hot_encoded[:test_segments_1.shape[0]]
test_one_hot_encoded_2 = test_one_hot_encoded[test_segments_1.shape[0]:(test_segments_1.shape[0] + test_segments_2.shape[0])]
test_one_hot_encoded_3 = test_one_hot_encoded[(test_segments_1.shape[0] + test_segments_2.shape[0]):]

test_labels = np.concatenate((test_one_hot_encoded_1, test_one_hot_encoded_2, test_one_hot_encoded_3))



loss_100, accuracy_100 = model.evaluate(test_normalized_segments, test_labels)


predicted_labels = []
for one_seg in test_normalized_segments:
  predicted_label = model.predict(np.expand_dims(one_seg, axis = 0))
  predicted_labels.append(np.argmax(predicted_label))


from sklearn import metrics

confusion_matrix = metrics.confusion_matrix(test_all_labels, predicted_labels)

cm_display = metrics.ConfusionMatrixDisplay(confusion_matrix = confusion_matrix)
cm_display.plot()
plt.show()

print(metrics.accuracy_score(test_all_labels, predicted_labels))

#save model
from keras2cpp import export_model
export_model(model, 'cnn_model.model')