import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten

# Create random data with the correct shape for the model
test_x = np.random.rand(10, 24, 3, 1).astype('f')  # Shape (10, 24, 3, 1) for model
test_y = np.random.rand(10).astype('f')

# Define the model
model = Sequential([
    Conv2D(1, (1, 1), input_shape=(24, 3, 1), activation='linear'),
    Flatten(),
    Dense(1, activation='linear')
])

model.compile(loss='mse', optimizer='adam')

# Train the model
model.fit(test_x, test_y, epochs=1, verbose=False)

# Prepare prediction input with the correct shape for the model
data = np.array([[  # 24x3 matrix
    [0.46838379, 0.48919678, 0.74868774],
    [0.46890259, 0.48904419, 0.74890137],
    [0.46881104, 0.48913574, 0.74896240],
    [0.46868896, 0.48947144, 0.74896240],
    [0.46847534, 0.48898315, 0.74911499],
    [0.46856689, 0.48928833, 0.74868774],
    [0.46844482, 0.48916626, 0.74914551],
    [0.46856689, 0.48892212, 0.74865723],
    [0.46859741, 0.48889160, 0.74877930],
    [0.46844482, 0.48876953, 0.74868774],
    [0.46817017, 0.48883057, 0.74899292],
    [0.46862793, 0.48928833, 0.74832153],
    [0.46859741, 0.48919678, 0.74905396],
    [0.46829224, 0.48922729, 0.74893188],
    [0.46856689, 0.48913574, 0.74911499],
    [0.46871948, 0.48907471, 0.74914551],
    [0.46856689, 0.48934937, 0.74923706],
    [0.46841431, 0.48889160, 0.74838257],
    [0.46908569, 0.48895264, 0.74920654],
    [0.46878052, 0.48959351, 0.74926758],
    [0.46817017, 0.48889160, 0.74880981],
    [0.46929932, 0.48919678, 0.74896240],
    [0.46859741, 0.48925781, 0.74908447],
    [0.46911621, 0.48873901, 0.74853516]
]]).astype('f').reshape(1, 24, 3, 1)  # Reshape to (1, 24, 3, 1) for model compatibility

# Make a prediction
prediction = model.predict(data)
print("\nPrediction output:")
print(prediction)

# Save the model
from keras2cpp import export_model
export_model(model, 'example.model')
