# Imports
from sklearn.datasets import make_multilabel_classification
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.losses import binary_crossentropy
from tensorflow.keras.optimizers import Adam
import tensorflow as tf
import pickle
import numpy as np

# Configuration options
n_features = 212
n_classes = 40
n_labels = 2
n_epochs = 50
random_state = 42
batch_size = 20
verbosity = 1
validation_split = 0.2

# Create dataset
infile = open("./Dataset/data263", 'rb')
mat_targets, y = pickle.load(infile)
'''
targets = np.array([], dtype=int)
for mat, targets in enumerate(mat_targets):
    array = []
    for target in targets:
        array.append(target[0])
        array.append(target[1])
np.Concatenate(mat_targets[0], targets)
'''
infile.close()
X = mat_targets[0]
y = y[0]
X = np.array(X)
y = np.array(y)
# Split into training and testing data
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.33, random_state=random_state)


# Create the model
input_1 = tf.keras.layers.Input(shape=(14, 14, 1))
conv2d_1 = tf.keras.layers.Conv2D(64, kernel_size=3,
                                  activation=tf.keras.activations.relu)(input_1)

# Second conv layer :
conv2d_2 = tf.keras.layers.Conv2D(32, kernel_size=3,
                                  activation=tf.keras.activations.relu)(conv2d_1)

# Flatten layer :
flatten = tf.keras.layers.Flatten()(conv2d_2)

# The other input
input_2 = tf.keras.layers.Input(shape=(16,))
dense_2 = tf.keras.layers.Dense(
    5, activation=tf.keras.activations.relu)(input_2)

# Concatenate
concat = tf.keras.layers.Concatenate()([flatten, dense_2])

n_classes = 40
# output layer
output = tf.keras.layers.Dense(units=n_classes,
                               activation=tf.keras.activations.sigmoid)(concat)

model = tf.keras.Model(inputs=[input_1, input_2], outputs=[output])


# Compile the model
model.compile(loss=binary_crossentropy,
              optimizer=Adam(),
              metrics=['accuracy'])

# Fit data to model
model.fit([X_train[0], X_train[1]], y_train,
          batch_size=batch_size,
          epochs=n_epochs,
          verbose=verbosity)

# Generate generalization metrics
score = model.evaluate([X_test[0], X_test[1]], y_test, verbose=0)
print(f'Test loss: {score[0]} / Test accuracy: {score[1]}')
print(model.predict(X_test[0:1]))
print(y_test[0:1])

model.save("test_model")
