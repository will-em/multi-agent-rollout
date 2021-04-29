# Imports
from sklearn.datasets import make_multilabel_classification
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.losses import binary_crossentropy
from tensorflow.keras.optimizers import Adam
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
infile = open("./pre_processed_data", 'rb')
X, y = pickle.load(infile)
infile.close()
X = np.array(X)
y = np.array(y)
# Split into training and testing data
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.33, random_state=random_state)
# Create the model
model = Sequential()
model.add(Dense(128, activation='relu', input_dim=n_features))
model.add(Dense(128, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(n_classes, activation='sigmoid'))

# Compile the model
model.compile(loss=binary_crossentropy,
              optimizer=Adam(),
              metrics=['accuracy'])

# Fit data to model
model.fit(X_train, y_train,
          batch_size=batch_size,
          epochs=n_epochs,
          verbose=verbosity,
          validation_split=validation_split)

# Generate generalization metrics
score = model.evaluate(X_test, y_test, verbose=0)
print(f'Test loss: {score[0]} / Test accuracy: {score[1]}')
print(model.predict(X_test[0:1]))
print(y_test[0:1])

model.save("test_model")
