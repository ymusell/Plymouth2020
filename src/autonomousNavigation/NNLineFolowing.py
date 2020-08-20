from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.optimizers import SGD
import numpy as np 

X = np.array([[0,0],[0,1],[1,0],[1,1]])	#Getting the data from the boat
y = np.array([[0],[1],[1],[0]])

model = Sequential()
model.add(Dense(2, input_dim=2, activation='tanh'))		#Size of the hidden layer to change
model.add(Dense(1, activation = 'sigmoid'))

model.compile(loss='binary_crossentropy', optimizer=SGD(lr=0.1))	#Change the optimizer

model.fit(X, y, batch_size=1, nb_epoch=500)
print(model.predict_proba(X))

for layer in model.layers:
	weights = layer.get_weights()
	print(weights)
print(model.summary())
model.save_weights("data/model.h5")