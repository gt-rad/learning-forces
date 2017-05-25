import os, sys, time
import numpy as np
from datetime import datetime

from keras.models import Sequential, Model
from keras.layers import LSTM, Dense, TimeDistributed, Dropout

import util

# Deterministic output
np.random.seed(1000)

def multiBatch(directories, seqCount=0):
    '''
    Loads several mini-batches at a time and yields all of the data as a single dataset, X, and single label set, Y.
    When seqCount = 0, all available sequences are returned at once.
    '''
    # Get all mini-batch files within the specified directories
    filenames = util.getFilenames(directories, parentDir=util.batchDir)
    # Loop over all mini-batch files and cumulate all data into a single dataset
    X = []
    Y = []
    for i, filename in enumerate(filenames):
        # Load a mini-batch from file and combine data
        data = util.loadFile(filename)
        X.extend(data['X'].tolist())
        Y.extend(data['Y'].tolist())
        if seqCount > 0 and len(X) / seqCount >= 1:
            # Yield this collection of batches, then empty X, Y for the next collection of batches
            data = None
            yield np.array(X), np.array(Y)
            X = []
            Y = []
    if X:
        # Yield any remaining data
        yield np.array(X), np.array(Y)

def kerasLSTM(dataDirectories, featureCount=6, outputSize=300, timesteps=700, seqCount=250, batchSize=5, epochs=5, savemodel=True, savedDir=None):
    t = time.time()
    print 'Beginning LSTM compilation'
    model = Sequential()
    model.add(LSTM(50, input_shape=(timesteps, featureCount), activation='tanh', return_sequences=True))
    model.add(LSTM(50, activation='tanh', return_sequences=True))
    model.add(LSTM(50, activation='tanh', return_sequences=True))
    model.add(TimeDistributed(Dense(outputSize * 2, activation='linear')))
    model.compile(loss='mse', optimizer='rmsprop', metrics=['accuracy'])
    print 'Completed compilation in %.3f seconds' % (time.time() - t)
    t = time.time()

    modelDir = datetime.now().strftime('%Y-%m-%d_%H-%M-%S_' + '_'.join(dataDirectories[0].split('_')[2:]))

    # Perform batch training with epochs
    for e in xrange(epochs):
        print '\n\n-- Epoch %d --\n\n' % e

        for X, y in multiBatch(dataDirectories, seqCount=seqCount):
            model.fit(X, y, batch_size=batchSize, nb_epoch=1, validation_split=0.0, verbose=2)
            # Flush output
            sys.stdout.flush()

        # Save the model after each epoch
        if savemodel:
            print 'Saving epoch', e
            util.saveLSTMModel(model, modelDir, e)

    averageTime = (time.time() - t) / epochs
    print 'Total time:', time.time() - t, ', Average time per epoch:', averageTime

if __name__ == '__main__':
    # Raw forcemap estimations
    kerasLSTM(['Arm_2016_ForceTorqueVelocity_rawPressuremap_neoarm'], featureCount=9, outputSize=300, seqCount=256, batchSize=16, epochs=10, savemodel=True)
    kerasLSTM(['Leg_2016_ForceTorqueVelocity_rawPressuremap_neoleg'], featureCount=9, outputSize=400, seqCount=256, batchSize=16, epochs=10, savemodel=True)

