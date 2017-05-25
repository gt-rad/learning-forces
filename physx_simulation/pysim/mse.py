import os, sys, time, random
import numpy as np

from keras import objectives, backend

import util

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

def mse(modelDir, epoch, dataDirectories, seqCount=256, batchSize=16):
    # Compute the mean squared error of all the test sequences.
    model = util.loadModel(modelDir, epoch)
    lstmmse = []
    lstmrmse = []
    msebatches = []
    rmsebatches = []
    for X, y in multiBatch(dataDirectories, seqCount=seqCount):
        predictions = model.predict(X, batch_size=batchSize)
        # MSE computations that the LSTM uses for it's loss function
        lstmmse.append(np.mean((y - predictions) ** 2))
        lstmrmse.append(np.sqrt(np.mean((y - predictions) ** 2)))

        # Split data into estimations and activations
        predActivations = predictions[:, :, predictions.shape[-1]/2:]
        predictions = predictions[:, :, :predictions.shape[-1]/2]
        yActivations = y[:, :, y.shape[-1]/2:]
        y = y[:, :, :y.shape[-1]/2]
        # Update data based on activations
        predictions[predActivations < 0] = 0
        y[yActivations < 0] = 0
        msebatches.append(np.mean((y - predictions) ** 2))
        rmsebatches.append(np.sqrt(msebatches[-1]))
        print 'LSTM MSE:', lstmmse[-1], '| LSTM RMSE:', lstmrmse[-1], '| msebatches:', msebatches[-1], '| rmsebatches:', rmsebatches[-1]
        # print np.mean(np.sum(np.mean((y - predictions) ** 2, axis=-1), axis=-1))
        # print np.mean(np.mean((y - predictions) ** 2, axis=-1), axis=-1)
    print 'Mean LSTM MSE', np.mean(lstmmse)
    print 'Mean LSTM RMSE', np.mean(lstmrmse)
    print 'Mean MSE', np.mean(msebatches)
    print 'Mean RMSE', np.mean(rmsebatches)

if __name__ == '__main__':
    mse('2016-09-10_18-25-14_18-10-54_ForceTorqueVelocity_rawPressuremap_neoarm', 9, ['Arm_2016-09-10_22-41-38_ForceTorqueVelocity_rawPressuremap_neoarm_2500mse'], seqCount=100, batchSize=5)

