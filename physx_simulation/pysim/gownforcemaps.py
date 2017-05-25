import sys, copy
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from keras.models import Sequential, Model
from keras.layers import LSTM, GRU, Dense, TimeDistributed, Dropout, Merge, Highway, Masking

from basesim import BaseSim
import util, datapreprocess

'''
Used to visualize results from a learned LSTM model.
'''

# Deterministic output
np.random.seed(1000)

class LearningGown(BaseSim):
    def __init__(self, modelDir, epoch, activations=True, featureDim=9, outputDim=300, hdVideo=False):
        self.activations = activations
        self.hdVideo = hdVideo
        super(LearningGown, self).__init__(maxSteps=1199, enableWind=False, pressurePointCount=outputDim, showForcemap=True, dualArm=True)

        # Preprocessing functions
        self.functions = [datapreprocess.gripperForce, datapreprocess.gripperTorque, datapreprocess.gripperVelocity]
        self.labelFunction = datapreprocess.rawPressuremap

        # Load learning algorithm architecture and model weights
        self.model = util.loadModel(modelDir, epoch)

        # Make our LSTM stateful for fast predictions
        weights = self.model.get_weights()
        self.model = Sequential()
        self.model.add(LSTM(50, batch_input_shape=(1, 1, featureDim), activation='tanh', return_sequences=True, stateful=True))
        self.model.add(LSTM(50, activation='tanh', return_sequences=True, stateful=True))
        self.model.add(LSTM(50, activation='tanh', return_sequences=True, stateful=True))
        self.model.add(TimeDistributed(Dense(outputDim * 2, activation='linear')))
        self.model.compile(loss='mse', optimizer='rmsprop', metrics=['accuracy'])
        self.model.set_weights(weights)

        # Compile and prepare model
        self.y = None
        self.predictions = None
        self.predActivations = None
        self.yActivations = None

    def initialize(self):
        self.initData()
        self.varySimulation()
        self.recordArmSpheres()

    def initData(self):
        totalSteps = 700

        self.data = {'recordedTimes': [[]], 'gripperForce': [[]], 'gripperTorque': [[]], 'gripperPos': [[]], 'forcemap': [[]], 'armSpheres': [[]]}
        self.prevData = None
        self.prevProgress = None
        self.startPos = None
        self.prevTime = 0
        self.prevPos = 0
        # Deterministic output
        self.random = np.random.RandomState(1000)

        self.colors = [plt.cm.jet(i) for i in np.linspace(0, 0.9, 100)]

        self.rotateFist = [0, 0, 0]
        self.startTime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Learning
        self.y = None
        self.predictions = None
        self.predActivations = None
        self.yActivations = None
        self.points = None

    def varySimulation(self):
        # Randomize arm location
        # Horizontal (+ is towards gripper), Vertical (+ is upwards towards sky), sideways (+ is towards camera, arm's right side)
        self.simulator.positionArm(0.0, -0.05, 0.0, rotateFist=self.rotateFist, rotateArm=[0, 0, 0, 0])
        velocityFactor = 1.25

        self.simulator.setVelocityFactor(velocityFactor)
        splines = []
        splineCount = int(round(6 * velocityFactor))
        side = 0 # self.random.randint(1, 2)
        for i in xrange(1, splineCount + 1):
            # [tx, ty, tz, rw, rx, ry, rz]
            # [0.75 max length, 0 height, alternate sides of arm]
            splines.append([-0.75 * velocityFactor * i / splineCount + self.random.uniform(-0.03, 0.03), self.random.uniform(-0.02, 0.02), (-1)**(i+side) * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        print splines
        self.simulator.initSpline(splines)

    def recordArmSpheres(self):
        # Record the location for each sphere of the arm
        # Order of spheres: Hand, wrist, elbow, shoulder
        spheres = self.simulator.getArmSpheres()
        self.data['armSpheres'][0] = [[s.x, s.y, s.z, s.w] for s in spheres]
        # self.simData['armSpheres'][0] = [[s.x, s.y, s.z, s.w] for s in spheres]

    def performLearning(self):
        # Preprocess data
        X = np.concatenate([fun(self.data, prevData=self.prevData) if fun == datapreprocess.gripperVelocity else fun(self.data) for fun in self.functions], axis=2)
        self.points, self.y = self.labelFunction(self.data['forcemap'], np.array(self.data['armSpheres']), knearest=5, retPoints=True, rotateFist=self.rotateFist, precomputedPoints=self.points)

        # Predict data
        self.predictions = self.model.predict(X, batch_size=1)

        if self.activations:
            self.predActivations = self.predictions[:, :, self.predictions.shape[-1]/2:]
            self.predictions = self.predictions[:, :, :self.predictions.shape[-1]/2]
            self.yActivations = self.y[:, :, self.y.shape[-1]/2:]
            self.y = self.y[:, :, :self.y.shape[-1]/2]

    def recordData(self):
        self.prevData = copy.deepcopy(self.data)

        # Get Simulation time
        t = self.simulator.recorded_time[-1]
        self.prevTime = self.data['recordedTimes'][0][0] if self.data['recordedTimes'][0] else 0
        self.data['recordedTimes'][0] = [t]

        # Get force data from gripper
        gripForce = self.simulator.rig_parts[1].recorded_forces[-1]
        self.data['gripperForce'][0] = [[gripForce.x, gripForce.y, gripForce.z]]

        # Get torque data from gripper
        gripTorque = self.simulator.rig_parts[1].recorded_torques[-1]
        self.data['gripperTorque'][0] = [[gripTorque.x, gripTorque.y, gripTorque.z]]

        # Gripper position
        gripPos = self.simulator.getGripperPos()
        self.prevPos = self.data['gripperPos'][0][0] if self.data['gripperPos'][0] else [0, 0, 0]
        self.data['gripperPos'][0] = [[gripPos.x, gripPos.y, gripPos.z]]

        # Get data of forces applied to the arm
        pxForcemap = self.simulator.getForcemap()
        # Data is currently a list of PxVec4. Turn this into a list of lists
        forcemap = []
        for vec4 in pxForcemap:
            forcemap.append([vec4.x, vec4.y, vec4.z, vec4.w])
        self.data['forcemap'][0] = [forcemap]

        # Perform learning on new data to estimate pressure distribution map
        if self.simStep > 1:
            self.performLearning()
        # if self.simStep % 4 == 0:
        #     self.performLearning()
        # elif self.predActivations is not None:
        #     self.predActivations[0, self.simStep] = self.predActivations[0, self.simStep - 1]
        #     self.predictions[0, self.simStep] = self.predictions[0, self.simStep - 1]
        #     self.yActivations[0, self.simStep] = self.yActivations[0, self.simStep - 1]
        #     self.y[0, self.simStep] = self.y[0, self.simStep - 1]

    def getPoints(self):
        if self.y is not None:
            # Set unactivated points to 0
            predicts = self.predictions[0, 0]
            predicts[self.predActivations[0, 0] < 0] = 0
            predicts[predicts < 0] = 0
            predicts = predicts[:, np.newaxis]
            # Limit magnitudes to range of [0, 1]
            predicts = np.min(np.concatenate([predicts / 2.0, np.ones((len(predicts), 1))], axis=1), axis=1)
            return np.concatenate([self.points, predicts[:, np.newaxis]], axis=1)
        else:
            if self.points is None:
                return np.concatenate([[[0, 0, 0]], np.zeros((len([[0, 0, 0]]), 1))], axis=1)
            else:
                return np.concatenate([self.points, np.zeros((len(self.points), 1))], axis=1)

    def getTruePoints(self):
        if self.y is not None:
            y = self.y[0, 0][:, np.newaxis]
            # Limit magnitudes to range of [0, 1]
            y = np.min(np.concatenate([y / 2.0, np.ones((len(y), 1))], axis=1), axis=1)
            return np.concatenate([self.points, y[:, np.newaxis]], axis=1)
        else:
            if self.points is None:
                return np.concatenate([[[0, 0, 0]], np.zeros((len([[0, 0, 0]]), 1))], axis=1)
            else:
                return np.concatenate([self.points, np.zeros((len(self.points), 1))], axis=1)

    def renderDualObjects(self):
        if self.simStep > 1:
            pos = np.array(self.data['gripperPos'][0][0])
            # Velocity
            velocity = (pos - np.array(self.prevPos)) / (self.data['recordedTimes'][0][0] - self.prevTime)
            # Normalize
            factor = 0.1
            velocity = velocity / np.linalg.norm(velocity) * factor
            force = np.array(self.data['gripperForce'][0][0]) / np.linalg.norm(self.data['gripperForce'][0][0]) * factor
            torque = np.array(self.data['gripperTorque'][0][0]) / np.linalg.norm(self.data['gripperTorque'][0][0]) * factor
            self.createLine(pos, pos + velocity, 0.96, 0.27, 0.21, 0.9, linewidth=4.0)
            self.createLine(pos, pos + force, 0.12, 0.59, 0.94, 0.9, linewidth=4.0)
            self.createLine(pos, pos + torque, 1.0, 0.59, 0, 0.9, linewidth=4.0)

        if self.hdVideo:
            # Save screenshots for video
            if self.simStep >= 699:
                exit()
            if self.simStep != 0:
                self.saveScreenshot('movieArm_' + self.startTime)

    def renderObjects(self):
        pass

if __name__ == "__main__":
    learninggown = LearningGown('2016-09-10_18-25-14_18-10-54_ForceTorqueVelocity_rawPressuremap_neoarm', 9, hdVideo=False)
    learninggown.start()

