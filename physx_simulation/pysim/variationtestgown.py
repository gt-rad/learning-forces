import os, sys, time, inspect, subprocess, gc
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))
sys.path.insert(0, parentdir)
import numpy as np
from datetime import datetime

import util, datapreprocess
import build.pysim as pysim

'''
Used to compare how a trained LSTM model handels against various variations.
'''

# Deterministic output
np.random.seed(1000)

# Generate or load 128 randomly positioned locations
sequences = 128
if util.fileExists('randomPositionsSplines_Arm_%d_New' % sequences, '', util.comparisonDir):
    allArmPositions, allSplines = util.loadFile('randomPositionsSplines_Arm_%d_New' % sequences, '', util.comparisonDir)
    print 'Loaded arm positions and spline trajectories'
else:
    allArmPositions = [[np.random.uniform(-0.05, 0.05), np.random.uniform(-0.45, -0.05), np.random.uniform(-0.1, 0.1)] for i in xrange(sequences)]
    # sides = [np.random.randint(1, 2) for j in xrange(sequences)]
    allSplines = [[[np.random.uniform(-0.03, 0.03), np.random.uniform(-0.02, 0.02), (-1)**i * np.random.uniform(0, 0.05), 0, 1, 0, 0] for i in xrange(1, 20)] for j in xrange(sequences)]
    util.saveData('randomPositionsSplines_Arm_%d_New' % sequences, [allArmPositions, allSplines], '', util.comparisonDir)
    print 'Saved arm positions for later reference'
# Use velocities increasing velocities
velocities = [1.0, 1.25, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]
# y axis variation (left and right, negative is right (wrt human))
rotateY = [0.0, -np.radians(1), -np.radians(5), -np.radians(10), -np.radians(15), np.radians(1), np.radians(5), np.radians(10), np.radians(15)]
# z axis variation (up and down, negative is down (wrt human))
rotateZ = [0.0, -np.radians(1), -np.radians(5), -np.radians(10), -np.radians(15), np.radians(1), np.radians(5)]

class VariationTest:
    def __init__(self, armPositions, splines, saveDir, processId, modelDir, epoch):
        # Process specific arm locations and spline trajectory
        self.armPositions = armPositions
        self.splines = splines

        # Directory to save data
        self.saveDir = saveDir

        self.processId = processId

        # Simulation specific
        self.simulator = pysim.gownSimulator()
        # Initialize simulator
        self.simulator.initialize(maxSteps=1199, enableWind=False)

        # Initialize all simulation data variables
        self.initialize()

        # Preprocessing functions
        # self.functions = [datapreprocess.gripperForce, datapreprocess.gripperTorque]
        self.functions = [datapreprocess.gripperForce, datapreprocess.gripperTorque, datapreprocess.gripperVelocity]
        self.labelFunction = datapreprocess.rawPressuremap

        # Load learning algorithm architecture and model weights
        self.model = util.loadModel(modelDir, epoch)

    def initialize(self, armPosition=[0, 0, 0], velocityFactor=1.0, rotateFist=[0, 0, 0, 0], splines=None):
        '''
        This should be called after the simulator is initialized or reset.
        '''
        self.initData()
        self.varySimulation(armPosition, velocityFactor, rotateFist, splines)
        self.recordArmSpheres()

    def initData(self):
        totalSteps = 700
        self.simData = {'recordedTimes': [[0]*totalSteps], 'gripperForce': [[[0, 0, 0]]*totalSteps], 'gripperTorque': [[[0, 0, 0]]*totalSteps], 'gripperPos': [[[0, 0, 0]]*totalSteps], 'forcemap': [[[]]*totalSteps], 'armSpheres': [[]]}
        self.simStep = 0

        # Data from learning predictions
        self.points = None
        self.timeSteps = []
        self.gripperPos = []
        self.gripperForce = []
        self.gripperTorque = []
        self.predForces = []
        self.predActivations = []
        self.yForces = []
        self.yActivations = []

    def varySimulation(self, armPos=[0, 0, 0], velocityFactor=1.0, rotateFist=[0, 0, 0, 0], splines=None):
        self.velocityFactor = velocityFactor
        self.rotateFist = rotateFist
        self.simulator.positionArm(armPos[0], armPos[1], armPos[2], rotateFist=rotateFist, rotateArm=[0, 0, 0, 0])
        self.simulator.setVelocityFactor(velocityFactor)
        if splines is not None:
            splineCount = int(round(6 * velocityFactor))
            for i in xrange(1, len(splines) + 1):
                # [tx, ty, tz, rw, rx, ry, rz]
                # [0.75 max length, 0 height, alternate sides of arm]
                splines[i - 1, 0] += -0.75 * velocityFactor * i / splineCount
            self.simulator.initSpline(splines[:splineCount].tolist())

    def recordArmSpheres(self):
        # Record the location for each sphere of the arm
        # Order of spheres: Hand, wrist, elbow, shoulder
        spheres = self.simulator.getArmSpheres()
        self.simData['armSpheres'][0] = [[s.x, s.y, s.z, s.w] for s in spheres]

    def recordData(self):
        # Get Simulation time
        t = self.simulator.recorded_time[-1]
        self.simData['recordedTimes'][0][self.simStep] = t

        # Get force data from gripper
        gripForce = self.simulator.rig_parts[1].recorded_forces[-1]
        self.simData['gripperForce'][0][self.simStep] = [gripForce.x, gripForce.y, gripForce.z]

        # Get torque data from gripper
        gripTorque = self.simulator.rig_parts[1].recorded_torques[-1]
        self.simData['gripperTorque'][0][self.simStep] = [gripTorque.x, gripTorque.y, gripTorque.z]

        # Gripper position
        gripPos = self.simulator.getGripperPos()
        self.simData['gripperPos'][0][self.simStep] = [gripPos.x, gripPos.y, gripPos.z]

        # Get data of forces applied to the arm
        pxForcemap = self.simulator.getForcemap()
        # Data is currently a list of PxVec4. Turn this into a list of lists
        forcemap = []
        for vec4 in pxForcemap:
            forcemap.append([vec4.x, vec4.y, vec4.z, vec4.w])

        if self.simStep % 10 == 0:
            # Perform learning on new data to estimate pressure distribution map every 10 time steps
            # Preprocess data
            X = np.concatenate([fun(self.simData) for fun in self.functions], axis=2)
            self.points, y = self.labelFunction([[forcemap]], np.array(self.simData['armSpheres']), retPoints=True, retActivations=True, rotateFist=self.rotateFist, precomputedPoints=self.points)

            # Predict data
            predictions = self.model.predict(X, batch_size=1)

            # Append predicted and true data
            self.timeSteps.append(t)
            self.gripperPos.append(self.simData['gripperPos'][0][self.simStep])
            self.gripperForce.append(self.simData['gripperForce'][0][self.simStep])
            self.gripperTorque.append(self.simData['gripperTorque'][0][self.simStep])
            self.predForces.append(predictions[0, self.simStep, :predictions.shape[-1]/2])
            self.predActivations.append(predictions[0, self.simStep, predictions.shape[-1]/2:])
            self.yForces.append(y[0, 0, :y.shape[-1]/2])
            self.yActivations.append(y[0, 0, y.shape[-1]/2:])

    def simulate(self):
        t = time.time()
        # Run the simulator until we reach the end
        while self.simulator.simulated_step >= -1:
            # Perform simulation
            self.simulator.simulate(3)

            if self.simulator.recorded_time:
                # Grab all data for this simulation step
                self.recordData()
                # Increment simulation step
                self.simStep += 1
        print 'Total time to simulate:', time.time() - t

    def saveData(self, locIter, name):
        data = {'timeSteps': self.timeSteps, 'gripperPos': self.gripperPos, 'gripperForce': self.gripperForce, 'gripperTorque': self.gripperTorque, 'predForces': self.predForces, 'predActivations': self.predActivations, 'yForces': self.yForces, 'yActivations': self.yActivations, 'points': self.points, 'velocity': self.velocityFactor, 'rotateFist': self.rotateFist}
        util.saveData('loc_%d_%s_' % (self.processId * len(self.armPositions) + locIter, name), data, directory=self.saveDir, parentDir=util.comparisonDir)

    def fullSequence(self, i, armPosition, velocity, rotateFist, splines, name):
        # Reset simulation and set variation
        self.simulator.reset()
        self.initialize(armPosition=armPosition, velocityFactor=velocity, rotateFist=rotateFist, splines=splines)
        # Request garbage collection for good measure
        gc.collect()
        # Perform full simulation
        self.simulate()
        # Save all predicted and true learning data to file
        self.saveData(i, name)
        # Request garbage collection for good measure
        gc.collect()

    def runSimulation(self):
        for i, (armPosition, splines) in enumerate(zip(self.armPositions, self.splines)):
            for velocity in velocities:
                self.fullSequence(i, armPosition, velocity, [0, 0, 0, 0], np.array(splines), 'velocity_%.2f' % velocity)

            for rotation in rotateY:
                self.fullSequence(i, armPosition, 1.25, [0, 1, 0, rotation], np.array(splines), 'rotateY_%.2f' % rotation)

            for rotation in rotateZ:
                self.fullSequence(i, armPosition, 1.25, [0, 0, 1, rotation], np.array(splines), 'rotateZ_%.2f' % rotation)

if __name__ == '__main__':
    # This is a super hack for running multiple gown simulations. Each one must have its own process.
    # For some reason Python's multiprocessing is not creating unique processes.
    if len(sys.argv) > 1 and sys.argv[1] == '-run':
        procId = int(sys.argv[2])
        posCount = len(allArmPositions) / int(sys.argv[4])
        posIndex = procId * posCount
        variationtest = VariationTest(armPositions=allArmPositions[posIndex:posIndex+posCount], splines=allSplines[posIndex:posIndex+posCount], saveDir=sys.argv[3], processId=procId, modelDir='2016-09-10_18-25-14_18-10-54_ForceTorqueVelocity_rawPressuremap_neoarm', epoch=9)
        variationtest.runSimulation()
        gc.collect()
        sys.exit()
    else:
        directory = datetime.now().strftime('%Y-%m-%d_%H-%M-%S_ForceTorqueVelocity_rawPressuremap_neoarm_rotations')
        processCount = 16
        for i in range(processCount):
            subprocess.call(['nohup python variationtest.py -run %d %s %d &' % (i, directory, processCount)], shell=True)
            time.sleep(1)

