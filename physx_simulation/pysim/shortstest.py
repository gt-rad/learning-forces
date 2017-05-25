import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from basesim import BaseSim
import util, datapreprocess

'''
Used to test out and visualize new ideas within the gown simulator.
'''

class ShortsTest(BaseSim):
    def __init__(self):
        super(ShortsTest, self).__init__(maxSteps=1199, enableWind=False, pressurePointCount=400, showForcemap=True, isArm=False)

    def initialize(self):
        '''
        This should be called after the simulator is initialized or reset.
        '''
        self.initData()
        self.varySimulation()
        self.recordArmSpheres()

    def initData(self):
        self.forcemap = []
        self.gripperPos = []
        self.colors = [plt.cm.jet(i) for i in np.linspace(0, 0.9, 100)]
        self.startTime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.rotateFist = [0, 0, 1, -np.radians(45)]
        self.rotateArm = [1, 0, 0, 0]
        self.points = [[0, 0, 0]]
        self.y = None
        # Deterministic output
        self.random = np.random.RandomState(1000)

        self.camRotateX = 16.2
        self.camRotateY = -140.6
        self.camTranslateX = -0.420705
        self.camTranslateY = -0.0674308333333
        self.camDistance = -7.09

        self.camRotateX = 10.2
        self.camRotateY = -141.8
        self.camTranslateX = -0.507651666667
        self.camTranslateY = -0.0407616666666
        self.camDistance = -8.24

        # For presentation video
        self.camRotateX = 10.2
        self.camRotateY = -141.8
        self.camTranslateX = -0.388858333334
        self.camTranslateY = -0.0428216666666
        self.camDistance = -8.24

    def varySimulation(self):
        # Randomize arm location
        # Horizontal (+ is towards gripper), Vertical (+ is upwards towards sky), sideways (+ is towards camera, arm's right side)
        # NOTE: For leg spectrum
        self.simulator.positionArm(0.0, -0.13, 0.0, rotateFist=self.rotateFist, rotateArm=self.rotateArm)
        velocityFactor = 2.0

        self.simulator.positionArm(0.0, -0.05, 0.0, rotateFist=self.rotateFist, rotateArm=self.rotateArm)
        velocityFactor = 2.0

        self.simulator.setVelocityFactor(velocityFactor)
        spheres = self.simulator.getArmSpheres()
        armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])

        splines = []
        splineCount = int(round(4 * velocityFactor))
        side = 1 # self.random.randint(1, 2)
        splines.append([-0.7 * velocityFactor / splineCount + self.random.uniform(-0.03, 0.03), -0.025, (-1)**side * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        splines.append([-0.7 * velocityFactor * 2 / splineCount + self.random.uniform(-0.03, 0.03), -0.05, (-1)**(1+side) * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        for i in xrange(3, splineCount + 1):
            splines.append([-0.7 * velocityFactor * i / splineCount + self.random.uniform(-0.03, 0.03), -0.1 + self.random.uniform(-0.02, 0.02), (-1)**((i-1)+side) * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        # print 'Splines:', splines
        self.simulator.initSpline(splines)

    def recordArmSpheres(self):
        # Record the location for each sphere of the arm
        # Order of spheres: Hand, wrist, elbow, shoulder
        spheres = self.simulator.getArmSpheres()
        self.armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])
        self.initArmSpheres = None

    def recordData(self):
        # Gripper position
        gripPos = self.simulator.getGripperPos()
        self.gripperPos.append([gripPos.x, gripPos.y, gripPos.z])

        # Get data of forces applied to the arm
        pxForcemap = self.simulator.getForcemap()
        # Data is currently a list of PxVec4. Turn this into a list of lists
        self.forcemap = []
        for vec4 in pxForcemap:
            self.forcemap.append([vec4.x, vec4.y, vec4.z, vec4.w])

        # Get data of all 3D forces applied to the arm
        self.fullForcemap = self.simulator.getFullForcemap()

    def getTruePoints(self):
        if self.y is not None:
            y = self.y[0, 0][:, np.newaxis]
            # Limit magnitudes to range of [0, 1]
            y = np.min(np.concatenate([y / 2.0, np.ones((len(y), 1))], axis=1), axis=1)
            return np.concatenate([self.points, y[:, np.newaxis]], axis=1)
        else:
            return np.concatenate([self.points, np.zeros((len(self.points), 1))], axis=1)

    def renderObjects(self):
        # self.rotateArm[-1] += np.radians(1)
        # self.simulator.positionArm(0.0, -0.13, 0.0, rotateFist=self.rotateFist, rotateArm=self.rotateArm)
        # spheres = self.simulator.getArmSpheres()
        # self.armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])
        if self.forcemap and self.simStep % 4 == 0:
            self.points, self.y = datapreprocess.rawPressuremap([[self.forcemap]], np.array([self.armSpheres]), isArm=False, retPoints=True, retActivations=False, rotateFist=self.rotateFist, rotateArm=self.rotateArm) # , precomputedPoints=self.points if list(self.points) != [[0, 0, 0]] else None)
        # print 'Point count:', len(self.points)
        # for point in self.points:
            # self.createSphere(point[0], point[1], point[2], 1, 1, 1, 0.5, 0.005)

        # Save screenshots for video
        # if self.simStep >= 699:
        #     exit()
        # if self.simStep != 0:
        #     self.saveScreenshot('movieLeg_' + self.startTime)

if __name__ == "__main__":
    shortstest = ShortsTest()
    shortstest.start()

