import time, colorsys
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from basesim import BaseSim
import util, datapreprocess

'''
Used to test out and visualize new ideas within the gown simulator.
'''

class DressingTest(BaseSim):
    def __init__(self):
        super(DressingTest, self).__init__(maxSteps=1299, enableWind=False, pressurePointCount=300, showForcemap=True)

    def initialize(self):
        '''
        This should be called after the simulator is initialized or reset.
        '''
        self.initData()
        self.varySimulation()
        self.recordArmSpheres()

    def initData(self):
        self.forcemap = []
        self.recordedTimes = []
        self.gripperPos = []
        self.gripperForce = []
        self.gripperTorque = []
        self.colors = [plt.cm.jet(i) for i in np.linspace(0, 0.9, 100)]
        self.startTime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.rotateFist = [0, 0, 0, 0]
        self.isActive = False
        self.forearmRotation = 0
        self.initRotate = time.time() + 10
        self.points = [[0, 0, 0]]
        self.y = None
        # Deterministic output
        self.random = np.random.RandomState(1000)

        # # Arm points figure
        # self.camRotateX = 37.8
        # self.camRotateY = -123.6
        # self.camTranslateX = -0.248698333334
        # self.camTranslateY = -0.138908333333
        # self.camDistance = -4.58

        # Splines figure
        self.camRotateX = 26.8
        self.camRotateY = -122.6
        self.camTranslateX = -0.180330000001
        self.camTranslateY = -0.160138333333
        self.camDistance = -6.72

        # self.camRotateX = 26.8
        # self.camRotateY = -122.6
        # self.camTranslateX = -0.186490000001
        # self.camTranslateY = -0.079498333333
        # self.camDistance = -6.72

        # Video motivation slide
        # self.camRotateX = 26.8
        # self.camRotateY = -122.6
        # self.camTranslateX = 0.108069999999
        # self.camTranslateY = -0.247498333333
        # self.camDistance = -6.72

        # Intro paper figure
        # self.camRotateX = 26.4
        # self.camRotateY = -107.4
        # self.camTranslateX = -0.177610000001
        # self.camTranslateY = -0.202298333333
        # self.camDistance = -5.44

        # Close up measurements
        # self.camRotateX = 26.8
        # self.camRotateY = -122.6
        # self.camTranslateX = -0.216237500001
        # self.camTranslateY = -0.247375
        # self.camDistance = -3.65

    def varySimulation(self):
        # Randomize arm location
        # Horizontal (+ is towards gripper), Vertical (+ is upwards towards sky), sideways (+ is towards camera, arm's right side)
        self.simulator.positionArm(0.0, -0.05, 0.0, rotateFist=self.rotateFist, rotateArm=[0, 0, 0, 0])
        velocityFactor = 1.5
        self.simulator.setVelocityFactor(velocityFactor)
        spheres = self.simulator.getArmSpheres()
        armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])

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
        self.armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])
        self.initArmSpheres = None

    def recordData(self):
        # Get Simulation time
        t = self.simulator.recorded_time[-1]
        self.recordedTimes.append(t)

        # Gripper position
        gripPos = self.simulator.getGripperPos()
        self.gripperPos.append([gripPos.x, gripPos.y, gripPos.z])

        # Get force data from gripper
        gripForce = self.simulator.rig_parts[1].recorded_forces[-1]
        self.gripperForce.append([gripForce.x, gripForce.y, gripForce.z])

        # Get torque data from gripper
        gripTorque = self.simulator.rig_parts[1].recorded_torques[-1]
        self.gripperTorque.append([gripTorque.x, gripTorque.y, gripTorque.z])

        # Get data of forces applied to the arm
        pxForcemap = self.simulator.getForcemap()
        # Data is currently a list of PxVec4. Turn this into a list of lists
        self.forcemap = []
        for vec4 in pxForcemap:
            self.forcemap.append([vec4.x, vec4.y, vec4.z, vec4.w])

        # Get data of all 3D forces applied to the arm
        # self.fullForcemap = self.simulator.getFullForcemap()

    def getPoints(self):
        return np.concatenate([self.points, np.zeros((len(self.points), 1))], axis=1)

    def getTruePoints(self):
        if self.y is not None:
            y = self.y[0, 0][:, np.newaxis]
            # Limit magnitudes to range of [0, 1]
            y = np.min(np.concatenate([y / 2.0, np.ones((len(y), 1))], axis=1), axis=1)
            return np.concatenate([self.points, y[:, np.newaxis]], axis=1)
        else:
            return np.concatenate([self.points, np.zeros((len(self.points), 1))], axis=1)

    def renderObjects(self):
        # if self.forcemap or True:
        #     self.points, forceDistribution = datapreprocess.rawPressuremap([[self.forcemap]], np.array([self.armSpheres]), retPoints=True, retActivations=False, rotateFist=self.rotateFist) # , precomputedPoints=self.points if list(self.points) != [[0, 0, 0]] else None
        #     self.y = forceDistribution
        #     for point in self.points:
        #         self.createSphere(point[0], point[1], point[2], 1, 1, 1, 0.5, 0.005)

        if len(self.gripperPos) > 1 and False:
            pos = self.gripperPos[-1]
            # Velocity
            prevPos = np.array(self.gripperPos[-2])
            prevTimes = np.array(self.recordedTimes[-2])
            velocity = (pos - prevPos) / (self.recordedTimes[-1] - prevTimes)
            # Normalize
            factor = 0.1
            velocity = velocity / np.linalg.norm(velocity) * factor
            force = self.gripperForce[-1] / np.linalg.norm(self.gripperForce[-1]) * factor
            torque = self.gripperTorque[-1] / np.linalg.norm(self.gripperTorque[-1]) * factor
            # self.createLine(pos, pos + velocity, 1.0, 0.59, 0, 0.9, linewidth=4.0) # orange
            self.createLine(pos, pos + velocity, 0.96, 0.27, 0.21, 0.9, linewidth=4.0)
            self.createLine(pos, pos + force, 0.12, 0.59, 0.94, 0.9, linewidth=4.0)
            # self.createLine(pos, pos + torque, 0.47, 0.12, 0.63, 0.75, linewidth=4.0) # purple
            self.createLine(pos, pos + torque, 1.0, 0.59, 0, 0.9, linewidth=4.0)

        if self.forcemap:
            self.points, self.y = datapreprocess.rawPressuremap([[self.forcemap]], np.array([self.armSpheres]), retPoints=True, retActivations=False, rotateFist=self.rotateFist)

        # if self.forcemap:
        #     points, forceDistribution = datapreprocess.rawPressuremap([[self.forcemap]], np.array([self.armSpheres]), retPoints=True, retActivations=False, rotateFist=self.rotateFist)
        #     for point, magnitude in zip(points, forceDistribution[0][0]):
        #         if magnitude <= 0.01:
        #             # self.createSphere(point[0], point[1], point[2], 1, 1, 1, 0.5, 0.005)
        #             continue
        #         # color = colorsys.hsv_to_rgb(min((1-magnitude)*0.65, 0.65), 1.0, 1.0)
        #         cIndex = int(min(magnitude / 0.6, 1)*len(self.colors))
        #         color = self.colors[cIndex if cIndex < len(self.colors) else -1]
        #         alpha = min(0.25 + magnitude/2.0, 0.75)
        #         self.createSphere(point[0], point[1], point[2], color[0], color[1], color[2], alpha, 0.005)

        # movieForcePoints = False
        # if not movieForcePoints:
        #     # Save screenshots for video
        #     if self.simStep >= 699:
        #         exit()
        #     if self.simStep != 0:
        #         self.saveScreenshot('movieArm_' + self.startTime)
        # else:
        #     if 600 <= self.simStep < 799:
        #         self.saveScreenshot('movieForcePoints_' + self.startTime)

if __name__ == "__main__":
    dressingtest = DressingTest()
    dressingtest.start()

