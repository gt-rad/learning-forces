import os, sys, time, inspect, subprocess, gc
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))
sys.path.insert(0, parentdir)
import numpy as np
from datetime import datetime

import util
if sys.version_info >= (3, 0):
    import build.pysim3 as pysim
else:
    import build.pysim as pysim

'''
Examples of using this file:

Launch 8 shorts simulations which each perform 100 simulations:
    python autoshorts.py 100

Launch one shorts simulation which performs 10 simulations:
    python autoshorts.py -run 10
'''

class AutoShorts:
    def __init__(self, iterations=1):
        self.iterations = iterations

        # Simulation specific
        self.simulator = pysim.shortsSimulator()

        # Initialize simulator
        self.simulator.initialize(maxSteps=1199, enableWind=True)

        # Initialize all simulation data variables
        self.initialize()

    def initialize(self):
        '''
        This should be called after the simulator is initialized or reset.
        '''
        self.initData()
        self.varySimulation()
        self.recordArmSpheresAndWind()

    def initData(self):
        # Simulation data
        self.simTime = 0
        self.simSteps = 0
        self.seed = np.random.randint(1e9)
        self.random = np.random.RandomState(self.seed)
        self.recordedTimes = []
        self.gripperForce = []
        self.gripperTorque = []
        self.forcemap = []
        self.gripperPos = []

    def varySimulation(self):
        # Randomize arm location
        # Horizontal (+ is towards gripper), Vertical (+ is upwards towards sky), sideways (+ is towards camera, arm's right side)
        self.armPosition = [self.random.uniform(-0.05, 0.05), self.random.uniform(-0.2, -0.05), self.random.uniform(-0.05, 0.05)]
        # self.rotateFist = [0, self.random.uniform(-1, 1), self.random.uniform(0, -1), np.radians(10)]
        # Randomize forearm rotation
        self.rotateFist = [0, 0, 1, -np.radians(45)]
        self.velocity = self.random.uniform(1.5, 2.0)
        # These simulations may miss the sleeve or get caught
        self.simulator.positionArm(self.armPosition[0], self.armPosition[1], self.armPosition[2], rotateFist=self.rotateFist, rotateArm=[0, 0, 0, 0])
        self.simulator.setVelocityFactor(self.velocity)

        # Small perturbations on arm rotation
        '''spheres = self.simulator.getArmSpheres()
        armSpheres = np.array([[s.x, s.y, s.z, s.w] for s in spheres])
        # Small pertubation of forearm rotation left and right
        self.armRotation = np.radians(5)
        self.initForearmRotation = [self.random.uniform(-1, 1), self.random.uniform(-1, 1)]
        for i in xrange(2):
            self.simulator.rotateSphereAround(i, self.armRotation, [0, self.initForearmRotation[0], self.initForearmRotation[1]], [float(k) for k in armSpheres[2, :3]])
        self.initArmRotation = [self.random.uniform(-1, 1), self.random.uniform(-1, 1)]
        for i in xrange(3):
            self.simulator.rotateSphereAround(i, self.armRotation, [self.initArmRotation[0], self.initArmRotation[1], 0], [float(k) for k in armSpheres[3, :3]])'''

        # Spline trajectories
        self.splines = []
        splineCount = int(round(4 * self.velocity))
        side = 0 # self.random.randint(1, 2)
        self.splines.append([-0.7 * self.velocity / splineCount + self.random.uniform(-0.03, 0.03), -0.025, (-1)**side * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        self.splines.append([-0.7 * self.velocity * 2 / splineCount + self.random.uniform(-0.03, 0.03), -0.05, (-1)**(1+side) * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        for i in xrange(3, splineCount + 1):
            self.splines.append([-0.7 * self.velocity * i / splineCount + self.random.uniform(-0.03, 0.03), -0.1 + self.random.uniform(-0.02, 0.02), (-1)**((i-1)+side) * self.random.uniform(0, 0.05), 0, 1, 0, 0])
        self.simulator.initSpline(self.splines)

    def recordArmSpheresAndWind(self):
        # Record the location for each sphere of the arm
        # Order of spheres: Hand, wrist, elbow, shoulder
        spheres = self.simulator.getArmSpheres()
        self.armSpheres = [[s.x, s.y, s.z, s.w] for s in spheres]
        if not self.armSpheres:
            print 'Error! Simulator was not initialized before initilizing data.'
            sys.exit()
        # Record wind forces that are applied to arm prior to dressing
        wind = self.simulator.random_wind
        self.windForce = [wind.x, wind.y, wind.z]
        print 'windForce:', self.windForce

    def runSimulation(self):
        for i in range(self.iterations):
            self.simulate()
            # Save all simulation data to file
            self.saveData()
            # Reset simulation
            self.reset()
            # Request garbage collection for good measure
            gc.collect()

    def reset(self):
        self.simulator.reset()
        self.initialize()

    def getData(self):
        # Get Simulation time
        t = self.simulator.recorded_time[-1]
        self.recordedTimes.append(t)

        # Get force data from gripper
        gripForce = self.simulator.rig_parts[1].recorded_forces[-1]
        self.gripperForce.append([gripForce.x, gripForce.y, gripForce.z])

        # Get torque data from gripper
        gripTorque = self.simulator.rig_parts[1].recorded_torques[-1]
        self.gripperTorque.append([gripTorque.x, gripTorque.y, gripTorque.z])

        # Gripper position
        gripPos = self.simulator.getGripperPos()
        self.gripperPos.append([gripPos.x, gripPos.y, gripPos.z])

        # Get data of forces applied to the arm
        pxForcemap = self.simulator.getForcemap()
        # Data is currently a list of PxVec4. Turn this into a list of lists
        forcemap = []
        for vec4 in pxForcemap:
            forcemap.append([vec4.x, vec4.y, vec4.z, vec4.w])
        self.forcemap.append(forcemap)

    def saveData(self):
        data = {'simTime': self.simTime, 'simSteps': self.simSteps, 'armPosition': self.armPosition, 'rotateFist': self.rotateFist,
                'velocity': self.velocity, 'armSpheres': self.armSpheres, 'windForce': self.windForce, 'splines': self.splines,
                'recordedTimes': self.recordedTimes, 'gripperPos': self.gripperPos, 'gripperForce': self.gripperForce,
                'gripperTorque': self.gripperTorque, 'forcemap': self.forcemap, 'seed':, self.seed}
        # Directory format: year-month-day, File format: hour-minute-second_microsecond.pkl
        if not util.saveData(datetime.now().strftime('%H-%M-%S_%f'), data, directory=datetime.now().strftime('%Y-%m-%d_leg'), parentDir=util.rawDataDir):
            # Filename already exists. Try again.
            util.saveData(datetime.now().strftime('%H-%M-%S_%f'), data, directory=datetime.now().strftime('%Y-%m-%d_leg'), parentDir=util.rawDataDir)

    def simulate(self):
        tt = time.time()
        # Run the simulator until we reach the end
        while self.simulator.simulated_step >= -1:
            # Perform simulation
            t = time.time()
            self.simulator.simulate(3)

            if self.simulator.recorded_time:
                # Record data once simulation has begun
                self.simTime += time.time() - t
                self.simSteps += 1
                # Grab all data for this simulation step
                self.getData()
        print 'Total time to simulate:', time.time() - tt

if __name__ == '__main__':
    # This is a super hack for running multiple gown simulations. Each one must have its own process.
    # For some reason Python's multiprocessing is not creating unique processes.
    if len(sys.argv) > 1 and sys.argv[1] == '-run':
        autoshorts = AutoShorts(iterations=int(sys.argv[2])) if len(sys.argv) > 2 else AutoShorts(iterations=1)
        autoshorts.runSimulation()
        gc.collect()
        sys.exit()
    else:
        for i in range(36):
            subprocess.call(['nohup python autoshorts.py -run %s &' % sys.argv[1] if len(sys.argv) > 1 else '10'], shell=True)
            time.sleep(0.1)

