import os, sys, time, random, inspect, subprocess, gc
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))
sys.path.insert(0, parentdir)
# import util
import cma, mlpy
import numpy as np
from datetime import datetime
if sys.version_info >= (3, 0):
    import pickle
    import build.pysim3 as pysim
else:
    import cPickle as pickle
    import build.pysim as pysim

class CMA_Optimization:
    def __init__(self, iterations=10, population=10, subject=4, visualize=False):

        self.node_start_time = time.time()
        # Initialize the simulator that will be used later
        self.simulator = pysim.gownSimulator()

        # print self.simulator.cloth.bend_stiff
        # self.simulator.cloth.bend_stiff[0] = 123
        # print self.simulator.cloth.bend_stiff
        # self.simulator.cloth.bend_stiff = [1,2,3,4]
        # print self.simulator.cloth.bend_stiff
        self.simulator.initialize(maxSteps=0)

        # print self.simulator.cloth.bend_stiff
        self.simulator.reset()


        # Load the data used for comparison in CMA. This is data from a single subject that is used for comparison in
        # the objective function.
        self.iterations = iterations
        self.population = population

        this_file_path = os.path.dirname(os.path.abspath(__file__))
        split_path = this_file_path.split('/')
        base_data_path = '/'
        for i in xrange(1, len(split_path)):
            base_data_path = base_data_path + split_path[i] + '/'
        base_data_path = base_data_path + 'Data/'

        good_data_path = base_data_path + 'good/subject' + str(subject) + '/'
        miss_data_path = base_data_path + 'miss/subject' + str(subject) + '/'

        good_data_list = os.listdir(good_data_path)
        miss_data_list = os.listdir(miss_data_path)

        self.good_trial_data = []
        self.miss_trial_data = []
        self.hapGood = pysim.HapticObjective()
        self.hapMiss = pysim.HapticObjective()


        for file_name in good_data_list:
            if 'txt' in file_name:
                good_file_path = good_data_path + file_name
                print 'Loading good file from file: ', good_file_path
                self.good_trial_data = np.array([map(float, line.strip().split()) for line in open(good_file_path)])
                self.hapGood.ReadData(good_file_path)
                break
        for file_name in miss_data_list:
            if 'txt' in file_name:
                miss_file_path = miss_data_path + file_name
                print 'Loading miss file from file: ', miss_file_path
                self.miss_trial_data = np.array([map(float, line.strip().split()) for line in open(miss_file_path)])
                self.hapMiss.ReadData(miss_file_path)
                break

        if self.good_trial_data == [] or self.miss_trial_data == []:
            print 'I was unable to find an appropriate file for good or miss. Cannot continue.'

        print 'Starting to do CMA-ES! This could take a while!'

        self.run_cma()

    def run_cma(self):

        popsize = self.population
        maxiter = self.iterations
        all_parameters = ['STRETCH', 'SHEAR', 'BEND',
                          'FRICTION', 'SELFCOLDIST', 'SELFCOLFRICTION',
                          'STIFFPOWER', 'ITERATION', 'WIND',
                          'ARM_HEIGHT_PERT_GOOD', 'ARM_HEIGHT_PERT_MISS', 'ARM_HEIGHT_PERT_CAUG',
                          'ARM_HORIZONTAL_PERT_GOOD', 'ARM_HORIZONTAL_PERT_MISS', 'ARM_HORIZONTAL_PERT_CAUG',
                          'FIST_RADIUS', 'WRIST_RADIUS', 'ELBOW_RADIUS', 'SHOULDER_RADIUS', 'FOREARM_LENGTH']
        use_parameters = np.array([1, 1, 1,
                                   1, 0, 1,
                                   0, 0, 0,
                                   1, 1, 0,
                                   0, 0, 0,
                                   0, 0, 0, 0, 0])
        all_parameters_min = np.array([0.0, 0.0, 0.0,
                                       0.3, 0.0, 0.0,
                                       5.0, 0.0, 0.0,
                                       -0.04, -0.04, -0.06,
                                       -0.03, -0.03, -0.03,
                                       0.04, 0.03, 0.04, 0.065, 0.28])
        all_parameters_max = np.array([1.0, 1.0, 1.0,
                                       0.8, 0.015, 1.0,
                                       10.0, 20.0, 1.0,
                                       0.04, 0.04, 0.0,
                                       0.03, 0.03, 0.03,
                                       0.06, 0.043, 0.06, 0.085, 0.35])
        all_parameters_initialization = (all_parameters_max+all_parameters_min)/2.
        all_parameters_scaling = (all_parameters_max - all_parameters_min)/4.

        parameters_min = []
        parameters_max = []
        parameters_init = []
        parameters_scaling = []
        self.parameters = []
        self.iteration_count = 0
        for i in xrange(len(use_parameters)):
            if use_parameters[i]:
                parameters_min.append(all_parameters_min[i])
                parameters_max.append(all_parameters_max[i])
                parameters_init.append(all_parameters_initialization[i])
                parameters_scaling.append(all_parameters_scaling[i])
                self.parameters.append(all_parameters[i])
                # print 'Using parameter: ', all_parameters[i]

        opts = {'seed': 1234, 'ftarget': 0., 'popsize': popsize, 'maxiter': maxiter, 'maxfevals': 1e8,
                'CMA_cmean': 0.5, 'scaling_of_variables': parameters_scaling,
                'bounds': [parameters_min,
                           parameters_max]}
        self.iteration_start_time = time.time()
        optimization_results = cma.fmin(self.objective_function,
                                        parameters_init,
                                        1.,
                                        options=opts)
        print 'Final Optimization Results: ', optimization_results
        print 'Final total time taken: ', time.time() - self.node_start_time

    def objective_function(self, current_parameters):
        # print 'Current parameters: ', current_parameters
        # print 'bend: ', self.simulator.cloth.bend_stiff[0]
        # print 'friction: ', self.simulator.cloth.friction
        print 'CMA Iteration count: ', self.iteration_count
        self.iteration_count += 1

        self.simulator.reset()
        good_height_perturbation = 0
        miss_height_perturbation = 0
        if len(current_parameters) != len(self.parameters):
            print 'I have a mismatch in number of parameters in cma and in named parameters given to cma'
            return None
        for i in xrange(len(current_parameters)):
            if self.parameters[i] == 'STRETCH':
                # self.simulator.cloth.hstretch_stiff[0] = current_parameters[i]
                # self.simulator.cloth.vstretch_stiff[0] = current_parameters[i]
                self.simulator.cloth.hstretch_stiff = [current_parameters[i], 1, 1, 1]
                self.simulator.cloth.vstretch_stiff = [current_parameters[i], 1, 1, 1]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'SHEAR':
                # self.simulator.cloth.shear_stiff[0] = current_parameters[i]
                self.simulator.cloth.shear_stiff = [current_parameters[i], 1, 1, 1]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'BEND':
                # print 'bend: ', self.simulator.cloth.bend_stiff[0]
                # self.simulator.cloth.bend_stiff[0] = current_parameters[i]
                self.simulator.cloth.bend_stiff = [current_parameters[i], 1, 1, 1]
                # print 'bend: ', self.simulator.cloth.bend_stiff[0]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'FRICTION':
                # print 'friction: ', self.simulator.cloth.friction
                self.simulator.cloth.friction = current_parameters[i]
                # print 'friction: ', self.simulator.cloth.friction
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'SELFCOLDIST':
                self.simulator.cloth.self_collision_distance = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'SELFCOLFRICTION':
                self.simulator.cloth.self_friction = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'STIFFPOWER':
                self.simulator.cloth.stiffpower = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ITERATION':
                self.simulator.cloth_solve_iteration_aftergrip = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'WIND':
                continue
                # self.simulator.random_wind.x = current_parameters[i]/4.
                # self.simulator.random_wind.z = (m.sqrt(1.-current_parameters[i]*current_parameters[i]))/4.
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ARM_HEIGHT_PERT_GOOD':
                good_height_perturbation = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ARM_HEIGHT_PERT_MISS':
                miss_height_perturbation = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ARM_HEIGHT_PERT_CAUG':
                caught_height_perturbation = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ARM_HORIZONTAL_PERT_GOOD':
                continue
            elif self.parameters[i] == 'ARM_HORIZONTAL_PERT_MISS':
                continue
            elif self.parameters[i] == 'ARM_HORIZONTAL_PERT_CAUG':
                continue
            elif self.parameters[i] == 'FIST_RADIUS':
                continue
                # self.simulator.fist_radius = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'WRIST_RADIUS':
                continue
                # self.simulator.wrist_radius = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'ELBOW_RADIUS':
                continue
                # self.simulator.elbow_radius = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'SHOULDER_RADIUS':
                continue
                # self.simulator.shoulder_radius = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
            elif self.parameters[i] == 'FOREARM_LENGTH':
                continue
                # self.simulator.forearm_length = current_parameters[i]
                print self.parameters[i], ' = ', current_parameters[i]
        # print 'bend: ', self.simulator.cloth.bend_stiff[0]
        # print 'friction: ', self.simulator.cloth.friction
        self.simulator.cloth.setParameters()
        # print 'bend: ', self.simulator.cloth.bend_stiff[0]
        # print 'friction: ', self.simulator.cloth.friction

        if good_height_perturbation:
            target_height = -0.13 + self.simulator.wrist_radius + 0.205 + good_height_perturbation
        else:
            target_height = -0.13 + self.simulator.wrist_radius + 0.205 + good_height_perturbation
        self.simulator.positionArm(-self.simulator.arm_horizontal_perturb, target_height, 0.)
        # print self.simulator.cloth.bend_stiff[0]
        self.initData()
        self.simulate()
        cost = 0.
        # self.experimentForce = np.array(self.experimentForce)
        self.gripperForce = np.array(self.gripperForce)
        dist_x_good, cost_x_good, path_x_good = mlpy.dtw_std(self.gripperForce[:, 0], self.good_trial_data[:, 2], dist_only=False)
        dist_z_good, cost_z_good, path_z_good = mlpy.dtw_std(self.gripperForce[:, 1], self.good_trial_data[:, 4], dist_only=False)
        print "Dist x good: ", dist_x_good
        print "Dist z good: ", dist_z_good
        cost += dist_x_good
        cost += dist_z_good

        self.simulator.reset()
        self.initData()
        # self.simulator.setParameters()
        if miss_height_perturbation:
            target_height = -0.13 + self.simulator.wrist_radius + miss_height_perturbation;
        else:
            target_height = -0.13 + self.simulator.wrist_radius

        self.simulator.positionArm(-self.simulator.arm_horizontal_perturb, target_height, 0.)
        self.simulate()
        self.gripperForce = np.array(self.gripperForce)
        dist_x_miss, cost_x_miss, path_x_miss = mlpy.dtw_std(self.gripperForce[:, 0], self.miss_trial_data[:, 2], dist_only=False)
        dist_z_miss, cost_z_miss, path_z_miss = mlpy.dtw_std(self.gripperForce[:, 1], self.miss_trial_data[:, 4], dist_only=False)
        print "Dist x miss: ", dist_x_miss
        print "Dist z miss: ", dist_z_miss
        cost += dist_x_miss
        cost += dist_x_miss

        # self.simulator.reset()
        print 'CMA single simulation run time: ', time.time() - self.iteration_start_time
        self.iteration_start_time = time.time()
        print 'Current CMA cost: ', cost
        return cost

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
        print('Total time to simulate:', time.time() - tt)

    def initData(self):
        # Simulation data
        self.simTime = 0
        self.simSteps = 0
        self.gripPos_start = self.simulator.getGripperPos()
        # print 'Max position the simulator will allow the grippers to move is (starting from 0.04 and decreasing):', -self.simulator.forearm_length + 0.44 - 0.85
        # print self.gripPos_start.x
        self.recordedTimes = []
        self.gripperForce = []
        # self.experimentForce = []
        self.gripperPos = []

    def getData(self):
        gripPos = self.simulator.getGripperPos()
        if np.abs(self.gripPos_start.x - gripPos.x) > 0.0001:

            # Get Simulation time
            t = self.simulator.recorded_time[-1]
            self.recordedTimes.append(t)

            # Get force data from gripper
            gripForce = self.simulator.rig_parts[1].recorded_forces[-1]
            self.gripperForce.append([gripForce.x, gripForce.y, gripForce.z])

            # Gripper position

            self.gripperPos.append([gripPos.x, gripPos.y, gripPos.z])

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
        print('Total time to simulate:', time.time() - tt)

if __name__ == '__main__':
    iterations = 20
    population = 40
    subject = 4
    cma_optimization = CMA_Optimization(iterations=iterations, population=population, subject=subject)

