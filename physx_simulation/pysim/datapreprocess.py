import os, sys, h5py, Queue
import numpy as np
from datetime import datetime
from sklearn import cluster, neighbors
import multiprocessing

import util

def createMinibatch(directories, functions, labelFunction, isArm=True, knearest=5, filesPerBatch=250, description=None):
    '''
    Reads in raw data pickle files and stores processed data into mini-batch files.
    Created mini-batch files are ready for machine learning.
    '''
    # Create directory name for mini-batches
    directory = 'Arm_' if isArm else 'Leg_'
    directory += datetime.now().strftime('%Y-%m-%d_%H-%M-%S_') + ''.join([fun(None, retName=True) for fun in functions]) + '_' + labelFunction(None, None, retName=True)
    if description:
        directory += '_' + description

    cpus = multiprocessing.cpu_count() / 6
    processes = Queue.Queue(maxsize=cpus)
    for i, rawdata in enumerate(util.loadRawData(directories, batchsize=filesPerBatch)):
        # Launch a new process to process a batch of files
        p = multiprocessing.Process(target=processAndSave, args=(rawdata, functions, labelFunction, knearest, i, directory, isArm))
        processes.put(p)
        p.start()
        if processes.full():
            # Wait for half of current processes to finish
            print 'Waiting for %d processes to complete' % (cpus / 2)
            for j in xrange(cpus/2):
                processes.get().join()
    # Wait for any left over processes still running
    for i in xrange(processes.qsize()):
        processes.get().join()

def processAndSave(rawdata, functions, labelFunction, knearest, batchNum, directory, isArm=True):
    # Combine features to create the dataset, X
    X = np.concatenate([fun(rawdata) for fun in functions], axis=2)
    # print 'X shape:', X.shape

    # Create the label set Y
    Y = labelFunction(np.array(rawdata['forcemap']), np.array(rawdata['armSpheres']), isArm=isArm, knearest=knearest)
    # print 'Y shape:', Y.shape

    # Save this batch to file
    util.saveData('batch_%d' % batchNum, {'X': X, 'Y': Y}, directory=directory, parentDir=util.batchDir)

def gripperToArmPosition(rawdata, retName=False):
    if retName:
        return 'Position'
    # Determine Euclidean distance from gripper to arm
    gripToArmPos = np.linalg.norm(np.array(rawdata['gripperPos']) - np.array(rawdata['armSpheres'])[:, 0, np.newaxis, :3], axis=2)
    # Add an empty axis for compatibility
    gripToArmPos = gripToArmPos[:, :, np.newaxis]
    return gripToArmPos

def gripperForce(rawdata, retName=False):
    if retName:
        return 'Force'
    # 3 dimensional gripper forces
    gripForce = np.array(rawdata['gripperForce'])
    return gripForce

def gripperTorque(rawdata, retName=False):
    if retName:
        return 'Torque'
    # 3 dimensional gripper torques
    gripTorque = np.array(rawdata['gripperTorque'])
    return gripTorque

def gripperVelocity(rawdata, retName=False, smoothing=False, prevData=None):
    if retName:
        return 'Velocity'
    gripperPos = np.array(rawdata['gripperPos'])
    times = np.array(rawdata['recordedTimes'])
    # Velocity is undefined at the first time step, so we simply use velocity at the second time step for both
    # the first and second time step
    if prevData is not None:
        prevPos = np.array(prevData['gripperPos'])
        prevTimes = np.array(prevData['recordedTimes'])
        velocities = gripperPos - prevPos
        velocities /= (times - prevTimes)[:, :, np.newaxis]
    elif len(gripperPos.shape) > 1:
        velocities = gripperPos[:, [1] + range(1, gripperPos.shape[1])] - gripperPos[:, [0] + range(gripperPos.shape[1] - 1)]
        velocities /= (times[:, [1] + range(1, times.shape[1])] - times[:, [0] + range(times.shape[1] - 1)])[:, :, np.newaxis]
    else:
        velocities = []
        for x, y in zip(gripperPos, times):
            velocities.append(x[[1] + range(1, len(x))] - x[[0] + range(len(x) - 1)])
            velocities[-1] /= (y[[1] + range(1, len(y))] - y[[0] + range(len(y) - 1)])[:, np.newaxis]
    return velocities

def rawPressuremap(allPressuremaps, allArmSpheres, isArm=True, knearest=5, retPoints=False, retActivations=True, rotateFist=[0, 0, 0, 0], rotateArm=[0, 0, 0, 0], precomputedPoints=None, retName=None):
    '''
    Determine a discrete vectorization of points along the arm, then use nearest neighbors to map force points to a
    point along the arm. Mapped pressure points force magnitudes are summed into each nearest neighbor (in case multiple
    pressure points map to the same nearest neighbor point). This creates a pressuremap activation vector.
    knearest is the number of nearest neighbors that each force point is distributed amongst.
    '''
    if retName:
        return 'rawPressuremap'
    allDiscretePressuremaps = []
    for pressuremap, armSpheres in zip(allPressuremaps, allArmSpheres):
        points = []
        # Account for the arm being moved +0.1 m in the X direction without a new position being recorded
        # armSpheres[:, 0] += 0.1
        if precomputedPoints is not None:
            points = precomputedPoints
        else:
            if isArm:
                # Add points around the fist
                # points.extend(util.fibonacciSphere(n=50, r=armSpheres[0, 3], center=armSpheres[0, :3]))
                points.extend(util.fibonacciSphere(n=51, r=armSpheres[0, 3], center=armSpheres[0, :3], excludeSections=[1], excludeBuffer=0.02, rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3]))
                # Add points around the elbow
                points.extend(util.fibonacciSphere(n=50, r=armSpheres[2, 3], center=armSpheres[2, :3], excludeSections=[0, 5], rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3]))
                # Add points along forearm
                points.extend(util.conicalFrustumPoints(armSpheres[1], armSpheres[2], sections=10, arcLengthBetweenPoints=0.025, pushOut=True, rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3]))
                # Add points along upper arm
                points.extend(util.conicalFrustumPoints(armSpheres[2], armSpheres[3], sections=10, arcLengthBetweenPoints=0.025, pushOut=True))
            else:
                # Add points along the knee
                points.extend(util.fibonacciSphere(n=50, r=armSpheres[1, 3], center=armSpheres[1, :3], excludeSections=[0, 1], excludeBuffer=0.02))
                # Add points around the toes
                points.extend(util.fibonacciSphere(n=52, r=armSpheres[3, 3], center=armSpheres[3, :3], excludeSections=[3], rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3], rotationArmAngle=rotateArm[-1], rotationArmAxis=rotateArm[:3], rotationArmCenter=armSpheres[2, :3]))
                # Add points around the heel
                points.extend(util.fibonacciSphere(n=50, r=armSpheres[4, 3], center=armSpheres[4, :3], excludeSections=[1, 2], excludeBuffer=0.031, rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3], rotationArmAngle=rotateArm[-1], rotationArmAxis=rotateArm[:3], rotationArmCenter=armSpheres[2, :3]))
                # Add points along thigh
                points.extend(util.conicalFrustumPoints(armSpheres[0], armSpheres[1], sections=12, arcLengthBetweenPoints=0.0325))
                # Add points along leg
                points.extend(util.conicalFrustumPoints(armSpheres[1], armSpheres[2], sections=12, arcLengthBetweenPoints=0.0325))
                # Add points along foot
                points.extend(util.conicalFrustumPoints(armSpheres[2], armSpheres[3], sections=6, arcLengthBetweenPoints=0.0325, rotationAngle=rotateFist[-1], rotationAxis=rotateFist[:3], rotationCenter=armSpheres[2, :3], rotationArmAngle=rotateArm[-1], rotationArmAxis=rotateArm[:3], rotationArmCenter=armSpheres[2, :3]))
            points = np.array(points)
        # Nearest neighbors to map force points to discrete points along the arm
        nn = neighbors.NearestNeighbors(n_neighbors=knearest, radius=0.1).fit(points)
        discretePressuremap = []
        for forces in pressuremap:
            if not forces:
                # No forces, append an empty force vector
                discretePressuremap.append(np.zeros(len(points)))
                continue
            forces = np.array(forces)
            # Map force points to discrete arm points
            distances, indices = nn.kneighbors(forces[:, :3])
            pressuremapDistribution = np.zeros(len(points))
            for force, nnIndex, nnDist in zip(forces, indices, distances):
                if knearest == 1:
                    # Add this force point's magnitude to the nearest point on the arm
                    pressuremapDistribution[nnIndex] += force[-1]
                else:
                    # Distribute the force throughout the k nearest points based on distances
                    ratios = (1.0 - nnDist / np.sum(nnDist)) / (knearest - 1)
                    for index, ratio in zip(nnIndex, ratios):
                        pressuremapDistribution[index] += ratio * force[-1]
            discretePressuremap.append(pressuremapDistribution)
        allDiscretePressuremaps.append(discretePressuremap)
    allDiscretePressuremaps = np.array(allDiscretePressuremaps)
    if retActivations:
        # Include not only the force value at every discrete point, but also if each discrete force is activated or not.
        # Create a duplicate array of the pressuremap with valuies of 0.1 if force > 0, else -0.1 if force == 0
        activations = np.copy(allDiscretePressuremaps)
        activations[activations > 0] = 0.1
        activations[activations == 0] = -0.1
        if retPoints:
            return points, np.concatenate((allDiscretePressuremaps, activations), axis=2)
        return np.concatenate((allDiscretePressuremaps, activations), axis=2)
    else:
        if retPoints:
            return points, np.array(allDiscretePressuremaps)
        return np.array(allDiscretePressuremaps)

if __name__ == '__main__':
    # Build a set of function processing combinations
    funSet = []
    funSet.append([gripperForce, gripperTorque])
    funSet.append([gripperForce, gripperTorque, gripperVelocity])

    # rawPressuremap
    createMinibatch(['2016-08-31_arm'], funSet[1], rawPressuremap, isArm=True, knearest=5, filesPerBatch=100, description='neoarm')
    createMinibatch(['2016-09-06_leg'], funSet[1], rawPressuremap, isArm=False, knearest=5, filesPerBatch=32, description='neoleg')

