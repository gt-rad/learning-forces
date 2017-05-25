import os, sys, glob, gc, joblib
import numpy as np
from keras.models import model_from_json
if sys.version_info >= (3, 0):
    import pickle
else:
    import cPickle as pickle

dataDir = os.path.dirname(os.path.realpath(__file__))
batchDir = os.path.join(dataDir, 'minibatches')
rawDataDir = os.path.join(dataDir, 'rawdata')
modelsDir = os.path.join(dataDir, 'models')
comparisonDir = os.path.join(dataDir, 'comparison')
verbose = True

def loadRawData(directories, batchsize=0):
    '''
    Returns a generator of dictionaries that contains aggregate data from all files within each specified directory.
    Each dictionary is filled with batchsize files.
    '''
    data = dict()
    filenames = getFilenames(directories, parentDir=rawDataDir)
    if verbose: print 'Found %d files in all provided directories' % len(filenames)
    # Fill the dictionary with data from all the files
    for i, filename in enumerate(filenames):
        filedata = loadFile(filename)
        for key, value in filedata.iteritems():
            if key not in data:
                data[key] = []
            data[key].append(np.array(value))
            key = None
            value = None
        filedata = None
        if verbose and (i + 1) % 50 == 0:
            print '%d files loaded' % (i + 1)
        if batchsize > 0 and (i + 1) % batchsize == 0:
            # Return a completed batch of load files
            if verbose: print 'Batch of %d files loaded' % (i + 1)
            yield data
            data = dict()
            # Clean up memory
            gc.collect()
    if data:
        # Yield any remaining data that has not been yielded yet
        yield data

def loadFile(filename, directory=None, parentDir=None):
    '''
    Load a single pickle file.
    '''
    if directory or parentDir:
        filename = os.path.join(parentDir, directory, '%s.pkl' % filename)
    with open(filename, 'rb') as f:
        return pickle.load(f)

def loadModel(modelDir, epoch):
    '''
    Loads and constructs a Keras model from a previously saved model at a given training epoch.
    The returned model need not be compiled before use.
    '''
    archFilename = os.path.join(modelsDir, modelDir, 'architectureLSTM.json')
    weightsFilename = os.path.join(modelsDir, modelDir, 'weights_%d.h5' % epoch)
    if not fileExists(archFilename) or not fileExists(weightsFilename):
        print 'Could not find architecture or weight files in model directory:', modelDir
        return None
    with open(archFilename, 'r') as f:
        model = model_from_json(f.read())
    model.load_weights(weightsFilename)
    return model

def getFilenames(directories, parentDir='', namesearch='*'):
    '''
    Get full filenames of all files within the specified directories.
    '''
    filenames = []
    for directory in directories:
        # Directory of data
        filedir = os.path.join(parentDir, directory)
        if not os.path.exists(filedir):
            print 'Directory does not exist:', filedir
            continue
        # Find all files in the given directory
        filenames.extend(glob.glob(os.path.join(filedir, namesearch)))
    return filenames

def fileExists(filename, directory='', parentDir=''):
    fullFilename = os.path.join(parentDir, directory, filename if '.' in filename else '%s.pkl' % filename)
    return os.path.isfile(fullFilename)

def verifyFilename(filename):
    '''
    Creates parent directories and verifies if the file already exists. Returns False if the file already exists.
    '''
    filedir = os.path.dirname(filename)
    if not os.path.exists(filedir):
        os.makedirs(filedir)
    elif os.path.isfile(filename):
        return False
    return True

def saveData(filename, data, directory='', parentDir=dataDir):
    '''
    Saves data to a pickle file. Returns False if unable to save the data to the specified file.
    '''
    fullFilename = os.path.join(parentDir, directory, '%s.pkl' % filename)
    if not verifyFilename(fullFilename):
        return False
    # Save the data to a pickle file
    # joblib.dump(data, fullFilename, protocol=pickle.HIGHEST_PROTOCOL)
    with open(fullFilename, 'wb') as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    return True

def saveLSTMModel(model, modelDir, epoch):
    '''
    Saves a Keras LSTM network model architecture and weights into a specified directory.
    Returns False if the model (epoch) already exists in the specified modelDir.
    '''
    archName = None
    if epoch == 0:
        # Save model only on the first epoch
        archName = 'architectureLSTM'
    weightsName = 'weights_%d' % epoch
    # Save LSTM network
    return saveGenericModel(model, modelDir, archName=archName, weightsName=weightsName)

def saveGenericModel(model, modelDir, archName=None, weightsName=None):
    '''
    Saves a Keras neural network model architecture and weights into a specified directory.
    Returns False if the model or weights already exist in the specified modelDir.
    '''
    if archName is not None:
        # Save model architecture as JSON format
        archFilename = os.path.join(modelsDir, modelDir, '%s.json' % archName)
        if not verifyFilename(archFilename):
            print 'Model architecture already exists:', weightsFilename
            return False
        with open(archFilename, 'w') as f:
            f.write(model.to_json())

    if weightsName is not None:
        # Save model weights in HDF5 format (requires HDF5 and h5py to be install)
        weightsFilename = os.path.join(modelsDir, modelDir, '%s.h5' % weightsName)
        if not verifyFilename(weightsFilename):
            print 'Weights model already exists:', weightsFilename
            return False
        model.save_weights(weightsFilename)
    return True

def fibonacciSphere(n=50, r=0.05, center=[0, 0, 0], excludeSections=[], excludeBuffer=0.01, rotationAngle=0, rotationAxis=[0, 0, 0], rotationCenter=[0, 0, 0], rotationArmAngle=0, rotationArmAxis=[0, 0, 0], rotationArmCenter=[0, 0, 0]):
    # NOTE: When the arm is rotated, the generated fibonacci spherical points will be incorrect as they do not account
    # for rotation along an axis with respect to a center location. We fix this using a rotation matrix.

    # Rotate center of sphere to account for the arm having already been rotated
    if rotationArmAngle != 0 and rotationArmAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationArmAxis, rotationArmAngle)
        center = np.dot(np.array(center) - np.array(rotationArmCenter), matrix) + np.array(rotationArmCenter)

    # Rotate center of sphere to account for the arm having already been rotated
    if rotationAngle != 0 and rotationAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationAxis, rotationAngle)
        center = np.dot(np.array(center) - np.array(rotationCenter), matrix) + np.array(rotationCenter)

    golden_angle = np.pi * (3 - np.sqrt(5))
    theta = golden_angle * np.arange(n)
    z = np.linspace(1 - 1.0 / n, 1.0 / n - 1, n)
    radius = np.sqrt(1 - z * z) * r
    z = z * r

    points = np.zeros((n, 3))
    points[:, 0] = radius * np.cos(theta) + center[0]
    points[:, 1] = radius * np.sin(theta) + center[1]
    points[:, 2] = z + center[2]

    # Exclude points that lie within various sections
    # Section 0 = +x axis half of sphere, 1 = -x axis, 2 = +y, 3 = -y, 4 = +z, 5 = -z
    for section in excludeSections:
        axis = section / 2
        pointsAxis = points[:, axis]
        centerAxis = np.array(center)[axis]
        if section % 2 == 0:
            # Exclude positive axis. Exclude points that have larger values along the axis than the center point
            points = points[pointsAxis <= centerAxis + excludeBuffer]
        else:
            # Exclude negative axis. Exclude points that have smaller values along the axis than the center point
            points = points[pointsAxis >= centerAxis - excludeBuffer]

    # Rotate points back onto the arm based on a center point
    if rotationAngle != 0 and rotationAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationAxis, -rotationAngle)
        points = np.dot(points - np.array(rotationCenter), matrix) + np.array(rotationCenter)

    # Rotate points back onto the arm based on a center point
    if rotationArmAngle != 0 and rotationArmAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationArmAxis, -rotationArmAngle)
        points = np.dot(points - np.array(rotationArmCenter), matrix) + np.array(rotationArmCenter)

    return points

def rotationMatrix(axis, theta):
    '''
    Return the rotation matrix associated with counterclockwise rotation about the given axis by theta radians.
    Uses the Euler-Rodrigues formula.
    '''
    axis = np.array(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta/2.0)
    b, c, d = -axis*np.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def conicalFrustumPoints(sphere1, sphere2, sections=10, arcLengthBetweenPoints=0.025, pushOut=False, rotationAngle=0, rotationAxis=[0, 0, 0], rotationCenter=[0, 0, 0], rotationArmAngle=0, rotationArmAxis=[0, 0, 0], rotationArmCenter=[0, 0, 0]):
    '''
    Creates a set of points around a conical frustum.
    Check out: http://mathworld.wolfram.com/ConicalFrustum.html
    and: http://math.stackexchange.com/questions/73237/parametric-equation-of-a-circle-in-3d-space
    sphere = [x, y, z, r]
    '''
    points = []
    # Define sphere radii
    r1 = sphere1[3]
    r2 = sphere2[3]
    # Define sphere center points
    p1 = sphere1[:3]
    p2 = sphere2[:3]

    # Rotate center of sphere to account for the arm having already been rotated
    if rotationArmAngle != 0 and rotationArmAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationArmAxis, rotationArmAngle)
        p1 = np.dot(np.array(p1) - np.array(rotationArmCenter), matrix) + np.array(rotationArmCenter)
        p2 = np.dot(np.array(p2) - np.array(rotationArmCenter), matrix) + np.array(rotationArmCenter)

    # Rotate center of sphere to account for the arm having already been rotated
    if rotationAngle != 0 and rotationAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationAxis, rotationAngle)
        p1 = np.dot(np.array(p1) - np.array(rotationCenter), matrix) + np.array(rotationCenter)
        p2 = np.dot(np.array(p2) - np.array(rotationCenter), matrix) + np.array(rotationCenter)

    axisVector = p2 - p1
    # Normalize axis vector to unit length
    axisVector = axisVector / np.linalg.norm(axisVector)
    orthoVector = orthogonalVector(axisVector)
    # Normalize orthogonal vector to unit length
    orthoVector = orthoVector / np.linalg.norm(orthoVector)
    # Determine normal vector through cross product (this will be of unit length)
    normalVector = np.cross(axisVector, orthoVector)

    # Determine the section positions along the frustum at which we will create point around in a circular fashion
    sectionPositions = [(p2 - p1) / (sections + 1) * (i + 1) for i in xrange(sections)]
    for i, sectionPos in enumerate(sectionPositions):
        # Determine radius and circumference of this section
        sectionRadius = r1 + (r2 - r1) * (i / float(sections + 1))
        sectionCircumference = 2 * np.pi * sectionRadius
        # Determine the angle difference (in radians) between points
        thetaDist = arcLengthBetweenPoints / sectionRadius
        if pushOut:
            sectionRadius += 0.0025
        for j in xrange(int(2*np.pi / thetaDist)):
            theta = thetaDist * j
            # Determine cartesian coordinates for the point along the circular section of the frustum
            pointOnCircle = p1 + sectionPos + sectionRadius*np.cos(theta)*orthoVector + sectionRadius*np.sin(theta)*normalVector
            points.append(pointOnCircle)

    # Rotate points back onto the arm based on a center point
    if rotationAngle != 0 and rotationAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationAxis, -rotationAngle)
        points = np.dot(points - np.array(rotationCenter), matrix) + np.array(rotationCenter)

    # Rotate points back onto the arm based on a center point
    if rotationArmAngle != 0 and rotationArmAxis != [0, 0, 0]:
        matrix = rotationMatrix(rotationArmAxis, -rotationArmAngle)
        points = np.dot(points - np.array(rotationArmCenter), matrix) + np.array(rotationArmCenter)

    return points

def orthogonalVector(v):
    '''
    Two Euclidean vectors are orthogonal if and only if their dot product is zero.
    '''
    if v[0] == v[1] == 0:
        if v[2] == 0:
            # v is the zero vector (0, 0, 0)
            raise ValueError('zero vector')
        # v is the vector (0, 0, v.z)
        return np.array([0, 1, 0])
    return np.array([-v[1], v[0], 0])

def pickleToJoblib(parentDir=dataDir):
    '''
    Convert pickle files to Joblib files
    '''
    for root, dirnames, filenames in os.walk(parentDir):
        if 'models' in root:
            continue
        newDir = root.replace('pyData', 'pyDataJoblib')
        if not os.path.exists(newDir):
            os.makedirs(newDir)
        else:
            continue
        print 'Converting %d files' % len(filenames)
        for i, filename in enumerate(filenames):
            with open(os.path.join(root, filename), 'rb') as f:
                joblib.dump(pickle.load(f), os.path.join(newDir, filename), protocol=pickle.HIGHEST_PROTOCOL)
            if (i + 1) % 100 == 0:
                print 'Converted %d files' % (i + 1)

if __name__ == '__main__':
    pass

