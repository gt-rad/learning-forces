import os, sys, time, inspect
from datetime import datetime
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))
sys.path.insert(0, parentdir)
import build.pysim as pysim
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from OpenGL.GL import shaders
import numpy as np
from PIL import Image

'''
Used to test out and visualize new ideas within the gown simulator.
'''

class BaseSim(object):
    def __init__(self, maxSteps=1200, enableWind=False, pressurePointCount=314, showForcemap=True, dualArm=False, isArm=True, fullscreen=True):
        # Window dimensions
        self.width = 1920
        self.height = 1080
        self.fullscreen = fullscreen

        # Initialize Simulator
        if isArm:
            self.simulator = pysim.gownSimulator()
        else:
            self.simulator = pysim.shortsSimulator()
        self.simulator.initialize(maxSteps=maxSteps, enableWind=enableWind)

        # Initialize renderer
        self.renderer = pysim.Renderer()

        # Simulation data
        self.simTime = 0
        self.simStep = 0

        # Scene navigation
        self.oldMouseX = 0
        self.oldMouseY = 0
        self.leftMouseDown = False
        self.rightMouseDown = False
        self.ctrlDown = False
        self.camTranslateX = 0
        self.camTranslateY = 0
        self.camRotateX = 15
        self.camRotateY = 0
        self.camDistance = -2

        if dualArm:
            if isArm:
                # self.camRotateX = 15.6
                # self.camRotateY = -108.0
                # Differing image ratios
                self.camRotateX = 22.6
                self.camRotateY = -109.0
                self.camTranslateX = 0.1178
                self.camTranslateY = -0.148
                self.camDistance = -8.5
                '''self.camRotateX = 39
                self.camRotateY = -104
                self.camTranslateX = 0.1447
                self.camTranslateY = -0.171
                self.camDistance = -8.5'''
            else:
                self.camRotateX = 10.8
                self.camRotateY = -128.6
                self.camTranslateX = -0.073520833333
                self.camTranslateY = -0.302300833333
                # Differing image ratios
                # self.camTranslateX = -0.075300833333
                # self.camTranslateY = -0.329000833333
                self.camDistance = -10.68
                '''self.camRotateX = 15.8
                self.camRotateY = -125.4
                self.camTranslateX = -0.0531108333333
                self.camTranslateY = -0.280360833333
                self.camDistance = -9.84'''
        else:
            self.camRotateX = 39
            self.camRotateY = -104
            self.camTranslateX = -0.097
            self.camTranslateY = -0.1915
            self.camDistance = -8.5

        # Simulation and rendering
        self.simulating = False
        self.showObstacle = True
        self.showEndEffector = True
        self.showForcemap = showForcemap
        self.showMesh = True
        self.dualArm = dualArm
        self.isArm = isArm
	self.displayGripperMotion = False

        # Initialize all simulation data variables
        self.initialize()

        # Initialize all simulation data variables
        self.setupGlut()

        self.pressureShadeProgram = self.compileProgram(
            shaders.compileShader('''#version 130
                varying vec3 vert;
                void main(void)
                {
                    vert = gl_Vertex.xyz;
                    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
                }''', GL_VERTEX_SHADER),
            shaders.compileShader('''#version 130
                uniform vec4 points[%d];
                // uniform vec4 points[4];
                uniform vec3 pos;
                varying vec3 vert;
                vec3 HSVtoRGB(float h) {
                    // https://www.cs.rit.edu/~ncs/color/t_convert.html
                    vec3 rgb;
                    float s = 1.0; // Saturation
                    float v = 1.0; // Value
                    h /= 60;
                    int i = int(h);
                    float f = h - i;
                    float p = v * (1 - s);
                    float q = v * (1 - s * f);
                    float t = v * (1 - s * (1 - f));
                    switch(i) {
                        case 0:
                            rgb = vec3(v, t, p);
                            break;
                        case 1:
                            rgb = vec3(q, v, p);
                            break;
                        case 2:
                            rgb = vec3(p, v, t);
                            break;
                        case 3:
                            rgb = vec3(p, q, v);
                            break;
                        case 4:
                            rgb = vec3(t, p, v);
                            break;
                        default:
                            rgb = vec3(v, p, q);
                            break;
                    }
                    return rgb;
                }

                void main (void)
                {
                    float value = 0.0;
                    float alpha = 0.0;
                    // for (int i = 0; i < 4; i++) {
                    for (int i = 0; i < %d; i++) {
                        float dist = distance(points[i].xyz - pos, vert);
                        value += pow(0.5, 50000*pow(dist, 3)) * points[i].w;
                        if (points[i].w > 0.0)
                            // alpha += pow(0.5, 40000*pow(dist, 3));
                            alpha += exp(-100000000 * pow(dist, 5));

                        /*if (dist < 0.04 && points[i].w > 0.0) {
                            // value += exp(-20 * dist) * points[i].w;
                            // value += pow(0.5, 4000000*pow(dist, 4)) * points[i].w;
                            // value += pow(0.5, 700*pow(dist, 2)) * points[i].w;
                            value += pow(0.5, 40000*pow(dist, 3)) * points[i].w;
                            alpha = max(alpha, 1 - dist / 0.04);
                        } else {
                            value += pow(0.5, 40000*pow(dist, 3)) * points[i].w;
                            alpha += pow(0.5, 40000*pow(dist, 3)) * points[i].w;
                        }*/
                    }
                    value = clamp(value, 0.0, 1.0);
                    alpha = clamp(alpha * value * 10, 0.0, 1.0);
                    gl_FragColor.xyz = HSVtoRGB((1 - value) * 360 * 0.65);
                    gl_FragColor.a = alpha;
                }''' % (pressurePointCount, pressurePointCount), GL_FRAGMENT_SHADER))

    def initialize(self):
        '''
        This should be called after the simulator is initialized or reset.
        '''
        raise NotImplementedError('Implement me!')

    def recordData(self):
        raise NotImplementedError('Implement me!')

    def renderObjects(self):
        raise NotImplementedError('Implement me!')

    def renderDualObjects(self):
        raise NotImplementedError('Implement me!')

    def getPoints(self):
        raise NotImplementedError('Implement me!')

    def getTruePoints(self):
        raise NotImplementedError('Implement me!')

    def start(self):
        glutMainLoop()

    def setupGlut(self):
        # Initialize Glut, enable double buffering, and create window
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.width, self.height)
        glutCreateWindow('Robot Assisted Dressing Simulation')
        if self.fullscreen:
            glutFullScreen()
        # self.width = glutGet(GLUT_WINDOW_WIDTH)
        # self.height = glutGet(GLUT_WINDOW_HEIGHT)
        # print self.width, self.height

        # Callback for window display, when app is idle, and when app is resized
        glutDisplayFunc(self.render)
        glutIdleFunc(self.idle)
        glutReshapeFunc(self.reshape)

        # Callback for mouse button, mouse motion, and keyboard events
        glutMouseFunc(self.mousePress)
        glutMotionFunc(self.mouseMotion)
        glutKeyboardFunc(self.keyPress)
        glClearColor(0.5, 0.5, 0.5, 1.0)
        # glClearColor(67.0/255, 67.0/255, 67.0/255, 1.0)

    def compileProgram(self, *shaders):
        program = glCreateProgram()
        for shader in shaders:
            glAttachShader(program, shader)
        glLinkProgram(program)
        for shader in shaders:
            glDeleteShader(shader)

        return program

    def render(self):
        if self.simulating:
            # Perform simulation
            t = time.time()
            self.simulator.simulate(3)
            if self.simulator.recorded_time:
                # Record data once simulation has begun
                self.simTime += time.time() - t
                # Grab all data for this simulation step
                self.recordData()
            if self.simulator.simulated_step < -1:
                # Disable automatic simulation
                self.keyPress(' ', 0, 0)
                # Reset simulation
                self.keyPress('r', 0, 0)
                print 'Total simulation time:', self.simTime

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_COLOR_MATERIAL)
        glLoadIdentity()

        position = [10, 10, 0, 0]
        diffuse = [0.9, 0.9, 0.9, 1]
        glEnable(GL_LIGHT0)
        # Set directional light
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT0, GL_POSITION, position)
        # Set ambient light
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, diffuse)
        # Enable lighting
        glEnable(GL_LIGHTING)

        # Translate and rotate camera. Currently only zoom translation is enabled
        glTranslatef(self.camTranslateX, self.camTranslateY, self.camDistance)
        glRotatef(self.camRotateX, 1, 0, 0)
        glRotatef(self.camRotateY, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)

        # Render scene
        glEnable(GL_LIGHTING)
        clothColor = [1.0, 1.0, 1.0]
        # clothColor = [0.25, 0.75, 1.0]
        # clothColor = [0.4, 1.0, 0.85]
        objColor = [0.7, 0.4, 0.2, 0.95]
        if self.isArm:
            offset = [0, -0.095, 0.7]
            # Differing image ratios
            # offset = [0, -0.09, 0.65]
        else:
            offset = [0, 0.5, 0.65]
            offset = [0, 0.1, 1.1]
        if self.showMesh:
            self.renderer.renderMesh(self.simulator.cloth.cloth_mesh, clothColor)
            # for i in xrange(2 if self.dualArm else 1):
            #     # Draw the limbs and associated force maps
            #     if i == 1:
            #         glTranslatef(offset[0], offset[1], offset[2])
            #     # Draw the cloth garments
            #     self.renderer.renderMesh(self.simulator.cloth.cloth_mesh, clothColor)
            #     if i == 1:
            #         glTranslatef(-offset[0], -offset[1], -offset[2])

        for i in xrange(2 if self.dualArm else 1):
            # Draw the limbs and associated force maps
            if i == 1:
                glTranslatef(offset[0], offset[1], offset[2])
            if self.showObstacle:
                self.renderer.renderSphereManager(objColor)
            if self.showForcemap:
                # Setup pressure map shading for arm
                if i == 1:
                    points = self.getPoints()
                else:
                    points = self.getTruePoints()
                glUseProgram(self.pressureShadeProgram)
                glUniform4fv(glGetUniformLocation(self.pressureShadeProgram, 'points'), len(points), points)
                # Display hand, wrist, elbow, and shoulder
                spheres = self.simulator.getArmSpheres()
                for s in spheres:
                    self.createSphere(s.x, s.y, s.z, radius=s.w)
                # Reset glTranslatef offset
                glUniform3fv(glGetUniformLocation(self.pressureShadeProgram, 'pos'), 1, [0, 0, 0])
                # Display arm sections (conical frustums)
                self.renderer.renderSphereManager(objColor, drawSpheres=False)
                glUseProgram(0)
            if i == 1:
                glTranslatef(-offset[0], -offset[1], -offset[2])

        if self.showEndEffector:
            for i in xrange(2 if self.dualArm else 1):
                # Draw robot end effector
                if i == 1:
                    glTranslatef(offset[0], offset[1], offset[2])
                # Draw robot's end effector
                glDisable(GL_DEPTH_TEST)
                glDisable(GL_LIGHTING)
                for rigPart in self.simulator.rig_parts:
                    self.renderer.renderRigPart(rigPart, objColor)
                glEnable(GL_LIGHTING)
                glEnable(GL_DEPTH_TEST)
                if i == 1:
                    glTranslatef(-offset[0], -offset[1], -offset[2])

        # Allow child class to perform any computations
        for i in xrange(2 if self.dualArm else 1):
            # Draw the limbs and associated force maps
            if i == 1:
                glTranslatef(offset[0], offset[1], offset[2])
                self.renderDualObjects()
                glTranslatef(-offset[0], -offset[1], -offset[2])
            else:
                self.renderObjects()

        # Display the gripper spline
        if self.displayGripperMotion:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            objColor = [0.6, 0.6, 0.6, 0.9]
            # glDisable(GL_DEPTH_TEST)
            for rigPart in self.simulator.rig_parts:
                self.renderer.renderRigMotion(rigPart, objColor)
            # glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)

        glDisable(GL_LIGHTING)

        glutSwapBuffers()

        if self.simulator.recorded_time:
            # Increment to next simulation step
            self.simStep += 1

    def createSphere(self, x, y, z, r=-1, g=-1, b=-1, a=-1, radius=0.01):
        # Creates a single sphere in the scene
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glPushMatrix()
        glTranslatef(x, y, z)
        if r != -1 and g != -1 and b != -1 and a != -1:
            glColor4f(r, g, b, a)
        else:
            glUniform3fv(glGetUniformLocation(self.pressureShadeProgram, 'pos'), 1, [x, y, z])
        glutSolidSphere(radius, 30, 30)
        glPopMatrix()
        glDisable(GL_BLEND)

    def createRect(self, x, r, g, b, a, size=0):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(r, g, b, a)
        glPushMatrix()
        glTranslatef(x, 0, 0)
        glRotatef(90.0, 0, 1, 0)
        glRectd(-0.075 - size, self.armYOffeset + 0.05 - size, 0.05 + size, self.armYOffeset + 0.175 + size)
        glPopMatrix()
        glDisable(GL_BLEND)

    def createLine(self, p1, p2, r=-1, g=-1, b=-1, a=-1, linewidth=1):
        # Creates a single line in the scene
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(linewidth)
        glColor4f(r, g, b, a)
        glBegin(GL_LINES)
        glVertex3f(*p1)
        glVertex3f(*p2)
        glEnd()
        glLineWidth(1)
        glDisable(GL_BLEND)

    def screenshotCrop(self):
        t = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Create directories if they do not already exist
        directory = 'images'

        if self.dualArm:
            if self.isArm:
                dualCrop = [690, 360, 865, 248, 85, 245]
            else:
                dualCrop = [690, 360, 715, 540, 250, 5]

            self.saveScreenshot(directory, filename=t + '.png', dualCrop=dualCrop)
        else:
            self.saveScreenshot(directory, filename=t + '.png')

    def saveScreenshot(self, directory, filename=None, dualCrop=None):
        # Create directory and update filename
        if not os.path.exists(directory):
            os.makedirs(directory)
        if filename is None:
            filename = '%s_step_%d.png' % ('Arm' if self.isArm else 'Leg', self.simStep)
        else:
            filename = ('Arm' if self.isArm else 'Leg') + filename

        # Read pixels from window
        glReadBuffer(GL_FRONT)
        pixels = glReadPixels(0, 0, self.width, self.height, GL_RGB, GL_UNSIGNED_BYTE)

        # Save pixels to image file
        image = Image.frombytes("RGB", (self.width, self.height), pixels)
        image = image.transpose(Image.FLIP_TOP_BOTTOM)

        if dualCrop is not None:
            w, h, xTruth, yTruth, xEst, yEst = dualCrop
            truth = image.crop((xTruth, yTruth, xTruth + w, yTruth + h))
            est = image.crop((xEst, yEst, xEst + w, yEst + h))
            combined = Image.fromarray(np.vstack((truth, est)))
            combined.save(os.path.join(directory, 'combined_' + filename))

        image.save(os.path.join(directory, filename))
        print 'Screenshot saved. Time step', self.simStep

    def idle(self):
        glutPostRedisplay()

    def reshape(self, width, height):
        # Reshape window size
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(5, float(width) / height, 0.1, 100000.0)
        glMatrixMode(GL_MODELVIEW)

    def mousePress(self, mouseButton, mouseButtonState, mouseX, mouseY):
        # Capture when left and right mouse buttons are clicked
        mod = glutGetModifiers()
        self.ctrlDown = mod & GLUT_ACTIVE_CTRL
        if mouseButtonState == GLUT_DOWN:
            if mouseButton == GLUT_LEFT_BUTTON:
                self.leftMouseDown = True
            elif mouseButton == GLUT_RIGHT_BUTTON:
                self.rightMouseDown = True
            self.oldMouseX = mouseX
            self.oldMouseY = mouseY
        elif mouseButtonState == GLUT_UP:
            self.leftMouseDown = False
            self.rightMouseDown = False

    def mouseMotion(self, mouseX, mouseY):
        if self.leftMouseDown:
            if self.ctrlDown:
                # Translate
                self.camTranslateX += (mouseX - self.oldMouseX) / 6000.0 * (-self.camDistance / 2.0)
                self.camTranslateY -= (mouseY - self.oldMouseY) / 6000.0 * (-self.camDistance / 2.0)
            else:
                # Rotate
                self.camRotateX += (mouseY - self.oldMouseY) / 5.0
                self.camRotateY += (mouseX - self.oldMouseX) / 5.0
        if self.rightMouseDown:
            self.camDistance -= (mouseY - self.oldMouseY) / 100.0
        self.oldMouseX = mouseX
        self.oldMouseY = mouseY

    def keyPress(self, key, x, y):
        if key == '\x1b' or key == 'q':
            sys.exit(0)
        elif key == ' ':
            self.simulating = not self.simulating
        elif key == 's':
            self.simulating = True
            self.render()
            self.simulating = False
        elif key == 'r':
            self.simulator.reset()
            self.simStep = 0
            self.simTime = 0
            self.initialize()
        elif key == 'o':
            self.showObstacle = not self.showObstacle
        elif key == 'e':
            self.showEndEffector = not self.showEndEffector
        elif key == 'f':
            self.showForcemap = not self.showForcemap
        elif key == 'c':
            self.showMesh = not self.showMesh
        elif key == 'p':
            # Display camera parameters
            print 'self.camRotateX =', self.camRotateX, '\nself.camRotateY =', self.camRotateY, '\nself.camTranslateX =', self.camTranslateX, '\nself.camTranslateY =', self.camTranslateY, '\nself.camDistance =', self.camDistance
	elif key == 'g':
            self.displayGripperMotion = not self.displayGripperMotion
        elif key == 'z':
            self.screenshot()

    def shutdown(self):
        self.simulator.destroy()

