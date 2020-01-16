
# Import Modules
import os
import sys
import threading
import rospy
import asyncore
import subprocess
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
from Common.system_config import   EbolabotSystemConfig
sys.path.append(ebolabot_root)
#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/USBControllers'))
import gamepad
from task_generator import TaskGenerator
import time
import csv
from sspp.service import Service
from sspp.topic import MultiTopicListener
from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.math import so3,se3,vectorops
from std_msgs.msg import String, Int8
# from baxter_pykdl import baxter_kinematics
import numpy as np
#from UI.utils.gripper_controller import *
#icon_flag = 0
#imaging stuff
try:
    from PIL import Image
except ImportError, err:
    import Image

#set this -1 for view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1
''' gripper Mode: power, precision '''
#GripperMode = {}
#GripperMode['left'] = 'power'
#GripperMode['right'] = 'power'

''' Hold Mode: free, hold '''
HoldMode = {}
HoldMode['left'] = 'free'
HoldMode['right'] = 'free'

HoldPose = {}
HoldPose['left'] = 0
HoldPose['right'] = 0

TuckPose = {}
TuckPose['left'] = [-0.05897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, 2.475]
TuckPose['right'] = [0.05897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, -2.475]

TuckStatus = {}
TuckStatus['left'] = False
TuckStatus['right'] = False

gamepad_switch = "/transcript"  # to switch camera view
M_limit = 0.04
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))


class GLTexture:
    def __init__(self,fn=None):
        self.glid = None
        if fn:
            self.loadImage(fn)
    def destroy(self):
        glDeleteTextures([self.glid])

    def setBytes(self,w,h,buffer,glformat=GL_RGBA):
        self.w,self.h = w,h
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, glformat, w, h, 0,
            glformat, GL_UNSIGNED_BYTE, buffer
        )
    def loadImage(self,fn):
        im = Image.open(fn)
        try:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBA",0,-1)
        except SystemError:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBX",0,-1)
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, self.w, self.h, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, image
        )
        return True

    def enable(self,smooth=True,glmode=GL_MODULATE):
        glEnable(GL_TEXTURE_2D)
        if smooth:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        else:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, glmode)
        glBindTexture(GL_TEXTURE_2D,self.glid)

    def disable(self):
        glDisable(GL_TEXTURE_2D)

    def blit(self,x,y,w=None,h=None):
        if w==None: w = self.w
        if h==None: h = self.h
        self.enable()
        glDisable(GL_LIGHTING)
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1)
        glVertex2f(x,y)
        glTexCoord2f(0,0)
        glVertex2f(x,y+h)
        glTexCoord2f(1,0)
        glVertex2f(x+w,y+h)
        glTexCoord2f(1,1)
        glVertex2f(x+w,y)
        glEnd()
        self.disable()


class MyWidgetPlugin(GLPluginBase):
    def __init__(self,taskGen):
        GLPluginBase.__init__(self)
        self.taskGen = taskGen

    def initialize(self):
        GLPluginBase.initialize(self)
        self.images = {}
        self.images['Cartesian position'] = GLTexture("UI/Resources/cartesian-translation.png")
        self.images['Cartesian rotation'] = GLTexture("UI/Resources/cartesian-rotation.png")
        self.images['Joint angles'] = GLTexture("UI/Resources/joint.png")
        self.images['left'] = GLTexture("UI/Resources/left-arm.png")
        self.images['right'] = GLTexture("UI/Resources/right-arm.png")
        self.images['arm'] = GLTexture("UI/Resources/ArmMode.png")
        self.images['base'] = GLTexture("UI/Resources/BaseMode.png")
        self.images['auto'] = GLTexture("UI/Resources/AutoMode.png")
        # Logging images
        self.images['log']=[None,None]

        self.images['log'][1] = GLTexture("UI/Resources/log-on.png")
        self.images['log'][0] = GLTexture("UI/Resources/log-off.png") 

        return True

    def keyboardfunc(self,c,x,y):
        c=c.lower()

        if c=='l':
            self.taskGen.log ^=1 
            print("Logging: " + str(self.taskGen.log ) )

            
        elif c == 'a' or c == 's' or c == 'd' or c == 'f':
            self.taskGen.flag = c
        #disable label (keeping it seperate for clarity)
        elif c == '0':
            self.taskGen.flag = c


    def display(self):
        pass

    def display_screen(self):
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)
        #global icon_flag
        if self.taskGen.limb in self.images:
            self.images[self.taskGen.limb].blit(20,40)
        if self.taskGen.controlMode() in self.images:
            self.images[self.taskGen.controlMode()].blit(40+64,40)
        if self.taskGen.controlSet in self.images: #arm/base
            self.images[self.taskGen.controlSet].blit(60+128,40)
        if 'log' in self.images:
            self.images['log'][self.taskGen.log].blit(20,74+40)
        '''if icon_flag == 1:
            self.images['auto'].blit(20,154)'''
        
    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        if type=='button':
            if args=='left':
                self.taskGen.limb = 'left'
            if args=='right':
                self.taskGen.limb = 'right'


class GamePadTaskGenerator(TaskGenerator):

    def name(self):
        return "Prince Philip is a camera"

    def __init__(self):
        print("*****************************************************************************")
        self.serviceThread = None
        #self.gripperController = None
        self.j = None
        self.limb = 'left'
        self.controlSet = 'arm'
        self.lastState = {}
        self.plugin = None
        # set initial values
        self.baseSensedVelocity = [0.0, 0.0, 0.0]
        self.baseCommandVelocity = [0.0, 0.0, 0.0]
        self.log = False
        # === joint control ===
        self.jointControlRatio = 0.4

        # == arm position ===
        self.ArmPosition = [[0.0]*7, [0.0]*7]
        self.robotEndEffectorPosition = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        self.robotEndEffectorTransform = [se3.identity(),se3.identity()]
        #self.gripperPosition = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
        self.gripperPosition = [0,0]

        self.kin = {}

        # self.kin['left'] = baxter_kinematics('left')
        # self.kin['right'] = baxter_kinematics('right')
        self.last_driveVel = [0.0, 0.0, 0.0]
        self.near_singularity = False
        self.last_sigulatiry = False

        # flag to mark the data
        self.flag  = 0
        # open log file
        timestr = time.strftime("%Y%m%d-%H%M%S")
        self.csvfile_gamepad = open('data/gamepad_log' + timestr + '.csv', 'wb')
        fieldnames = ['rstick', 'lstick', 'LB', 'RB', 'LT', 'RT', 'B', 'A', 'X', 'Y', 'Dpad_x', 'Dpad_y', 
                      'jointAngles', 'gripperStatus', 'eePosition', 'eeTransformation',
                      'baseSensedVelocity', 'baseCommandVelocity', 'time', 'Flag']
        self.gamepad_csv = csv.DictWriter(self.csvfile_gamepad, fieldnames=fieldnames)
        self.gamepad_csv.writeheader()
        self.switch_pub = None
        
        #Publishers for Softhands
        self.pub_r = rospy.Publisher('/left/UbirosGentle', Int8, queue_size = 1)
        self.pub_l = rospy.Publisher('/right/UbirosGentle', Int8, queue_size = 1)
        self.pubPro_l = rospy.Publisher('/left/UbirosGentlePro', Int8, queue_size = 1)
        self.pubPro_r = rospy.Publisher('/right/UbirosGentlePro', Int8, queue_size = 1)
        self.grip_l = 0
        self.grip_r = 0
        self.camera_count = 1

    def init(self,world):
        assert self.j == None,"Init may only be called once"
        self.world = world
        rospy.init_node("gamepad_node")
        # Connect to controller
        return True

    def start(self):
        global gamepad_switch
        try:
        	self.j = gamepad.Gamepad()

    	except:
        	print "Gamepad not found"
        	print "Note: Pygame reports there are " + str(gamepad.Gamepad.numJoys()) + " joysticks"
        	return False
        	
        if self.serviceThread==None:
            self.serviceThread = ServiceThread()
            self.serviceThread.start()
        '''
        if self.gripperController==None:
            self.gripperController = GripperController()
            self.gripperController.start()
        '''
            
        if not self.j: return False
        self.limb = 'left'
        self._status = 'ok'
        self.plugin = MyWidgetPlugin(self)
        self.lastState = {}

        self.switch_pub = rospy.Publisher(gamepad_switch, String, queue_size=1)
        self.camera_switch = rospy.Publisher('switch_GUI', Int8, queue_size=1)
        return True

    def status(self):
        if self.j:
            return 'ok'
        else:
            return 'error'

    def messages(self):
        return ["Controlling "+self.limb]

    def controlMode(self):
        if len(self.lastState)==0: return 'None'
        if self.lastState['RB']: return 'Joint angles'
        elif self.lastState['LB']: return "Cartesian rotation"
        else: return "Cartesian position"

    def stop(self):
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        '''
        if self.gripperController:
            self.gripperController.kill()
            print "gripper control thread killed"
            self.gripperController = None
        '''
        self._status=''
        self.plugin = None
        self.j.quit()

    def get(self):
        j = self.j
        j.updateState()
        state = {}
        state['rstick'] = j.get_stick_R()
        state['lstick'] = j.get_stick_L()
        state['LB'] = j.get_LB()
        state['RB'] = j.get_RB()
        state['LT'] = j.get_LT()
        state['RT'] = j.get_RT()
        state['B'] = j.get_B()
        state['A'] = j.get_A()
        state['X'] = j.get_X()
        state['Y'] = j.get_Y()
        print("LT",state['LT'])
        print("RT",state['RT'])
        Dpad = j.get_Dpad()
        state['Dpad_x'] = Dpad[0]
        # print Dpad[0]
        state['Dpad_y'] = Dpad[1]
        # print Dpad[1]
        # log gamepad 
        #print(state)
        if self.log:
            self.gamepad_csv.writerow(state)

        if len(self.lastState) > 0:
            res = self.do_logic(self.lastState,state)
        else:
            res = None

        self.lastState = state
        return res

    def do_logic(self,lastState,state):
        global GripperMode
        global HoldMode
        global HoldPose
        #global icon_flag

        # get robot state data
        self.getRobotStatus()
        robot_state = {'jointAngles': self.ArmPosition, 'gripperStatus': [self.grip_l,self.grip_r],
                        'eePosition': self.robotEndEffectorPosition, 'eeTransformation': self.robotEndEffectorTransform,
                        'baseSensedVelocity': self.baseSensedVelocity, 'baseCommandVelocity': self.baseCommandVelocity,
                        'time':int(round(time.time() * 1000)), 'Flag': self.flag}
        if self.log:
            self.gamepad_csv.writerow(robot_state)
        rstick = state['rstick']
        lstick = state['lstick']

        if abs(state['Dpad_x']) == 1 and self.lastState['Dpad_x'] == 0:
            print "switching main camera view"
            self.switch_pub.publish("switch")
            self.camera_count += 1
        if abs(state['Dpad_y']) == 1 and self.lastState['Dpad_y'] == 0:
            print "switching secondary camera view"
            self.switch_pub.publish("switch secondary")
            self.camera_count += 1
        self.camera_switch.publish(self.camera_count)
        if state['LT'] > 0.1 and self.lastState['LT'] <= 0.1:
            self.limb = 'left'
        
        if state['B'] == 1 and self.lastState['B'] == 0:
           print "do something cool here"
           
        if state['A'] == 1 and self.lastState['A'] == 0:
            if HoldMode[self.limb] == 'free':
                HoldMode[self.limb] = 'hold'
            elif HoldMode[self.limb] == 'hold':
                HoldMode[self.limb] = 'free'

        if state['Y'] and not self.lastState['Y']:
            if self.controlSet == 'arm':
                self.controlSet = 'base'
                print 'Controlling base now'
            elif self.controlSet == 'base':
                self.controlSet = 'arm'
                print 'Controlling arm now'

        if state['X']:
            TuckStatus[self.limb] = True
            
            #icon_flag = 1
        if not state['X']:
            TuckStatus[self.limb] = False
            #icon_flag = 0
        if TuckStatus[self.limb]:
            Jointmsg = {}
            Jointmsg['type'] = "JointPose"
            Jointmsg['part'] = self.limb
            Jointmsg['position'] = TuckPose[self.limb]
            Jointmsg['speed'] = 1
            Jointmsg['safe'] = 0
            # TuckStatus[self.limb] = False
            # print Jointmsg
            return Jointmsg

        if state['RT'] >= -1.1:
            '''
            gripMsg = {}
            preshape = [1.0]
            if GripperMode[self.limb] == 'power':
                preshape = [1.0]
            elif GripperMode[self.limb] == 'precision':
                preshape = [0.8]

            p = [1.0,1.0, 1.0] + preshape
            gripMsg['limb'] = self.limb
            gripMsg['type'] = 'Gripper'
            gripMsg['force'] = 0.4
            gripMsg['speed'] = 0.2
            if HoldMode[self.limb] == 'free':
                gripsize = (0.2 - 1.8)*state['RT']/2.0 + 1.8/2.0
                print(gripsize)
                p = [gripsize, gripsize, gripsize] + preshape
                HoldPose[self.limb] = p
                gripMsg['position'] = p
            elif HoldMode[self.limb] == 'hold':
                gripMsg['position'] = HoldPose[self.limb]
            self.gripperController.send_pos(gripMsg['position'], gripMsg['limb'])
            return
            # return gripMsg
            '''
            
            if HoldMode[self.limb] == 'free':
                #Calculate the value to close the gripper, shifting the range of the trigger 'RT' from -1 to 1 --> 0 to 2 then mapping to a range of 0 to 100
                gripPercent = (state['RT']+1)*40 # changed it to 0 to 80 since the hardware issue
                #gripPercent = (state['RT']+1)*50
                #update the hold pose
                HoldPose[self.limb] = gripPercent     
            elif HoldMode[self.limb] == 'hold':        
                #use hold percent
                gripPercent = HoldPose[self.limb]
                
            #publish grip command to the correct hand
            if (self.limb == 'right'):
                self.pub_r.publish(gripPercent)
                print(gripPercent)
                self.grip_r = gripPercent
            else:
                print(gripPercent)
                self.pub_l.publish(gripPercent)
                self.grip_l = gripPercent

        tweak = lambda x: 0 if abs(x*self.jointControlRatio) < 0.001 else x*self.jointControlRatio

        # if state['RT'] > 0.1 and lastState['RT'] <= 0.1:
            # return {'type':'Gripper','limb':self.limb,'command':'close'}
        # elif state['RT'] <= 0.1 and lastState['RT'] > 0.1:
            # return {'type':'Gripper','limb':self.limb,'command':'open'}

        if state['LB']:
            # Joint velocity mode
            jVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #lambda for thresholding vel. values

            jVel[6] = tweak(rstick[0])  # wrist 2
            jVel[5] = tweak(rstick[1])  # wrist 1
            jVel[4] = tweak(-lstick[0])  # wrist 1\
            jVel[3] = tweak(lstick[1])  # elbow 1
            if self.limb == 'right':
                jVel[5]*=-1
            return {'type':'JointVelocity','limb':self.limb,'velocity':jVel}

        if state['RB']:  # End effector angular velocity mode
            driveVel = [0.0, 0.0, 0.0]
            driveAngVel = [0.0, 0.0, 0.0]
            driveAngVel[0] = tweak(viewToWorldScaleXY*rstick[0])  # about end effector x
            driveAngVel[1] = tweak(-viewToWorldScaleXY*rstick[1])   # about end effector y
            driveAngVel[2] = tweak(-lstick[0])   # about end effector z
            return {'type':'CartesianVelocity','limb':self.limb,'linear':driveVel,'angular':driveAngVel,'safe':0}

        else:  # End effector velocity mode
            if self.controlSet == 'arm':
                
                    driveVel = [0.0, 0.0, 0.0]
                    driveAngVel = [0.0, 0.0, 0.0]
                    driveVel[1] =tweak(-viewToWorldScaleXY*rstick[0] )  # y
                    driveVel[0] =tweak(-viewToWorldScaleXY*rstick[1] )  # x
                    driveVel[2] =tweak(-lstick[1])  # z
                    self.check_manipulability()
                    if self.near_singularity:
                        if not self.last_sigulatiry:
                            print "recording the current vel"
                            print self.last_driveVel
                            self.last_driveVel = driveVel
                            self.last_sigulatiry = True

                        if self.check_veloctiy_direction(driveVel):
                            #print("dr.MA1")
                            return {'type':'CartesianVelocity','limb':self.limb,'linear':driveVel,'angular':driveAngVel,'safe':0}
                        else:
                            #print("dr.MA2")
                            return {'type':'CartesianVelocity','limb':self.limb,'linear':[0.0, 0.0, 0.0],'angular':driveAngVel,'safe':0}
                    #print("dr.MA3")
                    #return {'type':'CartesianVelocity','limb':self.limb,'linear':driveVel,'angular':driveAngVel,'safe':0}
                    #TODO: integrate relaxed ik to solve for jointPositions variable

                    #rostopic echo /robot/limb/right/joint_command
                    jointPositions = [-0.4224070000000009, -0.5977809999999982, 1.4659799999999963, 1.331100002514084, 0.11256699067231556, 1.461949935679012, -3.130116576124427]

                    return {'type':'PhilipPoseVel','linear':driveVel,'angular':driveAngVel,'safe':0,'position': jointPositions}
                   

            if self.controlSet == 'base':
                print("Base Dr.Ma")
                self.baseCommandVelocity = [-viewToWorldScaleXY*float(lstick[1])/5,-viewToWorldScaleXY*float(lstick[0])/5,-float(rstick[0])/5]
                return {'type':'BaseVelocity',
                        'velocity':self.baseCommandVelocity, 
                        'safe':0}
    
    def check_veloctiy_direction(self, velocity):
        for i in range(3): 
            if self.last_driveVel[i]*velocity[i] < 0:
                return True

    def check_manipulability(self):
        pass
        # # return true if near sigularity
        # global M_limit
        # J = self.kin[self.limb].jacobian()
        # J_trans = self.kin[self.limb].jacobian_transpose()
        # # print J
        # M = np.sqrt(np.linalg.det(np.matmul(J, J_trans)))
        # # print M
        # if M < M_limit:
        #     self.near_singularity = True
        #     self.near_singularity = False  # comment it out later on
        #     self.last_sigulatiry = False
        # else:
        #     self.near_singularity = False
        #     self.last_sigulatiry = False
    
    def getRobotStatus(self):
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        R1,t1=T1
        T2 = self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        R2,t2=T2

        self.baseSensedVelocity = self.serviceThread.baseVelocitySensed.get()
        self.baseCommandVelocity = self.serviceThread.baseVelocityCommand.get()

        self.ArmPosition[0] = self.serviceThread.ArmPosition_left.get()
        self.ArmPosition[1] = self.serviceThread.ArmPosition_right.get()

        self.robotEndEffectorPosition[0] = t1
        self.robotEndEffectorPosition[1] = t2

        self.robotEndEffectorTransform[0] = T2
        self.robotEndEffectorTransform[1] = T2

        self.gripperPosition[0] = self.serviceThread.gripperGetter_left.get()
        self.gripperPosition[1] = self.serviceThread.gripperGetter_right.get()

    def glPlugin(self):
        return self.plugin
 
class ServiceThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.stateUpdateFreq = 50

        self.state_listener = MultiTopicListener(system_state_addr,topics=None,rate=self.stateUpdateFreq)
        self.baseVelocitySensed = self.state_listener.listen('.robot.sensed.base.velocity')
        self.baseVelocityCommand = self.state_listener.listen('.robot.command.base.velocity')
        self.ArmPosition_left = self.state_listener.listen('.robot.sensed.left.q')
        self.ArmPosition_right = self.state_listener.listen('.robot.sensed.right.q')
        self.eeGetter_left = self.state_listener.listen('.robot.endEffectors.0.xform.destination')
        self.eeGetter_right = self.state_listener.listen('.robot.endEffectors.1.xform.destination')
        self.gripperGetter_left = self.state_listener.listen('.robot.gripper.left.positionCommand')
        self.gripperGetter_right = self.state_listener.listen('.robot.gripper.right.positionCommand')
        self.state_listener.setName("GamepadGUIListener")

    def run(self):
        """Note: don't call ServiceThread.run(), call ServiceThread.start()"""
        #self.hapticupdater.run(1)
        while not self._kill:
            #listen to state topics
            asyncore.loop(timeout = 1.0/self.stateUpdateFreq, count=10)

    def kill(self):
        self._kill = True

def make():
    return GamePadTaskGenerator()

