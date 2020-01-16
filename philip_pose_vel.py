from task_controller import TaskController
from motion import robot
# sys.path.append('/home/motion/iml-internal/Ebolabot/Controller/Tasks/SafeBaseControl')
import time, sys, threading
from SafeBaseControl.safe_base_control import SafeBaseControl

class PhilipPoseVel(TaskController):
    """Task: CartesianVelocity
    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - linear: velocity in torso space, array of size 3 (single arm) or
      6 (dual arm)
    - angular: angular velocity in torso space, array of size 3 (single arm)
      or 6 (dual arm)
    - toolCenterPoint*: position in the local frame of the end effector (wrist) at 
      which the desired velocity is considered to be applied.
    - maxTime*: a maximum amount of time to apply the velocity, default 1.
    - safe*: either 1 (true) or 0 (false).  If true, performs planning
      to avoid obstacles.  Otherwise, does no planning.  Default false.
    """
    def __init__(self):
        TaskController.__init__(self)

        #--- Safe Control ---
        self.sbc = SafeBaseControl()
        self.sbc.set_previous_velocity((0,0,0))

        # enable safe base control thread
        self.control_factor_thread = threading.Thread(target=self.sbc.update_control_factor)
        self.control_factor_thread.setDaemon(True)
        self.control_factor_thread.start()

        self.message("Safe base control initialized")

        self.safe = False # toggle starting value
        self.safe_prev = 0
        #--- End Safe Control ---

    def taskName(self):
        return 'PhilipPoseVel'

    def start(self,args):
        # Error checking the input arguments from the dictionary
        print "Calling PhilipPoseVel start",args
        """if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False"""
        if 'linear' not in args and 'angular' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'angular' not in args:
            self.onInvalidParameters("invalid argument list, must have velocity specified")
            return False
        if 'position' not in args:
            self.onInvalidParameters("invalid argument list, must have joint position specified")
            return False
        if len(args['linear']) != 3:
            self.onInvalidParameters("invalid argument list, velocity command is not length 3")
            return False
        if len(args['position']) != 7:
            self.onInvalidParameters("invalid argument list, joint position command is not length 7")
            return False
                        
        # Now I extract the arguments from the pose/vel command

        #--- Safe Control ---
        safe = args.get('safe',False)
        if safe:
            print 'safe'
            robot.enableCollisionChecking(True)
            robot.left_limb.enableSelfCollisionAvoidance(False)
            #robot.right_limb.enableSelfCollisionAvoidance(False)
        else:
            robot.enableCollisionChecking(False)
            robot.left_limb.enableSelfCollisionAvoidance(True)
            #robot.right_limb.enableSelfCollisionAvoidance(True)
        #--- End Safe Control ---
        
        # We do not actually care about which limb -- the left is pose control and the right one is cartesian control
        #limb = args['limb']
        # velocity
        dlinear = args['linear']
        dangular = args['angular']
        toolCenterPoint = args.get('toolCenterPoint',None)
        # position
        angles = args['position']

        # Time
        maxTime = args.get('maxTime',1.0)
        if maxTime < 0:
            self.onInvalidParameters("maxTime argument is negative")
            return False
        
        # Now I set the Zero configuration for velocity control
        if len(dlinear) > 0 and len(dlinear) !=3:
            self.onInvalidParameters("invalid 'linear' argument for linear velocity, not size 3")
            return False
        if len(dangular) > 0 and len(dangular)!=3:
            self.onInvalidParameters("invalid 'angular' argument for angular velocity, not size 3")
            return False
        if len(dlinear) == 3 and len(dangular) == 0:
            dangular = [0.0, 0.0, 0.0]
        if len(dangular) == 3 and len(dlinear) == 0:
            dlinear = [0.0, 0.0, 0.0]

        #deadband
        for i,v in enumerate(dlinear):
            if abs(v) < 1e-4: dlinear[i] = 0
        for i,v in enumerate(dangular):
            if abs(v) < 1e-4: dangular[i] = 0
        if safe:
            robot.enableCollisionChecking(True)
        else:
            robot.enableCollisionChecking(False)

        # Command the left  arm to a cartesian velocity
        #robot.left_ee.moveTo(rotation, position, maxJointDeviation=maxJointDeviation)
        robot.left_ee.driveCommand(dangular, dlinear)    
        # Command the right arm to a joint pose
        #robot.right_ee.driveCommand(dangular, dlinear)
        robot.right_mq.setRamp(angles) 

        self.mqs = [robot.left_mq,robot.right_mq]

        self.endTime = time.time() + maxTime
        if max(abs(v) for v in dangular+dlinear) == 0:
            self._status = 'done'
        else:
            self._status = 'ok'
        return True
    
    def status(self):
        if self._status == 'ok':
            #check if the motion queue is moving
            if not any(mq.moving() for mq in self.mqs):
                self.onDone()
        return self._status
        
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()
        
        self._status = ''
        robot.enableCollisionChecking(False)
        return None

    def close(self):
        """Called by dispatcher to tell the controller the program is ending,
        and to do any final cleanup."""

def make():
    return PhilipPoseVel()


"""

Calling PhilipPoseVel start {'linear': [0, 0, 0], 'task_request_time': 24.563, 'safe': 0, 'position': [0.740906, 0.0543964, -0.292469], 'rotation': [0.311, 0.808, -0.407, 0.29], 'type': 'PacPoseVel', 'angular': [0, 0, 0]}
PhilipPoseVel: invalid 'rotation' argument, not size 3
Task PhilipPoseVel controller returned false on Start()...
Starting TASK {
  "position": [0.740906, 0.0543964, -0.292469], 
  "type": "PhilipPoseVel", 
  "rotation": [0.311, 0.808, -0.407, 0.29], 
  "task_request_time": 24.563, 
  "angular": [0, 0, 0], 
  "safe": 0, 
  "linear": [0, 0, 0]
}



"""

