#!/usr/bin/env python

import sys
lib_path = "/home/motion/iml-internal/Ebolabot/"
# model_path = "/home/motion/iml-internal/Ebolabot/klampt_models"
sys.path.append(lib_path)
# sys.path.append(model_path)

import roslib, rospy
from Motion import motion
from Motion import config, config_physical
from klampt import *
from klampt.glprogram import *
from visualization_msgs.msg import Marker 
import math
import time
import os
import tf
import numpy as np


Moment_pedestal2kinect = (0.423455, 0.424871, 0.5653, 0.566222)
Trans_pedestal2kinect = (0.228665, 0.0591513, 0.0977748)
Rot_pedestal2kinect = so3.from_moment(Moment_pedestal2kinect)
Transform = (Rot_pedestal2kinect, Trans_pedestal2kinect)
Transform_inv = se3.inv(Transform)
dim_str = ["x", "y", "z"]

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        self.t0 = time.time()
        self.velocity_scale = 2.0
        self.inPosition = False
        self.goCommand = False
        self.trackCommand = False
        self.move2Command = False 
        self.markerMotionStart = False
        self.lookatMarker = False
        self.counter = 0

        self.takeCurrentPosition = False

        # kinematic parameters
        self.time = np.repeat(0.0, 3, axis=0)
        self.delta_T = np.repeat(0.0, 3, axis=0)

        self.left_hand_position = np.zeros((3,3), dtype=np.double)
        self.right_hand_position = np.zeros((3,3), dtype=np.double)

        self.left_joint_angle = np.zeros((3,7), dtype=np.double)
        self.right_joint_angle = np.zeros((3,7), dtype=np.double)

        # marker position in kinect2_link frame
        self.marker_pos_kinect = np.array([0.0]*3)
        self.marker_pos_klampt = np.array([0.0]*3)

        # Marker position in Klampt frame
        self.mark_pos = np.zeros((3,3), dtype=np.double)
        self.mark_vel = np.zeros((3,3), dtype=np.double)
        self.mark_acc = np.zeros((3,3), dtype=np.double)
        self.mark_path_pos = np.array([0.0]*3)
        self.mark_path_vel = np.array([0.0]*3)
        self.mark_path_acc = np.array([0.0]*3)

        self.mark_time = np.repeat(time.time(), 3, axis=0)
        self.mark_delta_T = np.repeat(time.time(), 3, axis=0)

        # Tracking parameters:
        self.track_dist = np.array([0.0]*3)
        self.track_ratio = 0.1
        self.track_cmd_vel = np.array([0.0]*3)
        self.track_cmd_ang = np.array([0.0]*3)

        self.ang_vel = np.array([0.0]*3)
        self.pos_vel = np.array([0.0]*3)

        # Min jerk model predicted position
        self.move2Goal_pos = np.array([0.0]*3)
        self.target_initial_pos = np.array([0.0]*3)
        self.target_moveto_pos = np.array([0.0]*3)
        self.moveto_cmd_vel = np.array([0.0]*3)
        self.moveto_cmd_ang = np.array([0.0]*3)
        self.time_motionstart = np.array([0.0]*3)
        self.pos_motionstart = np.array([0.0]*3)
        self.pos_predicted = np.array([0.0]*3)
        self.time_predicted = np.array([0.0]*3)


        self.min_motion_vel = 0.0
        self.max_motion_vel = 3.0

        self.min_motion_vel_dim = 0.0
        self.max_motion_vel_dim = 3.0

        # ROS communication:
        rospy.init_node('listener', anonymous=True)
        self.listener = tf.TransformListener()
        self.sub_marker = rospy.Subscriber("Marker_glove", Marker, self.callback_marker, queue_size=1)
        
    def display(self):
        # Put your display handler here
        # the current example draws the sensed robot in grey and the
        # commanded configurations in transparent green
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

        # draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)

    def control_loop(self):
        robot = motion.robot
        # Put your control handler here
        # currently moves a joint according to sinusoid
        
        t = time.time()-self.t0

        # q = robot.left_limb.commandedPosition()
        # print "Left commanded",q
        # q[1] = math.sin(time.time() )
        # robot.left_limb.positionCommand(q)
        if self.takeCurrentPosition is True:
            self.target_initial_pos[:] = robot.right_ee.sensedTransform()[1]
            self.target_moveto_pos[:] = robot.right_ee.sensedTransform()[1]
            self.target_moveto_pos[1] = self.target_moveto_pos[1] - 0.3
            self.takeCurrentPosition = False

        if robot.right_mq.moving() or robot.left_mq.moving():
            pass
        elif not self.inPosition:
            q_left = robot.left_limb.commandedPosition()
            q_right = robot.right_limb.commandedPosition()

            # Tuck arm position
            # q_left = [-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.9132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
            # q_right = [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.9132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]

            q_left = [-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 0.5, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
            q_right = [0.28897088559570313, -0.9675583808532715, 1.2034079267211915, 1.5132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]
            
            print "Sent setRamp command"
            robot.left_mq.setRamp(q_left)
            robot.right_mq.setRamp(q_right)

            self.inPosition = True
            print "self.inPosition = ", self.inPosition

            # take a snap shot of current position, set a goal to move to

            self.move2Goal_pos[:] = robot.left_ee.sensedTransform()[1]

        else:
            # control robot motion - task space control - look into motion lib
            # robot.left_ee.velocityCommand([0,0,0],[math.sin(t)*0.2,math.cos(t)*0.2,0])

            # update robot states: 
            for i in range(2):
                self.left_hand_position[i][:] = self.left_hand_position[i+1]
                self.left_joint_angle[i][:] = self.left_joint_angle[i+1]

                self.right_hand_position[i][:] = self.right_hand_position[i+1]
                self.right_joint_angle[i][:] = self.right_joint_angle[i+1]
                
                self.time[i] = self.time[i+1]

            self.left_hand_position[2][:] = robot.left_ee.sensedTransform()[1]
            self.left_joint_angle[2][:] = robot.left_limb.sensedPosition()

            self.right_hand_position[2][:] = robot.right_ee.sensedTransform()[1]
            self.right_joint_angle[2][:] = robot.right_limb.sensedPosition()

            # print "self.right_hand_position[2]", self.right_hand_position[2]
            # print "self.target_moveto_pos", self.target_moveto_pos

            # Start to move target (right robot arm)
            if self.goCommand is True:
                vel_ratio = [0.0, 0.1, 0.0]
                command_vel = self.moveTo(vel_ratio, self.right_hand_position[2], self.target_moveto_pos)
                if self.if_reach_target(self.right_hand_position[2], self.target_moveto_pos, 0.01):
                    # print "reached"
                    self.pos_vel = [0.0, 0.0, 0.0]
                    self.goCommand = False
                else:
                    # print "not reached ... "
                    self.pos_vel = command_vel
                    
            
            robot.right_ee.velocityCommand(self.ang_vel, self.pos_vel)

            # robot.right_ee.driveCommand([0,0,0],[math.sin(t)*0.1,math.cos(t)*0.1,0])
            # robot.right_ee.driveCommand(self.ang_vel, self.pos_vel)

            # Start to track target (left robot arm)
            if self.trackCommand is True:
                robot.left_ee.velocityCommand(self.track_cmd_ang, self.track_cmd_vel)

            # Start to move to predicted position
            if self.move2Command is True:
                robot.left_ee.velocityCommand(self.moveto_cmd_ang, self.moveto_cmd_vel)
                # robot.left_ee.moveTo(self.move2Goal_R, self.move2Goal_t)
                                
        return

    def callback_marker(self, data):
        # print "time = " ,self.mark_time[2]
        marker_received = data.pose.position

        # timestamp = data.header.stamp.secs + data.header.stamp.nsecs * 10e-9
        timestamp = data.header.stamp.to_time()
        # print "timestamp.sec ", data.header.stamp.secs, "timestampe.nsec ", data.header.stamp.nsecs
        self.marker_pos_kinect = [marker_received.x, marker_received.y, marker_received.z]
        # print "received marker: ", self.tracked_pos

        (trans,rot) = self.listener.lookupTransform('/pedestal', '/marker_pos', rospy.Time(0))
        # print "[Marker position] ", trans

        # Hand position after correction
        self.marker_pos_klampt[0] = trans[0];
        self.marker_pos_klampt[1] = trans[1];
        self.marker_pos_klampt[2] = trans[2] + 1.0;
        # print "[ Time", time.time() , "] ","Marker position after correction: ", self.marker_pos_klampt
        if timestamp > self.mark_time[2]:
            self.update_marker_hist(timestamp)
            # print "time = ", timestamp
        else:
            print "skip message ... ", timestamp
            

        if self.trackCommand is True:
            self.commandTracking()

        if self.markerMotionStart is True:
            self.computePredictedGoal()

    def update_marker_hist(self, timestamp):
        # update marker history. Most recent data: idx = 2
        for i in range(2):
            self.mark_pos[i][:] = self.mark_pos[i+1]
            self.mark_vel[i][:] = self.mark_vel[i+1]
            self.mark_acc[i][:] = self.mark_acc[i+1]
            self.mark_path_pos[i] = self.mark_path_pos[i+1]
            self.mark_path_vel[i] = self.mark_path_vel[i+1]
            self.mark_path_acc[i] = self.mark_path_acc[i+1]

            self.mark_time[i] = self.mark_time[i+1]
            self.mark_delta_T[i] = self.mark_delta_T[i+1]

        self.mark_time[2] = timestamp

        self.mark_pos[2][:] = self.marker_pos_klampt
        self.mark_delta_T[2] = self.mark_time[2] - self.mark_time[1]

        self.mark_vel[2] = (self.mark_pos[2] - self.mark_pos[1])/self.mark_delta_T[2]
        self.mark_acc[2] = (self.mark_vel[2] - self.mark_vel[1])/self.mark_delta_T[2]
        # print "marker_delta_T = ", self.mark_delta_T

        self.mark_path_pos[2] = np.sqrt(np.dot(self.mark_pos[2], self.mark_pos[2]))
        self.mark_path_vel[2] = np.sqrt(np.dot(self.mark_vel[2], self.mark_vel[2]))
        self.mark_path_acc[2] = np.sqrt(np.dot(self.mark_acc[2], self.mark_acc[2]))

        if self.lookatMarker is True:
            # print "marker velocity: ", self.mark_path_vel[2]
            if abs(self.mark_path_vel[2])>self.min_motion_vel and abs(self.mark_path_vel[2]) < self.max_motion_vel:
                if self.markerMotionStart is False:
                    print "start motion"
                    self.markerMotionStart = True
                    self.time_motionstart = time.time()
                    self.pos_motionstart[:] = self.mark_pos[2]



    def commandTracking(self):
        self.track_dist = self.left_hand_position[2] - self.mark_pos[2]
        for i in range(0,3):
            self.track_cmd_vel[i] = -self.track_ratio * self.track_dist[i]

    def computePredictedGoal(self):
        # Computer min jerk prediction
        if self.markerMotionStart is True:
            # print "Start min jerk model prediction ... "
            tc = self.mark_time[2]
            t0 = self.time_motionstart

            print "\rvelocity:", "%+0.2f" % self.mark_vel[2][0], ",", "%+0.2f" % self.mark_vel[2][1], ",", "%+0.2f" % self.mark_vel[2][2],

            for i in range(0,3):
                x0 = 0.0
                xc = self.mark_pos[2][i] - self.pos_motionstart[i]
                vc = abs(self.mark_vel[2][i])
                vc_sign = np.sign(vc)



                if abs(vc) > self.min_motion_vel_dim and abs(vc) < self.max_motion_vel_dim:
                    # compute common terms
                    # print "motion exists in ", dim_str[i], "dimension"

                    z0 = vc - 2.0*x0 + 2.0*xc;
                    x0p2 = x0*x0
                    x0p3 = x0p2*x0
                    xcp2 = xc*xc
                    xcp3 = xcp2*xc

                    vcp2 = vc*vc
                    vcp3 = vcp2*vc
                    vcp4 = vcp2*vcp2
                    vcp5 = vcp2*vcp3
                    vcp6 = vcp2*vcp4

                    d11 = 1350.0*vcp3 + 4860.0*vcp2*x0 + 16200.0*vc*x0p2 + 54000.0*x0p3 -  4860.0*vcp2*xc - 32400.0*vc*x0*xc - 162000.0*x0p2*xc + 16200.0*vc*xcp2 +  162000.0*x0*xcp2 - 54000.0*xcp3
                    d12 = 1458000.0*vcp6 + 8748000.0*vcp5*x0 + 27993600.0*vcp4*x0p2 + 104976000.0*vcp3*x0p3 - 8748000.0*vcp5*xc - 55987200.0*vcp4*x0*xc - 314928000.0*vcp3*x0p2*xc + 27993600.0*vcp4*xcp2 + 314928000.0*vcp3*x0*xcp2 - 104976000*vcp3*xcp3
                    tmp = d11+np.sqrt(d12)
                    d = np.power(tmp, 1.0/3.0);
                    
                    a1 = 5.0*z0
                    a2 = 6.0*vc
                    a = a1/a2

                    b1 = -225.0 * z0*z0 + 180.0*vc*(vc - 6.0*x0 + 6.0*xc);
                    b2 = 9.0*(np.power(2.0, 2.0/3.0))*vc; 
                    b = b1/(b2*d);

                    c1 = -18.0*np.power(2.0,1.0/3.0)*vc;
                    c = d/c1;

                    ratio_t = a + b + c;

                    tf = t0 + (tc-t0)/ratio_t;                
                    xf = x0 + (xc-x0)/(10.0*np.power(ratio_t,3)-15.0*np.power(ratio_t,4) + 6.0*np.power(ratio_t,5));

                    self.time_predicted[i] = tf
                    self.pos_predicted[i] = xf

                    print "predicted position", self.pos_predicted
                    print "vc in ", "xyz"[i], ":", vc
                    # print "predicted time", self.time_predicted


                else:
                    # print "no motion in ", dim_str[i], "predicted position is where the human hand is. "
                    # self.pos_predicted[i] = self.mark_pos[2][i]
                    # self.time_predicted[i] = time.time()
                    pass


    def if_reach_target(self, pos, target, error):
        dist_to_target = np.linalg.norm(pos-target)
        if dist_to_target <= error:
            return True
        else:
            return False
    def moveTo(self, vel_ratio, pos, target):
        vel_direction = np.sign(target-pos)
        vel = vel_direction * vel_ratio
        return vel

    def idle(self):
        self.control_loop()
        glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        # Put your mouse handler here
        print "mouse",button,state,x,y
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        # Put your mouse motion handler here
        GLRealtimeProgram.motionfunc(self,x,y)

    def specialfunc(self,c,x,y):
        # Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        # Put your keyboard handler here
        # the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 'q':
            motion.robot.shutdown()
            exit(0)
        if c == 'g':
            print "g is pressed. Start target arm "
            if self.goCommand is False:
                self.goCommand = True

        if c == 's':
            print "s is pressed. Stop target arm"
            if self.goCommand is True:
                self.goCommand = False
        if c == 't':
            print "t is pressed. Starts to track target arm. "
            if self.trackCommand is False:
                self.trackCommand = True
        if c == 'm':
            print "m is pressed. Starts to move to predicted position by min-jerk-model. "
            if self.move2Command is False:
                self.move2Command = True
        if c == 'l':
            print "l is pressed. Starts to look at marker. "
            if self.lookatMarker is False:
                self.lookatMarker = True
        if c == 'i':
            print "i is pressed. Take current position as start point. "
            if self.takeCurrentPosition is False:
                self.takeCurrentPosition = True
        

        glutPostRedisplay()


if __name__ == "__main__":
    config_physical.parse_args()
    print "motion_template.py: sets up a visualizer for testing Motion"
    print "control loops.  Press q to exit."
    print

    print "Loading APC Motion model", config_physical.klampt_model
    print "config.mode =", config_physical.mode
    robot = motion.setup(mode=config_physical.mode, klampt_model=lib_path+config_physical.klampt_model, libpath=lib_path)

    res = robot.startup()
    if not res:
        print "Error starting up APC Motion"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(lib_path+config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load APC Motion model ")
    
    viewer = MyGLViewer(world)
    viewer.run()
