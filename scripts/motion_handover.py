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

        # kinematic parameters
        self.time = np.repeat(time.time(), 3, axis=0)
        self.delta_T = np.repeat(time.time(), 3, axis=0)

        self.left_hand_position = np.zeros((3,3), dtype=np.double)
        self.right_hand_position = np.zeros((3,3), dtype=np.double)

        self.left_joint_angle = np.zeros((3,7), dtype=np.double)
        self.right_joint_angle = np.zeros((3,7), dtype=np.double)

        # marker position in kinect2_link frame
        self.marker_pos_kinect = [0.0, 0.0, 0.0]
        self.marker_pos_klampt = [0.0, 0.0, 0.0]

        # Marker position in Klampt frame
        self.mark_pos = np.zeros((3,3), dtype=np.double)
        self.mark_vel = np.zeros((3,3), dtype=np.double)
        self.mark_acc = np.zeros((3,3), dtype=np.double)
        self.mark_time = np.repeat(time.time(), 3, axis=0)
        self.mark_delta_T = np.repeat(time.time(), 3, axis=0)

        # Tracking parameters:
        self.track_dist = [0.0, 0.0, 0.0]
        self.track_ratio = 0.1
        self.track_cmd_vel = [0.0, 0.0, 0.0]
        self.track_cmd_ang = [0.0, 0.0, 0.0]

        self.ang_vel = [0.0, 0.0, 0.0] 
        self.pos_vel = [0.0, 0.0, 0.0] 

        # Move-to predicted position
        self.move2Goal_R = so3.identity()
        self.move2Goal_t = [0.0, 0.0, 0.0]
        self.target_moveto_R = so3.identity()
        self.target_moveto_t = [0.0, 0.0, 0.0]

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
            self.time[0] = time.time()

            self.target_moveto_R = robot.right_ee.commandedTransform()[0]
            self.target_moveto_t = robot.right_ee.commandedTransform()[1]

            self.move2Goal_R = self.target_moveto_R
            self.move2Goal_t = self.target_moveto_t

            self.target_moveto_t[1] = self.target_moveto_t[1] + 0.5

        else:
            # control robot motion - task space control - look into motion lib
            # robot.left_ee.velocityCommand([0,0,0],[math.sin(t)*0.2,math.cos(t)*0.2,0])

            # update robot states: 
            for i in range(2):
                self.left_hand_position[i] = self.left_hand_position[i+1]
                self.left_joint_angle[i] = self.left_joint_angle[i+1]

                self.right_hand_position[i] = self.right_hand_position[i+1]
                self.right_joint_angle[i] = self.right_joint_angle[i+1]
                
                self.time[i] = self.time[i+1]

            self.left_hand_position[2] = robot.left_ee.commandedTransform()[1]
            self.left_joint_angle[2] = robot.left_limb.sensedPosition()

            self.right_hand_position[2] = robot.right_ee.commandedTransform()[1]
            self.right_joint_angle[2] = robot.right_limb.sensedPosition()

            self.time[2] = time.time()


            # Start to move target (right robot arm)
            if self.goCommand is True:
                self.pos_vel = [0.0, math.sin(t)*0.2, 0.0] 
            else:
                self.pos_vel = [0.0, 0.0, 0.0] 
            
            robot.right_ee.velocityCommand(self.ang_vel, self.pos_vel)

            # Start to track target (left robot arm)
            if self.trackCommand is True:
                robot.left_ee.velocityCommand(self.track_cmd_ang, self.track_cmd_vel)

            # Start to move to predicted position
            if self.move2Command is True:
                # robot.left_ee.moveTo(self.move2Goal_R, self.move2Goal_t)
                robot.left_ee.moveTo(self.target_moveto_R, self.target_moveto_t)

        return

    def callback_marker(self, data):
        marker_received = data.pose.position
        self.marker_pos_kinect = [marker_received.x, marker_received.y, marker_received.z]
        # print "received marker: ", self.tracked_pos

        (trans,rot) = self.listener.lookupTransform('/pedestal', '/marker_pos', rospy.Time(0))
        # print "[Marker position] ", trans

        # Hand position after correction
        self.marker_pos_klampt[0] = trans[0];
        self.marker_pos_klampt[1] = trans[1];
        self.marker_pos_klampt[2] = trans[2] + 1.0;
        # print "[ Time", time.time() , "] ","Marker position after correction: ", self.marker_pos_klampt

        self.update_marker_hist()
        self.commandTracking()

    def update_marker_hist(self):
        # update marker history. Most recent data: idx = 2
        for i in range(2):
            self.mark_pos[i] = self.mark_pos[i+1]
            self.mark_vel[i] = self.mark_vel[i+1]
            self.mark_acc[i] = self.mark_acc[i+1]
            self.mark_time[i] = self.mark_time[i+1]
            self.mark_delta_T[i] = self.mark_delta_T[i+1]

        self.mark_pos[2] = self.marker_pos_klampt
        self.mark_time[2] = time.time()

        self.mark_delta_T[2] = self.mark_time[2] - self.mark_time[1]
        self.mark_vel[2] = (self.mark_pos[2] - self.mark_pos[1])/self.mark_delta_T[2]
        self.mark_acc[2] = (self.mark_vel[2] - self.mark_vel[1])/self.mark_delta_T[2]

    def commandTracking(self):
        self.track_dist = self.left_hand_position[2] - self.mark_pos[2]
        for i in range(0,3):
            self.track_cmd_vel[i] = -self.track_ratio * self.track_dist[i]

    def commandPredictedGoal(self):
        # Computer min jerk prediction
        pass


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
            print "m is pressed. Starts to track target arm. "
            if self.move2Command is False:
                self.move2Command = True

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
