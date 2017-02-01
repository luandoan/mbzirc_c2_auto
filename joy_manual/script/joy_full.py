#! /usr/bin/env python
import rospy, sys, roslib, rosparam, actionlib, time
roslib.load_manifest("rosparam")
import moveit_commander
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.msg import RobotState
from actionlib_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
from std_msgs.msg import Int8

class joy_ur5():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('joy_ur5', anonymous=False)
	
	self.min_pos = -6.283
	self.max_pos = 6.283
	self.step = 0.1
	self.max_acc = 3.0
    	self.last_pan = 0.0
	self.last_lift = 0.0
	self.last_elbow = 0.0
	self.last_wrist1 = 0.0
	self.last_wrist2 = 0.0
	self.last_wrist3 = 0.0

	# Subscribe to Joy and Publish to Base and Arm
	rospy.Subscriber("/joy", Joy, self.callback)
	rospy.Subscriber("/joint_states", JointState, self.joint_state)
	self.base_pub = rospy.Publisher('/joy_teleop/cmd_vel', Twist, queue_size=5)
	self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=5)
	self.grip_gazebo_pub = rospy.Publisher('/gripper/cmd_vel', Twist, queue_size = 5)
	self.grip_physical_pub = rospy.Publisher('/gripper_req', Int8, queue_size=5)
	#rospy.Subscriber("/move_group/status", GoalStatusArray, self.status, queue_size=1)

	
		
    def callback(self, joy):
	# control signal
	ready = joy.buttons[4]*joy.buttons[7]
	back = joy.buttons[4]*joy.buttons[6]
	if ready != 0:
	    rospy.set_param('smach_state', 'Starting manual control')
	    print "Manual control node is ready"
	elif back != 0:
	    rospy.set_param('smach_state','Quit manual control node, back to autonomous')
	    print "Quit manual node, back to autonomous"

	# base control - press and hold LB and stick left
	#rospy.loginfo("Press and hold LB + stick left to move robot ... ")
	twist = Twist()
	cl = joy.buttons[4]*joy.axes[1]	# control signal - hold LB
	cr = joy.buttons[4]*joy.axes[0]
	twist.linear.x = 4*joy.buttons[4]*joy.axes[1]	# move stick left up-down
	twist.angular.z = 2*joy.buttons[4]*joy.axes[0]	# move stick left left-right
	
	
	# Get joint velocity from Joystick
	l_up_vel = 3.15*joy.buttons[5]*joy.buttons[3]*joy.axes[7]	# press and hold RB + crosskey up-down
	l_pan_vel = 3.15*joy.buttons[5]*joy.buttons[3]*joy.axes[6]	# press and hold RB + crosskey left-right
	l_push_vel = 3.15*joy.buttons[4]*joy.buttons[5]*joy.axes[7]	# press and hold RB + leftstick
	#print "Linear moving velocity: ", l_up_vel, l_pan_vel, l_push_vel

	pan_vel = 3.15*joy.buttons[5]*joy.buttons[2]*joy.axes[6]	# press and hold X + crosskey up-down for shoulder pan
	#print vel_pan
	lift_vel = 3.15*joy.buttons[5]*joy.buttons[2]*joy.axes[7]  # press and hold X + crosskey left-right for shoulder lift
	#print vel_lift
	elbow_vel = 3.15*joy.buttons[5]*joy.buttons[0]*joy.axes[6]	# press and hold A + crosskey up-down for elbow
	#print vel_elbow
	wrist1_vel = 3.15*joy.buttons[5]*joy.buttons[0]*joy.axes[7]	# press and hold A + crosskey left-right for wrist 1
	#print vel_wrist1
	wrist2_vel = 3.15*joy.buttons[5]*joy.buttons[1]*joy.axes[6]	# press and hold B + crosskey up-down for wrist 2
	#print vel_wrist2
	wrist3_vel = 3.15*joy.buttons[5]*joy.buttons[1]*joy.axes[7]	# press and hold B + crosskey left-right for wrist 3
	#print vel_wrist3
	
	# Gripper control:
	g_control = joy.buttons[4]*joy.buttons[5]*joy.axes[6] 
	#print "Gripper test: ", g_control

	if (cl != 0):
	    active_b = True
	    active = False
	    active_l = False
	    active_g = False
	elif (cr != 0):
	    active_b = True
	    active = False
	    active_l = False
	    active_g = False	
	elif (pan_vel != 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (lift_vel != 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (elbow_vel <> 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (wrist1_vel <> 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (wrist2_vel <> 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (wrist3_vel <> 0):
	    active_b = False
	    active = True
	    active_l = False
	    active_g = False
	elif (l_up_vel !=0):
	    active_b = False
	    active_l = True
	    active = False
	    active_g = False
	elif (l_pan_vel !=0):
	    active_b = False
	    active_l = True
	    active = False
	    active_g = False
	elif (l_push_vel !=0):
	    active_b = False
	    active_l = True
	    active = False
	    active_g = False
	elif (g_control != 0):
	    active_b = False
	    active_l = False
	    active = False
	    active_g = True	
	else:
	    active_b = False
	    active = False
	    active_l = False
	    active_g = False
	
	# Moving base:
	if (active_b):
	    self.base_pub.publish(twist)
	    print "Moving base: ", joy.buttons[4], 20*joy.buttons[4]*joy.axes[1], 10*joy.buttons[4]*joy.axes[0]
	    #rospy.loginfo("Moving robot joystick ...")
	    rospy.set_param('smach_state','Manual moving base')	 
	

	# Processing on arm
	arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

	# Load topic from param
	shoulder_pan_joint = 'ur5_arm_shoulder_pan_joint'
	shoulder_lift_joint = 'ur5_arm_shoulder_lift_joint'
	elbow_joint = 'ur5_arm_elbow_joint'
	wrist1_joint = 'ur5_arm_wrist_1_joint'
	wrist2_joint = 'ur5_arm_wrist_2_joint'
	wrist3_joint = 'ur5_arm_wrist_3_joint'


	# Moving arm linear step by step
	if (active_l):
	    # Initialize the move_group API
            moveit_commander.roscpp_initialize(sys.argv)
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
	    ## Linear moving arm 	
	    rospy.set_param('arm_prefix', 'ur5_arm_')
    	    rospy.set_param('reference_frame', '/base_link')
    	    self.arm = moveit_commander.MoveGroupCommander("ur5_arm")
	    end_effector_link = self.arm.get_end_effector_link()
    	    #rospy.loginfo(end_effector_link)

	    reference_frame = rospy.get_param("~reference_frame", "/base_link")

	    # Initialize the move group for the ur5_arm
	    self.arm.set_pose_reference_frame(reference_frame)
	    self.arm.allow_replanning(True)

	    # Allow some leeway in position (meters) and orientation (radians)
    	    self.arm.set_goal_position_tolerance(0.01)
    	    self.arm.set_goal_orientation_tolerance(0.01)
	 
 	    ## Linear moving
	    # Get current joint position to use for planning
            try:
            	cjs = rospy.get_param('current_joint_state')
            except:
            	cjs = [0,0,0,0,0,0]

	    jt = RobotState()
            jt.joint_state.header.frame_id = '/base_link'
            jt.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel', 'ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint', 'left_tip_hinge', 'right_tip_hinge']
            jt.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],cjs[5],0,0]

            #crs = robot.get_current_state()

            # Set the start state to the current state
            # self.arm.set_start_state(jt)
	    # Set the start state to the current state
	    self.arm.set_start_state_to_current_state()
	    	    
	    if (l_up_vel > 0):
		self.arm.shift_pose_target(2,0.005, end_effector_link)	
		print "Linear going up ...", l_up_vel
	    elif (l_up_vel < 0):
		self.arm.shift_pose_target(2,-0.005, end_effector_link)
		print "Linear going down ...", l_up_vel
	    elif (l_pan_vel > 0):
		self.arm.shift_pose_target(1, 0.005, end_effector_link)
		print "Linear moving right ...", l_pan_vel
 	    elif (l_pan_vel < 0):
		self.arm.shift_pose_target(1,-0.005, end_effector_link)
		print "Linear moving left ...", l_pan_vel
	    elif (l_push_vel > 0):
		self.arm.shift_pose_target(0, 0.005, end_effector_link)
		print "Linear moving forward ...", l_push_vel
	    elif (l_push_vel < 0):
		self.arm.shift_pose_target(0,-0.005, end_effector_link)
		print "Linear moving backward ...", l_push_vel
	
	    traj = self.arm.plan()
	    traj_pts = len(traj.joint_trajectory.points)
	    
	    # update current joint states
	    rospy.set_param('current_joint_state', traj.joint_trajectory.points[traj_pts-1].positions)

	    if traj is not None:
                # Execute the planned trajectory
                self.arm.execute(traj)
	    else:
		rospy.loginfo("No motion found")

	    rospy.set_param('smach_state', 'Linear manual moving arm')
	    #self.arm.go()
	    rospy.sleep(0.01)

	# Moving Joints (step by step):
	if (active):
	    try:
	        actual_pos = rospy.get_param('current_joint_state')
	    except:
	    	actual_pos = [0,0,0,0,0,0]
	    print "Actual position: ", actual_pos

	    ## Joint control 
	    # Define velocity message
	    pan_travel = (self.step* pan_vel)/2.0;
	    pan = max(self.min_pos, min(self.max_pos, actual_pos[0] + pan_travel))
	    print "Pan params: ", pan_vel, pan_travel, pan
	    #travel[0] = pan_travel 	

	    lift_travel = (self.step* lift_vel)/2.0;
	    lift = max(self.min_pos, min(self.max_pos, actual_pos[1] + lift_travel))
	    print "Lift params: ", lift_vel, lift_travel, lift
	    #travel[1] = lift_travel

	    elbow_travel = (self.step* elbow_vel)/2.0;
	    elbow = max(self.min_pos, min(self.max_pos, actual_pos[2] + elbow_travel))
	    print "Elbow params: ", elbow_vel, elbow_travel, elbow
	    #travel[2] = elbow_travel

	    wrist1_travel = (self.step* wrist1_vel)/2.0;
	    wrist1 = max(self.min_pos, min(self.max_pos, actual_pos[3] + wrist1_travel))
	    print "Wrist1 params: ", wrist1_vel, wrist1_travel, wrist1
	    #travel[3] = wrist1_travel

	    wrist2_travel = (self.step* wrist2_vel)/2.0;
	    wrist2 = max(self.min_pos, min(self.max_pos, actual_pos[4] + wrist2_travel))
	    print "Wrist2 params: ", wrist2_vel, wrist2_travel, wrist2
	    #travel[4] = wrist2_travel

	    wrist3_travel = (self.step* wrist3_vel)/2.0;
	    wrist3 = max(self.min_pos, min(self.max_pos, actual_pos[5] + wrist3_travel))
	    print "Wrist3 params: ", wrist3_vel, wrist3_travel, wrist3
	    #travel[5] = wrist3_travel
	
	    # Getting goal and move the arm
	    goal = FollowJointTrajectoryGoal()
	    goal.trajectory.joint_names.append(shoulder_pan_joint)
	    goal.trajectory.joint_names.append(shoulder_lift_joint)
	    goal.trajectory.joint_names.append(elbow_joint)
	    goal.trajectory.joint_names.append(wrist1_joint)
	    goal.trajectory.joint_names.append(wrist2_joint)
	    goal.trajectory.joint_names.append(wrist3_joint)

	    p = JointTrajectoryPoint()
	    p.positions.append(pan)
	    p.positions.append(lift)
	    p.positions.append(elbow)
	    p.positions.append(wrist1)
	    p.positions.append(wrist2)
	    p.positions.append(wrist3)
   
	    p.velocities.append(pan_vel)
	    p.velocities.append(lift_vel)
	    p.velocities.append(elbow_vel)
	    p.velocities.append(wrist1_vel)
	    p.velocities.append(wrist2_vel)
	    p.velocities.append(wrist3_vel)
  
	    p.time_from_start = rospy.Duration(self.step) 	    
	    goal.trajectory.points.append(p)
	    goal.goal_time_tolerance = rospy.Duration(0.0)
	    goal_pts = len(goal.trajectory.points)

	    # update current joint states
	    rospy.set_param('current_joint_state',goal.trajectory.points[goal_pts-1].positions)

	    arm_client.send_goal(goal)
	
	    rospy.set_param('smach_state', 'Manual moving joints')
	    
	    #rospy.sleep(0.01)
	#break pass
	  
	# Moving gripper
	if (active_g):
	    try:
		physical_robot = rospy.get_param('physical_robot')
	    except:
		physical_robot = 'False'
	    if not physical_robot:
		grip = Twist()
	    	if g_control > 0:
		    grip.linear.x = 0
		    grip.linear.y = 0
		    grip.linear.z = 0
		    grip.angular.x = 0
		    grip.angular.y = 0
		    grip.angular.z = 1
		    print "closing gripper"
	    	elif g_control < 0:
		    grip.linear.x = 0
		    grip.linear.y = 0
		    grip.linear.z = 0
		    grip.angular.x = 0
		    grip.angular.y = 0
		    grip.angular.z = -1
	    	    print "openning gripper"
	    	self.grip_gazebo_pub.publish(grip)
	    	rospy.set_param('smach_state','Manual gripper')
	    else:
		req = Int8()
		if g_control > 0:
		    req = 2
		elif g_control < 0:
		    req = 3
		self.grip_physical_pub.publish(req)
		rospy.set_param('smach_state','Manual gripper')
	
	

    def joint_state(self, state):
	self.state = state
	return state
	
    def integrate(self,desired, present, max_rate, dt):
	if (desired > present):
	    return min(desired, present + max_rate*dt)
	else:
	    return max(desired, present - max_rate*dt)

    

if __name__ == '__main__':
    try:
	joy_ur5()
	rospy.spin()
    except KeyboardInterrupt:
	print("Shuting down manual joy node")

