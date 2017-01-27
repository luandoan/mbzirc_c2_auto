#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cmath>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  
  // Toggle TELEOP-ENABLED
  if ( joy->buttons[teleop_toggle] ) { 				// TELEOP-TOGGLE pressed?
    teleop_enabled = !teleop_enabled; 				   // Toggle
    if ( !teleop_enabled ) { 					   // Voice Output/Logging (Disabled)
      ROS_INFO("Teleop Disabled");
      //talk 	(add this later)
      //log  	(add this later)
    }
    else { 							   // Voice Output/Logging (Enabled)
      ROS_INFO("Teleop Enabled");//reset
      //talk 	(add this later)
      //log  	(add this later)
    }
    ros::Duration(1).sleep(); 					// Waits a moment (necessary to keep from double-toggling?)
  }  
  
  // TELEOP-ENABLED? Otherwise, return
  if ( !teleop_enabled ) return; //log	(add this later) 	
  
// Should TRANSLATION-ENABLED be reset when !TELEOP-ENABLED? 
  // Toggle TRANSLATION-ENABLED
  if ( joy->buttons[translation_toggle] ) {
    translation = !translation;
    if ( translation ) {
      ROS_INFO("Translation Enabled");
      //talk 	(add this later)
      //log  	(add this later)
    }
    else {
      ROS_INFO("Translation Disabled");
      //talk 	(add this later)
      //log  	(add this later)
    }
    ros::Duration(.75).sleep();
  }
  
  // Generating Linear Outputs
  if ( translation ) {
      // Linear Inputs
      lin_in[0] = joy->axes[long_axis]; 					// X Input
      lin_in[1] = joy->axes[lat_axis]; 						// Y Input
      lin_in[2] = joy->buttons[z_move_plus]-joy->buttons[z_move_minus]; 	// Z Input

      // Linear Outputs
      lin_out[0] = (lin_in[1]-lin_in[2])*1/sqrt(2)*linear_velocity; 		// X Output
      lin_out[1] = -lin_in[0]*linear_velocity;					// Y Output
      lin_out[2] = (lin_in[2]+lin_in[1])*1/sqrt(2)*linear_velocity; 		// Z Output
  }
  
  // Generating Rotational Outputs
  else {
      // Rotational Inputs
      rot_in[0] = -joy->axes[lat_axis]; 					// X Input
      rot_in[1] = joy->axes[long_axis]; 					// Y Input
      rot_in[2] = joy->buttons[z_move_minus]-joy->buttons[z_move_plus]; 	// Z Input

      // Rotational Outputs
      rot_out[0] = (-rot_in[2]+rot_in[1])*1/sqrt(2)*rotational_velocity; 	// X Output
      rot_out[1] = -rot_in[0]*rotational_velocity; 				// Y Output
      rot_out[2] = (rot_in[2]+rot_in[1])*1/sqrt(2)*rotational_velocity; 	// Z Output
  }

  // LEFT-ARM-MOTION - only if LEFT-DEADMAN is pressed
  if ( joy->axes[left_deadman] < -.8 ) {	// at 1.0 when unpressed; goes to -1.0 when fully pressed
    in_motion = true;  ROS_INFO("%f %f %f %f %f %f", lin_in[0], lin_in[1], lin_in[2], fabs(lin_in[0]), fabs(lin_in[1]), fabs(lin_in[2]));
    if ( joy->buttons[preset_forward] ) {
      ROS_INFO("Moving left arm to Ready position");
      //log  	(add this later)
      vel_pub_l_.publish(preset_pose_l1);
    }
    else if ( joy->buttons[preset_stow] ) {
      ROS_INFO("Moving left arm to Stow position");
      //log  	(add this later)
      vel_pub_l_.publish(preset_pose_l2);
    }
    else if ( fabs(lin_in[0])+fabs(lin_in[1])+fabs(lin_in[2]) > 0 ) {
      if ( !translation ) {
        sprintf(cmd_, "speedl([0.0, 0.0, 0.0, %f, %f, %f], 0.2, 0.1)\n", rot_out[0], rot_out[1], rot_out[2]);
        rspeeds.data = cmd_;
        vel_pub_l_.publish(rspeeds);
ROS_INFO("speedl([0.0, 0.0, 0.0, %f, %f, %f], 0.2, 0.1)\n", rot_out[0], rot_out[1], rot_out[2]);
ros::Duration(.1).sleep();      }
      if ( translation ) {
        sprintf(cmd_, "speedl([ %f, %f, %f, 0.0, 0.0, 0.0], 0.2, 0.1)", lin_out[0], lin_out[1], lin_out[2]);
        lspeeds.data = "fish";//cmd_.str();
        vel_pub_l_.publish(lspeeds);
ROS_INFO("speedl([ %f, %f, %f, 0.0, 0.0, 0.0], 0.2, 0.1)", lin_out[0], lin_out[1], lin_out[2]);
ros::Duration(.1).sleep();
      }
    }
  }

  // RIGHT-ARM-MOTION - only if RIGHT-DEADMAN is pressed
  if ( joy->axes[right_deadman] < -.8 ) {	// at 1.0 when unpressed; goes to -1.0 when fully pressed
  }

  if ( !in_motion )
    ros::Duration(.1).sleep();
  in_motion = false;
}


int main(int argc, char **argv)
{ 

// -----------------------------------------Variable Initialization -----------------------------------------

  linear_velocity = 0.1;
  rotational_velocity = 0.1;
  acceleration = 0.1;
  
  //Params
  
  // State Booleans
  teleop_enabled = false;
  translation = true;
  in_motion = false;

  // Subscriber (Joystick)
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy_teleop/joy", 10, &joyCallback, this);
  // Publishers (Arm Drivers)
  vel_pub_l_ = nh_.advertise<std_msgs::String>("/chatter", 100);
  vel_pub_r_ = nh_.advertise<std_msgs::String>("/right_ur5_controller/right_ur5_URScript", 100);

  // Axis Names (but used like buttons)
  left_deadman = 2; 					// Logitech 'Left Trigger'
  right_deadman = 5; 					// Logitech 'Right Trigger'
  long_axis = 7; 					// Logitech 'U/D Touchpad'
  lat_axis = 6;						// Logitech 'R/L Touchpad'
  // Button Names
  teleop_toggle = 7; 					// Logitech 'Start'
  translation_toggle = 2; 				// Logitech 'X'
  z_move_plus = 5;					// Logitech 'Left Bumper'
  z_move_minus = 4; 					// Logitech 'Right Bumper'
  preset_stow = 1; 					// Logitech 'B'
  preset_forward = 3; 					// Logitech 'Y'

  // Preset Arm Positions
  preset_pose_l1.data = "movej([1.5700663309038547, -0.583366489468496, 1.3239782491742567, -2.245655451074253, -2.3654086332394697, 2.9917274285644684], 1.0, 0.1)\n"; 	// ready
  preset_pose_l2.data = "movej([1.74533, -3.0543, 2.793, -1.6581, -3.0543, 3.04159], 1.0, 0.1)\n"; 									// stow_back
  preset_pose_r1.data = "movej([-1.6723568037420726, -2.7089519656986383, -0.6380074601702258, -1.0541854410679474, 2.143715333202654, 1.6428859457432818], 1.0, 0.1)\n"; 	// stow_back (is this not actually the new ready pose?)
  //preset1_pose_r1.data = "movej([-1.57, -2.88, -0.663, -1.221, 1.92, -1.57], 1.0, 0.1)\n" 										// ready (old? deprecated?)
  preset_pose_r1.data = "movej([-1.74533, -0.0873, -2.793, -1.5708, 3.0543, 3.04159], 1.0, 0.1)\n"; 									// stow_back
  
  ros::spin();

}
