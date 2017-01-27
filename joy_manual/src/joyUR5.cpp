#include <algorithm>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <topic_tools/MuxSelect.h>

double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}


class TeleopComponent
{
public:
  TeleopComponent() : active_(false) {}
  virtual ~TeleopComponent() {}

  // This gets called whenever new joy message comes in
  // returns whether lower priority teleop components should be stopped
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state) = 0;

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt) = 0;

  // Start the component. Must be idempotent.
  virtual bool start()
  {
    active_ = true;
    return active_;
  }

  // Stop the component. Must be idempotent.
  virtual bool stop()
  {
    active_ = false;
    return active_;
  }

protected:
  bool active_;
};


class BaseTeleop : public TeleopComponent
{
public:
  BaseTeleop(const std::string& name, ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, name);

    // Button mapping
    pnh.param("button_deadman", deadman_, 10);
    pnh.param("axis_x", axis_x_, 3);
    pnh.param("axis_w", axis_w_, 0);

    // Base limits
    pnh.param("max_vel_x", max_vel_x_, 1.0);
    pnh.param("min_vel_x", min_vel_x_, -0.5);
    pnh.param("max_vel_w", max_vel_w_, 3.0);
    pnh.param("max_acc_x", max_acc_x_, 1.0);
    pnh.param("max_acc_w", max_acc_w_, 3.0);

    // Maximum windup of acceleration ramping
    pnh.param("max_windup_time", max_windup_time, 0.25);

    // Mux for overriding navigation, etc.
    pnh.param("use_mux", use_mux_, true);
    if(use_mux_)
    {
      mux_ = nh.serviceClient<topic_tools::MuxSelect>("/cmd_vel_mux/select");
    }

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 1);
    odom_sub_ = nh.subscribe("/odom", 1, &BaseTeleop::odomCallback, this);
  }

  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed = joy->buttons[deadman_];

    if (!deadman_pressed)
    {
      stop();
      return false;
    }

    start();

    if (joy->axes[axis_x_] > 0.0)
      desired_.linear.x = joy->axes[axis_x_] * max_vel_x_;
    else
      desired_.linear.x = joy->axes[axis_x_] * -min_vel_x_;
    desired_.angular.z = joy->axes[axis_w_] * max_vel_w_;

    // We are active, don't process lower priority components
    return true;
  }

  virtual void publish(const ros::Duration& dt)
  {
    if (active_)
    {
      {
        boost::mutex::scoped_lock lock(odom_mutex_);
        // Make sure base is actually keeping up with commands
        // When accelerating (in either direction) do not continue to ramp our
        //   acceleration more than max_windup_time ahead of actually attained speeds.
        // This is especially important if robot gets stuck.
        if (last_.linear.x >= 0)
          last_.linear.x = std::min(last_.linear.x, odom_.twist.twist.linear.x + max_acc_x_ * max_windup_time);
        else
          last_.linear.x = std::max(last_.linear.x, odom_.twist.twist.linear.x - max_acc_x_ * max_windup_time);
        if (last_.angular.z >= 0)
          last_.angular.z = std::min(last_.angular.z, odom_.twist.twist.angular.z + max_acc_w_ * max_windup_time);
        else
          last_.angular.z = std::max(last_.angular.z, odom_.twist.twist.angular.z - max_acc_w_ * max_windup_time);
      }
      // Ramp commands based on acceleration limits
      last_.linear.x = integrate(desired_.linear.x, last_.linear.x, max_acc_x_, dt.toSec());
      last_.angular.z = integrate(desired_.angular.z, last_.angular.z, max_acc_w_, dt.toSec());
      cmd_vel_pub_.publish(last_);
    }
  }

  virtual bool start()
  {
    if (!active_ && use_mux_)
    {
      // Connect mux
      topic_tools::MuxSelect req;
      req.request.topic = cmd_vel_pub_.getTopic();
      if (mux_.call(req))
      {
        prev_mux_topic_ = req.response.prev_topic;
      }
      else
      {
        ROS_ERROR("Unable to switch mux");
      }
    }
    active_ = true;
    return active_;
  }

  virtual bool stop()
  {
    // Publish stop message
    last_ = desired_ = geometry_msgs::Twist();
    cmd_vel_pub_.publish(last_);
    // Disconnect mux
    if (active_ && use_mux_)
    {
      topic_tools::MuxSelect req;
      req.request.topic = prev_mux_topic_;
      if (!mux_.call(req))
      {
        ROS_ERROR("Unable to switch mux");
        return active_;
      }
    }
    active_ = false;
    return active_;
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& odom)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(odom_mutex_);
    odom_ = *odom;
  }

  // Buttons from params
  int deadman_, axis_x_, axis_w_;

  // Limits from params
  double max_vel_x_, min_vel_x_, max_vel_w_;
  double max_acc_x_, max_acc_w_;

  // Support for multiplexor between teleop and application base commands
  bool use_mux_;
  std::string prev_mux_topic_;
  ros::ServiceClient mux_;

  // Twist output, odometry feedback
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;

  // Latest feedback, mutex around it
  boost::mutex odom_mutex_;
  nav_msgs::Odometry odom_;
  // Maximum timestep that our ramping can get ahead of actual velocities
  double max_windup_time;

  geometry_msgs::Twist desired_;
  geometry_msgs::Twist last_;
};


// Arm
class ArmTeleop : public TeleopComponent
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

public:
  ArmTeleop(const std::string& name, ros::NodeHandle& nh) :
    last_pan_(0.0),
    last_lift_(0.0)
  {
    ros::NodeHandle pnh(nh, name);

    // Button mapping
    pnh.param("button_deadman", deadman_, 8);
    pnh.param("axis_pan", axis_pan_, 0);
    pnh.param("axis_tilt", axis_lift_, 3);

    // Joint limits
    pnh.param("max_vel_pan", max_vel_pan_, 3.1415);
    pnh.param("max_vel_tilt", max_vel_lift_, 3.1415);
    pnh.param("max_acc_pan", max_acc_pan_, 3.15);
    pnh.param("max_acc_tilt", max_acc_lift_, 3.15);
    pnh.param("min_pos_pan", min_pos_pan_, -3.1415);
    pnh.param("max_pos_pan", max_pos_pan_, 3.1415);
    pnh.param("min_pos_tilt", min_pos_lift_, -3.1415);
    pnh.param("max_pos_tilt", max_pos_lift_, 3.1415);

    // TODO: load topic from params
    shoulder_pan_joint_ = "ur5_arm_shoulder_pan_joint";
    shoulder_lift_joint_ = "ur5_arm_shoulder_lift_joint";
    //elbow_joint = "ur5_arm_elbow_joint"
    //wrist1_joint = "ur5_arm_wrist_1_joint"
    //wrist2_joint = "ur5_arm_wrist_2_joint"
    //wrist3_joint = "ur5_arm_wrist_3_joint"

    std::string action_name = "arm_controller/follow_joint_trajectory/*";
    client_.reset(new client_t(action_name, true));
    if (!client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("%s may not be connected.", action_name.c_str());
    }
  }

  // This gets called whenever new joy message comes in
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed = joy->buttons[deadman_];

    if (!deadman_pressed)
    {
      stop();
      // Update joint positions
      for (size_t i = 0; i < state->name.size(); i++)
      {
        if (state->name[i] == shoulder_pan_joint_)
          actual_pos_pan_ = state->position[i];
        if (state->name[i] == shoulder_lift_joint_)
          actual_pos_lift_ = state->position[i];
      }
      return false;
    }

    desired_pan_ = joy->axes[axis_pan_] * max_vel_pan_;
    desired_lift_ = joy->axes[axis_lift_] * max_vel_lift_;
    start();

    return true;
  }

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt)
  {
    if (active_)
    {
      // Fill in message (future dated with fixed time step)
      double step = 0.125;
      double pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, step);
      double pan_travel = step * (pan_vel + last_pan_) / 2.0;
      double pan = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
      double lift_vel = integrate(desired_lift_, last_lift_, max_acc_lift_, step);
      double lift_travel = step * (lift_vel + last_lift_) / 2.0;
      double lift = std::max(min_pos_lift_, std::min(max_pos_lift_, actual_pos_lift_ + lift_travel));
      // Publish message
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names.push_back(shoulder_pan_joint_);
      goal.trajectory.joint_names.push_back(shoulder_lift_joint_);
      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.push_back(pan);
      p.positions.push_back(lift);
      p.velocities.push_back(pan_vel);
      p.velocities.push_back(lift_vel);
      p.time_from_start = ros::Duration(step);
      goal.trajectory.points.push_back(p);
      goal.goal_time_tolerance = ros::Duration(0.0);
      client_->sendGoal(goal);
      // Update based on actual timestep
      pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, dt.toSec());
      pan_travel = dt.toSec() * (pan_vel + last_pan_) / 2.0;
      actual_pos_pan_ = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
      last_pan_ = pan_vel;
      lift_vel = integrate(desired_lift_, last_lift_, max_acc_lift_, dt.toSec());
      lift_travel = dt.toSec() * (lift_vel + last_lift_) / 2.0;
      actual_pos_lift_ = std::max(min_pos_lift_, std::min(max_pos_lift_, actual_pos_lift_ + lift_travel));
      last_lift_ = lift_vel;
    }
  }

  virtual bool stop()
  {
    active_ = false;
    last_pan_ = last_lift_ = 0.0;  // reset velocities
    return active_;
  }

private:
  int deadman_, axis_pan_, axis_lift_;
  double max_vel_pan_, max_vel_lift_;
  double max_acc_pan_, max_acc_lift_;
  double min_pos_pan_, max_pos_pan_, min_pos_lift_, max_pos_lift_;
  std::string shoulder_pan_joint_, shoulder_lift_joint_;
  double actual_pos_pan_, actual_pos_lift_;  // actual positions
  double desired_pan_, desired_lift_;  // desired velocities
  double last_pan_, last_lift_;
  boost::shared_ptr<client_t> client_;
};


// This pulls all the components together
class Teleop
{
  typedef boost::shared_ptr<TeleopComponent> TeleopComponentPtr;

public:
  void init(ros::NodeHandle& nh)
  {
    bool is_ur5;
    nh.param("is_ur5", is_ur5, true);

    // TODO: load these from YAML

    TeleopComponentPtr c;
    if (is_ur5)
    {
      // Head overrides base
      c.reset(new ArmTeleop("head", nh));
      components_.push_back(c);
    }

    // BaseTeleop goes last
    c.reset(new BaseTeleop("base", nh));
    components_.push_back(c);

    state_msg_.reset(new sensor_msgs::JointState());
    joy_sub_ = nh.subscribe("/joy", 1, &Teleop::joyCallback, this);
    state_sub_ = nh.subscribe("/joint_states", 10, &Teleop::stateCallback, this);
  }

  void publish(const ros::Duration& dt)
  {
    if (ros::Time::now() - last_update_ > ros::Duration(0.25))
    {
      // Timed out
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->stop();
      }
    }
    else
    {
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->publish(dt);
      }
    }
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    bool ok = true;
    for (size_t c = 0; c < components_.size(); c++)
    {
      if (ok)
      {
        ok &= !components_[c]->update(msg, state_msg_);
      }
      else
      {
        // supressed by a higher priority component
        components_[c]->stop();
      }
    }
    last_update_ = ros::Time::now();
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    // Update each joint based on message
    for (size_t msg_j = 0; msg_j < msg->name.size(); msg_j++)
    {
      size_t state_j;
      for (state_j = 0; state_j < state_msg_->name.size(); state_j++)
      {
        if (state_msg_->name[state_j] == msg->name[msg_j])
        {
          state_msg_->position[state_j] = msg->position[msg_j];
          state_msg_->velocity[state_j] = msg->velocity[msg_j];
          break;
        }
      }
      if (state_j == state_msg_->name.size())
      {
        // New joint
        state_msg_->name.push_back(msg->name[msg_j]);
        state_msg_->position.push_back(msg->position[msg_j]);
        state_msg_->velocity.push_back(msg->velocity[msg_j]);
      }
    }
  }

  std::vector<TeleopComponentPtr> components_;
  ros::Time last_update_;
  boost::mutex state_mutex_;
  sensor_msgs::JointStatePtr state_msg_;
  ros::Subscriber joy_sub_, state_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n("~");

  Teleop teleop;
  teleop.init(n);

  ros::Rate r(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    teleop.publish(ros::Duration(1/30.0));
    r.sleep();
  }

  return 0;
}
