/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_msgs/default_topics.h>

#include <mav_lowlevel_attitude_controller/PID_attitude_controller_node.h>
#include <mav_msgs/ForceAngularAccel.h>

namespace mav_control {

PIDAttitudeControllerNode::PIDAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      got_first_attitude_command_(false),
      PID_attitude_controller_(nh, private_nh)
{

  PID_attitude_controller_.InitializeParams();

  command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
      &PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &PIDAttitudeControllerNode::OdometryCallback, this);

  force_angular_accel_pub_ = nh_.advertise<mav_msgs::ForceAngularAccel>("command/forceangular", 1);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig>::CallbackType f;
  f = boost::bind(&PIDAttitudeControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  SetpointpubCBTimer_ = nh_.createTimer(ros::Duration(0.01), &PIDAttitudeControllerNode::SetpointpubCB, this);  

}

PIDAttitudeControllerNode::~PIDAttitudeControllerNode()
{
}

void PIDAttitudeControllerNode::SetpointpubCB(const ros::TimerEvent& e)
{
    // ROS_INFO("publishing !!.");  
    force_angular_accel_pub_.publish(faac_msg_);
}

void PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference)
{

  PID_attitude_controller_.SetDesiredAttitude(roll_pitch_yawrate_thrust_reference->roll,
                                              roll_pitch_yawrate_thrust_reference->pitch,
                                              roll_pitch_yawrate_thrust_reference->yaw_rate,
                                              roll_pitch_yawrate_thrust_reference->thrust.z);
  got_first_attitude_command_ = true;
}

void PIDAttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");

  if (!got_first_attitude_command_)
    return;

  mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);

  PID_attitude_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  Eigen::VectorXd ref_forceAngAcc;

  PID_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::Actuators turning_velocities_msg;

  turning_velocities_msg.angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);

  PID_attitude_controller_.CalculateForceAngularAcc(&ref_forceAngAcc);
  // std::cout<<"ref_forceAngAcc(0) is "<<ref_forceAngAcc(0)<<std::endl;
  // std::cout<<"ref_forceAngAcc(1) is "<<ref_forceAngAcc(1)<<std::endl;
  // std::cout<<"ref_forceAngAcc(2) is "<<ref_forceAngAcc(2)<<std::endl;

  faac_msg_.force.x = ref_forceAngAcc(0);
  faac_msg_.force.y = ref_forceAngAcc(1);
  faac_msg_.force.z = ref_forceAngAcc(2);
  faac_msg_.angular_accel.x = ref_forceAngAcc(3);
  faac_msg_.angular_accel.y = ref_forceAngAcc(4);
  faac_msg_.angular_accel.z = ref_forceAngAcc(5);

}

void PIDAttitudeControllerNode::DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config,
                                                  uint32_t level)
{

  PID_attitude_controller_.SetPIDParameters(config.roll_gain, config.pitch_gain, config.p_gain,
                                            config.q_gain, config.r_gain, config.roll_int_gain,
                                            config.pitch_int_gain);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  ros::NodeHandle nh, private_nh("~");

  mav_control::PIDAttitudeControllerNode PID_attitude_controller(nh, private_nh);

  ros::spin();

  return 0;
}
