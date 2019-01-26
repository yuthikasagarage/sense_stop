#include <boost/thread.hpp>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StreamRate.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
class ErleShaftAscender {
public:
  ErleShaftAscender();
  ~ErleShaftAscender();
  bool init();
  void Publish();
  void UavPoseReceived(const geometry_msgs::PoseStampedConstPtr &msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  void state_cb(const mavros_msgs::State::ConstPtr &msg);
  float getAvoidRoll() { return (-vy_ * gain_ - vy_der_ * gain_der_); }
  float getAvoidPitch() { return (vx_ * gain_ + vx_der_ * gain_der_); }
  ros::Publisher override_pub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber manual_sub_;
  ros::Subscriber state_sub;
  geometry_msgs::PoseStamped uavPose;
  int inBound(float invalue);
  ros::Publisher bodyAxisVelocityPublisher;
  ros::Publisher offboardVelocityPublisher;
  ros::Publisher local_pos_pub;
  ros::ServiceClient set_mode_client;
  int stop_counter;
  mavros_msgs::State current_state;
  ros::Time last_request;
  geometry_msgs::Twist offboard_twist;

private:
  float saftey_dis_;
  int stick_magnitude_;
  float output_saturate_;
  float vx_, vx_der_;
  float vy_, vy_der_;
  float gain_, gain_der_;
  bool activate_;
  float min_thesh_;
  float MIN_DIST;
};
ErleShaftAscender::ErleShaftAscender()
    : // Initialization list

      saftey_dis_(1.0),
      activate_(false), vx_(0), vx_der_(0), vy_(0), vy_der_(0), min_thesh_(1.0),
      gain_(0.5), gain_der_(0.0), MIN_DIST(0.18) {

  ros::NodeHandle nh;

  ros::ServiceClient streamingRate_cl =
      nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  mavros_msgs::SetMode offb_set_mode;

  //  Set Higher Stream Rate for the remote
  mavros_msgs::StreamRate srv_setStreamRate;
  srv_setStreamRate.request.stream_id = 3;
  srv_setStreamRate.request.message_rate = 50;
  srv_setStreamRate.request.on_off = 1;
  streamingRate_cl.call(srv_setStreamRate);

  bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

  offboardVelocityPublisher = nh.advertise<geometry_msgs::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

  state_sub = nh.subscribe<mavros_msgs::State>(
      "mavros/state", 1, &ErleShaftAscender::state_cb, this);

  scan_sub_ =
      nh.subscribe("/laser/scan", 10, &ErleShaftAscender::laserCallback, this);

  ROS_INFO("Initialization complete");
  stop_counter = 0;
  ros::spin();
}

ErleShaftAscender::~ErleShaftAscender() {}

void ErleShaftAscender::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state = *msg;
}

void ErleShaftAscender::Publish() {

  geometry_msgs::TwistStamped vs_body_axis;

  if (activate_) {
    stop_counter++;

    // if (stop_counter % 3 == 0) {
    cout << current_state.mode << endl;
    if (current_state.mode == "OFFBOARD") {
      return;
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Rate rate(10.0);
    // send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i) {
      // local_pos_pub.publish(pose);
      // offboardVelocityPublisher.publish(offboard_twist);
      vs_body_axis.twist.linear.y = getAvoidPitch();
      vs_body_axis.twist.linear.x = getAvoidRoll();
      bodyAxisVelocityPublisher.publish(vs_body_axis);
      set_mode_client.call(offb_set_mode);

      rate.sleep();
    }
    cout << ros::Time::now() - last_request << endl;
    last_request = ros::Time::now();
    //  while (
    //     ros::ok() && current_state.mode != "OFFBOARD") {
    //   if (current_state.mode != "OFFBOARD" &&
    //       (ros::Time::now() - last_request > ros::Duration(2.0))) {
    //     if (set_mode_client.call(offb_set_mode) &&
    //         offb_set_mode.response.mode_sent) {
    //       ROS_INFO("Offboard enabled");
    //     }
    //     last_request = ros::Time::now();
    //   } else {
    //   }

    // AUTO.LAND
    // local_pos_pub.publish(pose);
    // offboardVelocityPublisher.publish(offboard_twist);

    //  ros::spinOnce();
    // rate.sleep();
    //}

    // local_pos_pub.publish(pose);

    // ros::spinOnce();
    // rate.sleep();
    //}

    //  cout << "2nd loop. offboard activated" << endl;
    //  }

    // if (stop_counter % 10 == 0) {
    // vs_body_axis.twist.linear.x = getAvoidPitch();
    // vs_body_axis.twist.linear.y = getAvoidRoll();
    // bodyAxisVelocityPublisher.publish(vs_body_axis);
    // cout << "3rd loop" << endl;
    //  }
  }
}
// int ErleShaftAscender::inBound(float invalue) {
//   if (invalue > stick_magnitude_)
//     return stick_magnitude_;
//   if (invalue < (-stick_magnitude_))
//     return -stick_magnitude_;
//   return (int)invalue;
// }

void ErleShaftAscender::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan) {
  // The current strategy is to move away from the wall
  // Wall scan creates a gradient if the distance is smaller
  // then 2 meters
  float angle_tmp = scan->angle_min;
  float inc = scan->angle_increment;

  float vx = 0;
  float vy = 0;

  // iterate over the entire scan
  activate_ = false;
  for (std::vector<float>::const_iterator it = scan->ranges.begin();
       it != scan->ranges.end(); it++, angle_tmp = angle_tmp + inc) {
    // only look at this value if it is valid within desired ranges
    if (*it > MIN_DIST && *it < saftey_dis_) {
      float x = (saftey_dis_ - *it) * cosf(angle_tmp);
      float y = (saftey_dis_ - *it) * sinf(angle_tmp);
      // (SAFTEY_DIS-range) describe the gradient
      vx += x;
      vy += y;
      activate_ = true;
    }
  }

  if (activate_) {
    if (abs(vx) < min_thesh_)
      vx = 0;
    if (abs(vy) < min_thesh_)
      vy = 0;
    if (vx == 0 && vy == 0) {
      activate_ = false;
    }
    vx_der_ = vx - vx_;
    vy_der_ = vy - vy_;
    //  ROS_INFO("From the RpLidarA2, most close direction: %f, %f", vx,vy);
    // ROS_INFO("From the RpLidarA2, direction change rate: %f, %f",
    // vx_der_,vy_der_);
  }
  vx_ = vx;
  vy_ = vy;
  Publish();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mavros_ascend_shaft");

  ErleShaftAscender erle_shaft_ascender;

  return 0;
}
