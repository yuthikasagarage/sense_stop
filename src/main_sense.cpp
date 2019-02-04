#include <boost/thread.hpp>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StreamRate.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>
#include <typeinfo>
using namespace std;
class tsa {
public:
  tsa();
  ~tsa();
  bool init();
  void Publish();
  void UavPoseReceived(const geometry_msgs::PoseStampedConstPtr &msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  void state_cb(const mavros_msgs::State::ConstPtr &msg);
  void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void accel_cb(const sensor_msgs::Imu::ConstPtr &msg);
  void quad_cb(const std_msgs::String::ConstPtr &msg);
  float vx;
  float vy;
  bool activate_;
  ros::Publisher override_pub_;
  ros::Subscriber scan_sub;
  ros::Subscriber state_sub;
  ros::Subscriber velocity_sub;
  ros::Subscriber accel_sub;
  ros::Subscriber quad_sub;
  geometry_msgs::PoseStamped uavPose;
  int inBound(float invalue);
  ros::Publisher bodyAxisVelocityPublisher;
  ros::Publisher local_pos_pub;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient frame_service;
  int stop_counter;
  mavros_msgs::State current_state;
  sensor_msgs::LaserScan a;
  double vbx;
  double vby;
  float braking_distance;
  double vbz;
  double abx;
  double aby;
  double abz;
  double vxfinal;
  double vyfinal;
  double minVal;
  std_msgs::String current_quad;
  ros::Time last_request;
  geometry_msgs::Twist offboard_twist;

private:
};
tsa::tsa()

{

  ros::NodeHandle nh;

  ros::ServiceClient streamingRate_cl =
      nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  frame_service = nh.serviceClient<mavros_msgs::SetMavFrame>(
      "mavros/setpoint_velocity/mav_frame");

  mavros_msgs::SetMavFrameRequest mavframe_req;
  mavframe_req.mav_frame = 8;
  mavros_msgs::SetMavFrameResponse mavframe_resp;
  frame_service.call(mavframe_req, mavframe_resp);

  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::SetMode prev_mode;
  //  Set Higher Stream Rate for the remote
  mavros_msgs::StreamRate srv_setStreamRate;
  srv_setStreamRate.request.stream_id = 3;
  srv_setStreamRate.request.message_rate = 50;
  srv_setStreamRate.request.on_off = 1;
  streamingRate_cl.call(srv_setStreamRate);

  bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20,
                                               &tsa::state_cb, this);

  quad_sub = nh.subscribe("/scan/quadrant", 100, &tsa::quad_cb, this);
  velocity_sub = nh.subscribe("/mavros/local_position/velocity", 20,
                              &tsa::velocity_cb, this);
  accel_sub = nh.subscribe("/mavros/imu/data", 20, &tsa::accel_cb, this);
  scan_sub = nh.subscribe("/laser/scan", 10, &tsa::laserCallback, this);

  ROS_INFO("Initialization complete");
  stop_counter = 0;
  ros::spin();
}

tsa::~tsa() {}
void tsa::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {

  minVal = 1000;

  for (int i = 0; i < 360; i++) {
    if ((msg->ranges[i] <= minVal) && (msg->ranges[i] >= msg->range_min) &&
        (msg->ranges[i] <= msg->range_max))
      minVal = msg->ranges[i];
  }
}

void tsa::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state = *msg;
  // cout << current_state << endl;
}

void tsa::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {

  vbx = msg->twist.linear.x;
  vby = msg->twist.linear.y;
  vbz = msg->twist.linear.z;

  // taking these to use as gain to stop the moment of the quad according to the
  // current velocity of the quad.
}
void tsa::accel_cb(const sensor_msgs::Imu::ConstPtr &msg) {

  abx = msg->linear_acceleration.x;
  aby = msg->linear_acceleration.y;
  abz = msg->linear_acceleration.z;

  // taking these to use as gain to stop the moment of the quad according to the
  // current accelararion of the quad.
}

void tsa::Publish() {

  vxfinal = vx * vbx * vbx * 0.5;
  vyfinal = vy * vby * vby * 0.5;
  cout << "x velocity" << vxfinal << endl;
  cout << "y velocity" << vyfinal << endl;
  geometry_msgs::TwistStamped vs_body_axis;
  ros::Rate rate(20.0);

  if (!activate_) {
    cout << "----in not activate--------" << ros::Time::now() - last_request
         << "--breaking distance---" << braking_distance << "--minValue--"
         << minVal << endl;
    cout << current_state.mode << endl;
    mavros_msgs::SetMode prev_mode;
    prev_mode.request.custom_mode = "POSCTL";
    set_mode_client.call(prev_mode);

    if (ros::Time::now() - last_request < ros::Duration(2.0) && minVal < 1) {

      ros::Rate rate(100.0);
      for (int i = 10; ros::ok() && i > 0; --i) {
        bodyAxisVelocityPublisher.publish(vs_body_axis);
        rate.sleep();
      }
    }
  }

  if (activate_) {
    stop_counter++;

    // if (stop_counter % 3 == 0) {
    cout << "---- activate--------" << ros::Time::now() - last_request
         << "------" << braking_distance << "----" << endl;
    cout << current_state.mode << endl;
    // if (current_state.mode == "OFFBOARD") {
    //   return;
    // }

    if (minVal > 2.25) {
      return;
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Rate rate(100.0);

    // send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i) {
    // local_pos_pub.publish(pose);

    vs_body_axis.twist.linear.x = vx + vxfinal;
    vs_body_axis.twist.linear.y = vy + vyfinal;
    set_mode_client.call(offb_set_mode);

    bodyAxisVelocityPublisher.publish(vs_body_axis);
    last_request = ros::Time::now();
    rate.sleep();
    //  }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // while (ros::ok()) {
    //   if (current_state.mode != "OFFBOARD" &&
    //       (ros::Time::now() - last_request > ros::Duration(0.1))) {
    //     if (set_mode_client.call(offb_set_mode) &&
    //         offb_set_mode.response.mode_sent) {
    //       ROS_INFO("Offboard enabled");
    //     }
    //     last_request = ros::Time::now();
    //   }
    //   bodyAxisVelocityPublisher.publish(vs_body_axis);
    //   ros::spin();
    rate.sleep();
  }
}
void tsa::quad_cb(const std_msgs::String::ConstPtr &msg) {
  current_quad = *msg;

  if (msg->data == "2 3") {
    vx = -1.0;
    vy = 0.0;
    cout << "quad 23" << endl;
  }
  if (msg->data == "3 4") {
    vy = -1.0;
    vx = 0.0;
    cout << "quad 34" << endl;
  }

  if (msg->data == "1 2") {
    vy = 1.0;
    vx = 0.0;
    cout << "quad 12" << endl;
  }

  if (msg->data == "1 4") {
    vx = 1.0;
    vy = 0.0;
    cout << "quad 14" << endl;
  }
  if (msg->data == "1") {
    vx = 0.7076;
    vy = 0.7076;
    cout << "quad 1" << endl;
  }
  if (msg->data == "2") {
    vx = -0.7076;
    vy = 0.7076;
    cout << "quad 2" << endl;
  }
  if (msg->data == "3") {
    vx = -0.7076;
    vy = -0.7076;
    cout << "quad 3" << endl;
  }
  if (msg->data == "4") {
    vx = 0.7076;
    vy = -0.7076;
    cout << "quad 4" << endl;
  }
  if (msg->data == "1 2 3") {
    vx = -0.7076;
    vy = 0.7076;
    cout << "quad 1 2 3" << endl;
  }
  if (msg->data == "1 2 4") {
    vx = 0.7076;
    vy = 0.7076;
    cout << "quad 1 2 4" << endl;
  }
  if (msg->data == "1 3 4") {
    vx = 0.7076;
    vy = -0.7076;
    cout << "quad 14" << endl;
  }
  if (msg->data == "2 3 4") {
    vx = -0.7076;
    vy = -0.7076;
    cout << "quad 14" << endl;
  }
  if (msg->data == "1 2 3 4") {
    vx = 0.0;
    vy = 0.0;
    cout << "quad 14" << endl;
  }

  braking_distance =
      sqrt((abs(vbx) * abs(vbx)) + (abs(vby) * abs(vby))) / (-2 * 1);

  braking_distance = abs(braking_distance) * sqrt(abx * abx + aby * aby);

  if (braking_distance < 2) {
    braking_distance = 2;
  }

  // drone offset
  float drone_offset = 0.25; // this is for iris
  braking_distance += drone_offset;

  if (braking_distance > 0.75) {
    cout << "breaking " << braking_distance << " | vx " << vbx << " | vy "
         << vby << " | abx " << abx << " | min " << minVal << endl;
  }

  if (minVal <= braking_distance) {
    activate_ = true;

  } else {
    activate_ = false;
  }
  Publish();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mavros_tsa_sense_stop");

  tsa tsa;

  return 0;
}
