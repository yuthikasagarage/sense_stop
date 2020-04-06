#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "tf/transform_datatypes.h"
#include <boost/thread.hpp>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
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
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void state_cb(const mavros_msgs::State::ConstPtr &msg);
  void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void accel_cb(const sensor_msgs::Imu::ConstPtr &msg);
  void quad_cb(const std_msgs::String::ConstPtr &msg);
  void min_cb(const std_msgs::Float32MultiArray::ConstPtr &array);
  void compute_velocity();
  void Publish_2();
  float vx;
  std_msgs::Float32MultiArray array;
  float vy;
  float orientation;
  float yaw_radians;
  string laserscan;
  float vya;
  float vxa;
  bool activate_;
  bool follow;
  float quad1min;
  float quad2min;
  float quad3min;
  float quad4min;
  float q1min_angle;
  float q2min_angle;
  float q3min_angle;
  float q4min_angle;

  float keep_distnace_y(float desired_distance, float minVal, float quad1min,
                        float quad2min, float quad4min, float quad3min);
  float keep_distnace_x(float desired_distance, float minVal, float quad1min,
                        float quad2min, float quad4min, float quad3min);

  float initalyaw(double desired_yaw, float orientation);
  float desired_distance;
  ros::Publisher override_pub_;
  ros::Subscriber scan_sub;
  ros::Subscriber state_sub;
  ros::Subscriber velocity_sub;
  ros::Subscriber accel_sub;
  ros::Subscriber position_sub;
  ros::Subscriber quad_sub;
  geometry_msgs::PoseStamped local_pose;
  geometry_msgs::PoseStamped uavPose;
  ros::Publisher bodyAxisVelocityPublisher;
  ros::Publisher bodyAxisPublisher;
  ros::Publisher local_pos_pub;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient frame_service;
  int stop_counter;
  mavros_msgs::State current_state;
  sensor_msgs::LaserScan a;
  ros::Subscriber sub_min;

  string prev_mode_name;
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
  float px;
  float py;
  float pz;
  float last_pz;
  float last_vbz;
  float yaw_error;
  float last_yaw;
  std_msgs::String current_quad;
  ros::Time last_request;
  geometry_msgs::Twist offboard_twist;
  struct PID {
    float Kp;
    float Kd;
    float Ki;
  };

private:
};

tsa::tsa()

{
  ros::NodeHandle nh;

  ros::ServiceClient streamingRate_cl =
      nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

  // local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
  //   "mavros/setpoint_position/local", 60);

  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  frame_service = nh.serviceClient<mavros_msgs::SetMavFrame>(
      "mavros/setpoint_velocity/mav_frame");
  ros::Rate rate(100.0);
  mavros_msgs::SetMavFrameRequest mavframe_req;
  mavframe_req.mav_frame = 8;
  mavros_msgs::SetMavFrameResponse mavframe_resp;
  frame_service.call(mavframe_req, mavframe_resp);

  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::SetMode prev_mode;
  //  Set Higher Stream Rate for the remote
  mavros_msgs::StreamRate srv_setStreamRate;
  srv_setStreamRate.request.stream_id = 3;
  srv_setStreamRate.request.message_rate = 60;
  srv_setStreamRate.request.on_off = 1;
  streamingRate_cl.call(srv_setStreamRate);

  // bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>(
  //  "/mavros/setpoint_velocity/cmd_vel", 10);

  //----------------------------------------------------------
  // using ssetpoint raw for consistency

  bodyAxisPublisher = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 60);
  //--------------------------------------------------------------
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 60,
                                               &tsa::state_cb, this);

  quad_sub = nh.subscribe("/scan/quadrant", 100, &tsa::quad_cb, this);

  velocity_sub = nh.subscribe("/mavros/local_position/velocity", 60,
                              &tsa::velocity_cb, this);

  sub_min = nh.subscribe("/scan/min", 10, &tsa::min_cb, this);

  position_sub =
      nh.subscribe("mavros/local_position/pose", 60, &tsa::pose_cb, this);

  accel_sub = nh.subscribe("/mavros/imu/data", 60, &tsa::accel_cb, this);
  scan_sub = nh.subscribe("/laser/scan", 10, &tsa::laserCallback, this);

  ROS_INFO("Initialization complete");
  stop_counter = 0;
  ros::spinOnce();
  rate.sleep();
}

tsa::~tsa() {}

void tsa::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  px = msg->pose.position.x;
  py = msg->pose.position.y;
  pz = msg->pose.position.z;
  local_pose = *msg;
  orientation = (float)(tf::getYaw(local_pose.pose.orientation));
  // * 180 / M_PI

  yaw_radians = orientation * 180 / M_PI;
}

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
}

void tsa::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  vbx = msg->twist.linear.x;
  vby = msg->twist.linear.y;
  vbz = msg->twist.linear.z;
}
void tsa::accel_cb(const sensor_msgs::Imu::ConstPtr &msg) {
  abx = msg->linear_acceleration.x;
  aby = msg->linear_acceleration.y;
  abz = msg->linear_acceleration.z;
}

void tsa::Publish() {

  mavros_msgs::PositionTarget vp_body_axis;
  vp_body_axis.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  vp_body_axis.type_mask = 0b101111000111;

  vp_body_axis.velocity.z = 0.0;
  ros::Rate rate(60.0);
  if (current_state.mode != "OFFBOARD") {
    prev_mode_name = current_state.mode;
    last_pz = pz;
    last_vbz = vbz;
    last_yaw = orientation;

    vp_body_axis.position.z = last_pz;
    vp_body_axis.yaw = last_yaw;
    vp_body_axis.yaw_rate = 1.0;
    bodyAxisPublisher.publish(vp_body_axis);
  }

  if (current_state.mode == "OFFBOARD") {
    ros::Rate rate(60.0);

    vp_body_axis.position.z = last_pz;
    vp_body_axis.velocity.x = vx;
    vp_body_axis.velocity.y = vy;

    vp_body_axis.yaw_rate = 1.0;
    vp_body_axis.yaw = last_yaw;

    bodyAxisPublisher.publish(vp_body_axis);
  }
}

void tsa::Publish_2() {

  mavros_msgs::PositionTarget vp_body_axis;
  vp_body_axis.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  vp_body_axis.type_mask = 0b101111000111;
  vp_body_axis.yaw = yaw_error;

  vp_body_axis.velocity.z = 0.0;
  ros::Rate rate(60.0);
  if (current_state.mode != "OFFBOARD") {
    prev_mode_name = current_state.mode;
    last_pz = pz;
    last_vbz = vbz;
    yaw_error = orientation;
    vy = 0.0;
    vx = 0.5;

    vp_body_axis.position.z = last_pz;
    vp_body_axis.yaw = yaw_error;
    vp_body_axis.yaw_rate = 1.0;
    vp_body_axis.velocity.x = vx;
    vp_body_axis.velocity.y = vy;
    bodyAxisPublisher.publish(vp_body_axis);
  }

  if (current_state.mode == "OFFBOARD") {
    ros::Rate rate(60.0);

    vp_body_axis.position.z = last_pz;
    vp_body_axis.velocity.x = vx;
    vp_body_axis.velocity.y = vy;

    vp_body_axis.yaw_rate = 1.0;
    vp_body_axis.yaw = yaw_error;

    bodyAxisPublisher.publish(vp_body_axis);
  }
}

void tsa::quad_cb(const std_msgs::String::ConstPtr &msg) {
  current_quad = *msg;
}
void tsa::min_cb(const std_msgs::Float32MultiArray::ConstPtr &msg) {

  array = *msg;
  quad1min = array.data[0];
  q1min_angle = array.data[1];

  quad2min = array.data[2];
  q2min_angle = array.data[3];

  quad3min = array.data[4];
  q3min_angle = array.data[5];

  quad4min = array.data[6];
  q4min_angle = array.data[7];
}

void tsa::compute_velocity() {
  // searching for a wall and stop near the wall as out desired distance from
  // the wall.
  cout << stop_counter << endl;
  if (minVal > 3) {
    cout << "searching for a wall , moving to the right" << endl;
    vy = 0.0;
    vx = 0.5;

    Publish();
  }

  if (quad1min < 3 && quad1min > 2) {
    cout << quad1min << endl;
    stop_counter = 51;
  }

  if (minVal < 3 && stop_counter == 51) {
    cout << " trying to keep distance to the wall while moving ahead" << endl;

    if (q1min_angle > 50) {
      if (quad2min < 3) {

        yaw_error = yaw_error + (M_PI / 540) * (5 - quad2min);

      } else {

        yaw_error = yaw_error + (M_PI / 540);
      }

    } else if (q1min_angle < 40) {
      if (quad2min < 3) {

        yaw_error = yaw_error - (M_PI / 540) * (5 - quad2min);

      } else {

        yaw_error = yaw_error - (M_PI / 540);
      }

    } else if (q1min_angle < 50 && quad2min < 3) {
      yaw_error = yaw_error + (M_PI / 540) * (5 - quad2min);

      vy = 0.5;

    } else if (q1min_angle > 40 && quad2min < 3) {
      yaw_error = yaw_error - (M_PI / 540) * (5 - quad2min);

      vy = 0.5;

    } else {

      yaw_error = orientation;
      vy = 0.5;
    }

    vx = keep_distnace_x(2, minVal, quad1min, quad2min, quad3min, quad4min);

    Publish_2();
  }
}

float tsa::keep_distnace_x(float desired_distance, float minVal, float quad1min,
                           float quad2min, float quad4min, float quad3min) {

  vxa = 0.5 * (minVal - desired_distance);
  return vxa;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mavros_asf_follow");

  tsa tsa1;
  ros::Rate rate(60.0);
  while (ros::ok()) {
    ros::spinOnce();
    tsa1.compute_velocity();

    rate.sleep();
  }
  return 0;
}
