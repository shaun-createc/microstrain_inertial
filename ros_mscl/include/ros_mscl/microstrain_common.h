/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MICROSTRAIN_COMMON_H
#define _MICROSTRAIN_COMMON_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fstream>

#include "mscl/mscl.h"

// Depending on the ROS version we will use different functions and types
#if MICROSTRAIN_ROS_VERSION==1
//ROS
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//MSCL
#include "mscl_msgs/Status.h"
#include "mscl_msgs/RTKStatus.h"
#include "mscl_msgs/FilterStatus.h"
#include "mscl_msgs/FilterHeading.h"
#include "mscl_msgs/FilterHeadingState.h"
#include "mscl_msgs/GPSCorrelationTimestampStamped.h"
#include "mscl_msgs/GNSSAidingStatus.h"
#include "mscl_msgs/GNSSDualAntennaStatus.h"

//Different message namespaces between ROS1 and ROS2
namespace Microstrain {
using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::NavSatFix;
using sensor_msgs::TimeReference;
using nav_msgs::Odometry;
using mscl_msgs::GPSCorrelationTimestampStamped;
using mscl_msgs::GNSSAidingStatus;
using mscl_msgs::GNSSDualAntennaStatus;
using mscl_msgs::RTKStatus;
using mscl_msgs::FilterStatus;
using mscl_msgs::FilterHeadingState;
using mscl_msgs::FilterHeading;
using mscl_msgs::Status;

using RosTime = ros::Time;
#define ROS_TIME_FROM_NSEC(TIME) ros::Time().fromNSec(time)
}  // namespace Microstrain
#elif MICROSTRAIN_ROS_VERSION==2
//ROS
#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//MSCL
#include "ros2_mscl_msgs/msg/status.hpp"
#include "ros2_mscl_msgs/msg/rtk_status.hpp"
#include "ros2_mscl_msgs/msg/filter_status.hpp"
#include "ros2_mscl_msgs/msg/filter_heading.hpp"
#include "ros2_mscl_msgs/msg/filter_heading_state.hpp"
#include "ros2_mscl_msgs/msg/gps_correlation_timestamp_stamped.hpp"
#include "ros2_mscl_msgs/msg/gnss_aiding_status.hpp"
#include "ros2_mscl_msgs/msg/gnss_dual_antenna_status.hpp"

namespace Microstrain {
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::TimeReference;
using nav_msgs::msg::Odometry;
using ros2_mscl_msgs::msg::GPSCorrelationTimestampStamped;
using ros2_mscl_msgs::msg::GNSSAidingStatus;
using ros2_mscl_msgs::msg::GNSSDualAntennaStatus;
using ros2_mscl_msgs::msg::RTKStatus;
using ros2_mscl_msgs::msg::FilterStatus;
using ros2_mscl_msgs::msg::FilterHeadingState;
using ros2_mscl_msgs::msg::FilterHeading;
using ros2_mscl_msgs::msg::Status;

using RosTime = rclcpp::Time;
#define ROS_TIME_FROM_NSEC(TIME) rclcpp::Time(static_cast<int64_t>(TIME))
}
#else
#error "Invalid ROS version"
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

#define SECS_PER_WEEK (60L*60*24*7)
#define UTC_GPS_EPOCH_DUR (315964800)

#define USTRAIN_G 9.80665  // from section 5.1.1 in https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

//Macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

#define GNSS1_ID 0
#define GNSS2_ID 1
#define NUM_GNSS 2



/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{
  
class MicrostrainCommon
{
 public:
  void parse_mip_packet(const mscl::MipDataPacket& packet);
  void parse_imu_packet(const mscl::MipDataPacket& packet);
  void parse_filter_packet(const mscl::MipDataPacket& packet);
  void parse_gnss_packet(const mscl::MipDataPacket& packet, const RosTime& ros_time, int gnss_id);
  void parse_rtk_packet(const mscl::MipDataPacket& packet);

  virtual void parse_and_publish_imu_packet(const mscl::MipDataPacket& packet) = 0;
  virtual void parse_and_publish_filter_packet(const mscl::MipDataPacket& packet) = 0;
  virtual void parse_and_publish_gnss_packet(const mscl::MipDataPacket& packet, int gnss_id) = 0;
  virtual void parse_and_publish_rtk_packet(const mscl::MipDataPacket& packet) = 0;

 protected: 
  //Variables/fields
  std::unique_ptr<mscl::InertialNode> m_inertial_device;

  //Info for converting to the ENU frame
  bool m_use_enu_frame;
  tf2::Matrix3x3 m_t_ned2enu;

  //Flag for using device timestamp instead of PC received time
  bool m_use_device_timestamp;

  //Packet Counters (valid, timeout, and checksum errors)
  uint32_t m_imu_valid_packet_count;
  uint32_t m_gnss_valid_packet_count[NUM_GNSS];
  uint32_t m_filter_valid_packet_count;
  uint32_t m_rtk_valid_packet_count;

  uint32_t m_imu_timeout_packet_count;
  uint32_t m_gnss_timeout_packet_count[NUM_GNSS];
  uint32_t m_filter_timeout_packet_count;

  uint32_t m_imu_checksum_error_packet_count;
  uint32_t m_gnss_checksum_error_packet_count[NUM_GNSS];
  uint32_t m_filter_checksum_error_packet_count;


  //Data field storage
  //IMU
  bool m_has_mag;
  bool m_time_valid;
  bool m_gnss_aiding_status_received[NUM_GNSS];

  float m_curr_imu_mag_x;
  float m_curr_imu_mag_y;
  float m_curr_imu_mag_z;

  mscl::Vector m_curr_ahrs_quaternion;

  //FILTER
  double m_gps_leap_seconds;

  double m_curr_filter_pos_lat;
  double m_curr_filter_pos_long;
  double m_curr_filter_pos_height;

  float m_curr_filter_vel_north;
  float m_curr_filter_vel_east;
  float m_curr_filter_vel_down;

  mscl::Vector m_curr_filter_quaternion;

  float m_curr_filter_roll;
  float m_curr_filter_pitch;
  float m_curr_filter_yaw;

  float m_curr_filter_angular_rate_x;
  float m_curr_filter_angular_rate_y;
  float m_curr_filter_angular_rate_z;

  float m_curr_filter_pos_uncert_north;
  float m_curr_filter_pos_uncert_east;
  float m_curr_filter_pos_uncert_down;

  float m_curr_filter_vel_uncert_north;
  float m_curr_filter_vel_uncert_east;
  float m_curr_filter_vel_uncert_down;

  float m_curr_filter_att_uncert_roll;
  float m_curr_filter_att_uncert_pitch;
  float m_curr_filter_att_uncert_yaw;

  //Frame ids
  std::string m_imu_frame_id;
  std::string m_gnss_frame_id[NUM_GNSS];
  std::string m_filter_frame_id;
  std::string m_filter_child_frame_id;
 
  //Topic strings
  std::string m_velocity_zupt_topic;
  std::string m_angular_zupt_topic;
  std::string m_external_gps_time_topic;
  
  //Publish data flags
  bool m_publish_imu;
  bool m_publish_gps_corr;
  bool m_publish_gnss[NUM_GNSS];
  bool m_publish_gnss_aiding_status[NUM_GNSS];
  bool m_publish_filter;
  bool m_publish_filter_relative_pos;
  bool m_publish_rtk;

  //ZUPT, angular ZUPT topic listener variables
  bool m_angular_zupt;
  bool m_velocity_zupt;
  
  bool m_vel_still;
  bool m_ang_still;
  
  //Static covariance vectors
  std::vector<double> m_imu_linear_cov;
  std::vector<double> m_imu_angular_cov;
  std::vector<double> m_imu_orientation_cov;

  // Update rates
  int m_imu_data_rate;
  int m_gnss_data_rate[NUM_GNSS];
  int m_filter_data_rate;

  //Gnss antenna offsets
  std::vector<double> m_gnss_antenna_offset[NUM_GNSS];

  //Various settings variables
  clock_t m_start;
  uint8_t m_com_mode;
  float   m_field_data[3];
  float   m_soft_iron[9];
  float   m_soft_iron_readback[9];
  float   m_angles[3];
  float   m_heading_angle;
  float   m_readback_angles[3];
  float   m_noise[3];
  float   m_beta[3];
  float   m_readback_beta[3];
  float   m_readback_noise[3];
  float   m_offset[3];
  float   m_readback_offset[3];
  double  m_reference_position_command[3];
  double  m_reference_position_readback[3];
  uint8_t m_dynamics_mode;

  //Raw data file parameters
  bool          m_raw_file_enable;
  bool          m_raw_file_include_support_data;
  std::ofstream m_raw_file;

  //IMU Messages
  Imu           m_imu_msg;
  MagneticField m_mag_msg;
  GPSCorrelationTimestampStamped m_gps_corr_msg;

  //GNSS Messages
  NavSatFix      m_gnss_msg[NUM_GNSS];
  Odometry          m_gnss_odom_msg[NUM_GNSS];
  TimeReference  m_gnss_time_msg[NUM_GNSS];
  GNSSAidingStatus m_gnss_aiding_status_msg[NUM_GNSS];
  GNSSDualAntennaStatus m_gnss_dual_antenna_status_msg;

  //RTK Messages
  RTKStatus   m_rtk_msg;
 
  //Filter Messages
  Odometry                 m_filter_msg;
  Imu                   m_filtered_imu_msg;
  Odometry                 m_filter_relative_pos_msg;
  FilterStatus            m_filter_status_msg;
  FilterHeadingState      m_filter_heading_state_msg;
  FilterHeading           m_filter_heading_msg;

  //Device Status Message
  Status m_device_status_msg;
};

} // namespace Microstrain

#endif  // _MICROSTRAIN_COMMON_H
