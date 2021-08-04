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


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>

//Common Implementation
#include "microstrain_common.h"

//ROS
#include "ros/ros.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
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
#include "mscl/mscl.h"
#include "ros_mscl/SetAccelBias.h"
#include "ros_mscl/GetAccelBias.h"
#include "ros_mscl/SetGyroBias.h"
#include "ros_mscl/GetGyroBias.h"
#include "ros_mscl/SetHardIronValues.h"
#include "ros_mscl/GetHardIronValues.h"
#include "ros_mscl/SetSoftIronMatrix.h"
#include "ros_mscl/GetSoftIronMatrix.h"
#include "ros_mscl/SetComplementaryFilter.h"
#include "ros_mscl/GetComplementaryFilter.h"
#include "ros_mscl/InitFilterEuler.h"
#include "ros_mscl/InitFilterHeading.h"
#include "ros_mscl/DeviceSettings.h"
#include "ros_mscl/SetAccelBiasModel.h"
#include "ros_mscl/GetAccelBiasModel.h"
#include "ros_mscl/SetGravityAdaptiveVals.h"
#include "ros_mscl/GetGravityAdaptiveVals.h"
#include "ros_mscl/SetSensor2VehicleRotation.h"
#include "ros_mscl/GetSensor2VehicleRotation.h"
#include "ros_mscl/SetSensor2VehicleOffset.h"
#include "ros_mscl/GetSensor2VehicleOffset.h"
#include "ros_mscl/SetReferencePosition.h"
#include "ros_mscl/GetReferencePosition.h"
#include "ros_mscl/SetConingScullingComp.h"
#include "ros_mscl/GetConingScullingComp.h"
#include "ros_mscl/SetEstimationControlFlags.h"
#include "ros_mscl/GetEstimationControlFlags.h"
#include "ros_mscl/SetDynamicsMode.h"
#include "ros_mscl/GetDynamicsMode.h"
#include "ros_mscl/SetZeroAngleUpdateThreshold.h"
#include "ros_mscl/GetZeroAngleUpdateThreshold.h"
#include "ros_mscl/SetZeroVelocityUpdateThreshold.h"
#include "ros_mscl/GetZeroVelocityUpdateThreshold.h"
#include "ros_mscl/SetTareOrientation.h"
#include "ros_mscl/SetAccelNoise.h"
#include "ros_mscl/GetAccelNoise.h"
#include "ros_mscl/SetGyroNoise.h"
#include "ros_mscl/GetGyroNoise.h"
#include "ros_mscl/SetMagNoise.h"
#include "ros_mscl/GetMagNoise.h"
#include "ros_mscl/SetGyroBiasModel.h"
#include "ros_mscl/GetGyroBiasModel.h"
#include "ros_mscl/SetMagAdaptiveVals.h"
#include "ros_mscl/GetMagAdaptiveVals.h"
#include "ros_mscl/SetMagDipAdaptiveVals.h"
#include "ros_mscl/GetMagDipAdaptiveVals.h"
#include "ros_mscl/SetHeadingSource.h"
#include "ros_mscl/GetHeadingSource.h"
#include "ros_mscl/GetSensor2VehicleTransformation.h"
#include "ros_mscl/ExternalHeadingUpdate.h"
#include "ros_mscl/SetRelativePositionReference.h"
#include "ros_mscl/GetRelativePositionReference.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{
  ///
  /// \brief Microstrain class
  ///

  class Microstrain : public MicrostrainCommon
  {
  public:

    Microstrain();
    ~Microstrain() = default;

    void run();

    void parse_and_publish_imu_packet(const mscl::MipDataPacket& packet);
    void parse_and_publish_filter_packet(const mscl::MipDataPacket& packet);
    void parse_and_publish_gnss_packet(const mscl::MipDataPacket& packet, int gnss_id);
    void parse_and_publish_rtk_packet(const mscl::MipDataPacket& packet);

    void device_status_callback();
    bool device_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool get_basic_status(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool get_diagnostic_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  
    bool set_accel_bias(ros_mscl::SetAccelBias::Request &req, ros_mscl::SetAccelBias::Response &res);
    bool get_accel_bias(ros_mscl::GetAccelBias::Request &req, ros_mscl::GetAccelBias::Response &res);

    bool set_gyro_bias(ros_mscl::SetGyroBias::Request &req, ros_mscl::SetGyroBias::Response &res);
    bool get_gyro_bias(ros_mscl::GetGyroBias::Request &req, ros_mscl::GetGyroBias::Response &res);

    bool gyro_bias_capture(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_hard_iron_values(ros_mscl::SetHardIronValues::Request &req, ros_mscl::SetHardIronValues::Response &res);
    bool get_hard_iron_values(ros_mscl::GetHardIronValues::Request &req, ros_mscl::GetHardIronValues::Response &res);

    bool set_soft_iron_matrix(ros_mscl::SetSoftIronMatrix::Request &req, ros_mscl::SetSoftIronMatrix::Response &res);
    bool get_soft_iron_matrix(ros_mscl::GetSoftIronMatrix::Request &req, ros_mscl::GetSoftIronMatrix::Response &res);

    bool set_complementary_filter(ros_mscl::SetComplementaryFilter::Request &req, ros_mscl::SetComplementaryFilter::Response &res);
    bool get_complementary_filter(ros_mscl::GetComplementaryFilter::Request &req, ros_mscl::GetComplementaryFilter::Response &res);

    bool set_coning_sculling_comp(ros_mscl::SetConingScullingComp::Request &req, ros_mscl::SetConingScullingComp::Response &res);
    bool get_coning_sculling_comp(ros_mscl::GetConingScullingComp::Request &req, ros_mscl::GetConingScullingComp::Response &res);

    bool set_sensor2vehicle_rotation(ros_mscl::SetSensor2VehicleRotation::Request &req, ros_mscl::SetSensor2VehicleRotation::Response &res);
    bool get_sensor2vehicle_rotation(ros_mscl::GetSensor2VehicleRotation::Request &req, ros_mscl::GetSensor2VehicleRotation::Response &res);

    bool set_sensor2vehicle_offset(ros_mscl::SetSensor2VehicleOffset::Request &req, ros_mscl::SetSensor2VehicleOffset::Response &res);
    bool get_sensor2vehicle_offset(ros_mscl::GetSensor2VehicleOffset::Request &req, ros_mscl::GetSensor2VehicleOffset::Response &res);

    bool get_sensor2vehicle_transformation(ros_mscl::GetSensor2VehicleTransformation::Request &req, ros_mscl::GetSensor2VehicleTransformation::Response &res);
      
    bool reset_filter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    bool init_filter_euler(ros_mscl::InitFilterEuler::Request &req, ros_mscl::InitFilterEuler::Response &res);
    bool init_filter_heading(ros_mscl::InitFilterHeading::Request &req, ros_mscl::InitFilterHeading::Response &res);
  
    bool set_heading_source(ros_mscl::SetHeadingSource::Request &req, ros_mscl::SetHeadingSource::Response &res);
    bool get_heading_source(ros_mscl::GetHeadingSource::Request &req, ros_mscl::GetHeadingSource::Response &res);

    bool set_reference_position(ros_mscl::SetReferencePosition::Request &req, ros_mscl::SetReferencePosition::Response &res);
    bool get_reference_position(ros_mscl::GetReferencePosition::Request &req, ros_mscl::GetReferencePosition::Response &res);

    bool set_estimation_control_flags(ros_mscl::SetEstimationControlFlags::Request &req, ros_mscl::SetEstimationControlFlags::Response &res);
    bool get_estimation_control_flags(ros_mscl::GetEstimationControlFlags::Request &req, ros_mscl::GetEstimationControlFlags::Response &res);

    bool set_dynamics_mode(ros_mscl::SetDynamicsMode::Request &req, ros_mscl::SetDynamicsMode::Response &res);
    bool get_dynamics_mode(ros_mscl::GetDynamicsMode::Request &req, ros_mscl::GetDynamicsMode::Response &res);

    bool set_zero_angle_update_threshold(ros_mscl::SetZeroAngleUpdateThreshold::Request &req, ros_mscl::SetZeroAngleUpdateThreshold::Response &res);
    bool get_zero_angle_update_threshold(ros_mscl::GetZeroAngleUpdateThreshold::Request &req, ros_mscl::GetZeroAngleUpdateThreshold::Response &res);
    
    bool set_zero_velocity_update_threshold(ros_mscl::SetZeroVelocityUpdateThreshold::Request &req, ros_mscl::SetZeroVelocityUpdateThreshold::Response &res);
    bool get_zero_velocity_update_threshold(ros_mscl::GetZeroVelocityUpdateThreshold::Request &req, ros_mscl::GetZeroVelocityUpdateThreshold::Response &res);

    bool set_tare_orientation(ros_mscl::SetTareOrientation::Request &req, ros_mscl::SetTareOrientation::Response &res);
    
    bool commanded_vel_zupt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool commanded_ang_rate_zupt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  
    bool set_accel_noise(ros_mscl::SetAccelNoise::Request &req, ros_mscl::SetAccelNoise::Response &res);
    bool get_accel_noise(ros_mscl::GetAccelNoise::Request &req, ros_mscl::GetAccelNoise::Response &res);

    bool set_gyro_noise(ros_mscl::SetGyroNoise::Request &req, ros_mscl::SetGyroNoise::Response &res);
    bool get_gyro_noise(ros_mscl::GetGyroNoise::Request &req, ros_mscl::GetGyroNoise::Response &res);

    bool set_mag_noise(ros_mscl::SetMagNoise::Request &req, ros_mscl::SetMagNoise::Response &res);
    bool get_mag_noise(ros_mscl::GetMagNoise::Request &req, ros_mscl::GetMagNoise::Response &res);

    bool set_gyro_bias_model(ros_mscl::SetGyroBiasModel::Request &req, ros_mscl::SetGyroBiasModel::Response &res);
    bool get_gyro_bias_model(ros_mscl::GetGyroBiasModel::Request &req, ros_mscl::GetGyroBiasModel::Response &res);

    bool set_accel_bias_model(ros_mscl::SetAccelBiasModel::Request &req, ros_mscl::SetAccelBiasModel::Response &res);
    bool get_accel_bias_model(ros_mscl::GetAccelBiasModel::Request &req, ros_mscl::GetAccelBiasModel::Response &res);

    bool set_gravity_adaptive_vals(ros_mscl::SetGravityAdaptiveVals::Request &req, ros_mscl::SetGravityAdaptiveVals::Response &res);
    bool get_gravity_adaptive_vals(ros_mscl::GetGravityAdaptiveVals::Request &req, ros_mscl::GetGravityAdaptiveVals::Response &res);

    bool set_mag_adaptive_vals(ros_mscl::SetMagAdaptiveVals::Request &req, ros_mscl::SetMagAdaptiveVals::Response &res);
    bool get_mag_adaptive_vals(ros_mscl::GetMagAdaptiveVals::Request &req, ros_mscl::GetMagAdaptiveVals::Response &res);

    bool set_mag_dip_adaptive_vals(ros_mscl::SetMagDipAdaptiveVals::Request &req, ros_mscl::SetMagDipAdaptiveVals::Response &res);
    bool get_mag_dip_adaptive_vals(ros_mscl::GetMagDipAdaptiveVals::Request &req, ros_mscl::GetMagDipAdaptiveVals::Response &res);
    
    bool external_heading_update(ros_mscl::ExternalHeadingUpdate::Request &req, ros_mscl::ExternalHeadingUpdate::Response &res);

    bool set_relative_position_reference(ros_mscl::SetRelativePositionReference::Request &req, ros_mscl::SetRelativePositionReference::Response &res);
    bool get_relative_position_reference(ros_mscl::GetRelativePositionReference::Request &req, ros_mscl::GetRelativePositionReference::Response &res);

    bool device_settings(ros_mscl::DeviceSettings::Request &req, ros_mscl::DeviceSettings::Response &res);

    void velocity_zupt_callback(const std_msgs::Bool& state);
    void vel_zupt();
    
    void ang_zupt_callback(const std_msgs::Bool& state);
    void ang_zupt();    

    void external_gps_time_callback(const sensor_msgs::TimeReference& time);

  private:


  //Convience for printing packet stats
  void print_packet_stats();

  //IMU Publishers
  ros::Publisher m_imu_pub;
  ros::Publisher m_mag_pub;
  ros::Publisher m_gps_corr_pub;

  //GNSS Publishers
  ros::Publisher m_gnss_pub[NUM_GNSS];
  ros::Publisher m_gnss_odom_pub[NUM_GNSS];
  ros::Publisher m_gnss_time_pub[NUM_GNSS];
  ros::Publisher m_gnss_aiding_status_pub[NUM_GNSS];
  ros::Publisher m_gnss_dual_antenna_status_pub;

  //RTK Data publisher
  ros::Publisher m_rtk_pub;
  
  //Filter Publishers
  ros::Publisher m_filter_status_pub;
  ros::Publisher m_filter_heading_pub;
  ros::Publisher m_filter_heading_state_pub;
  ros::Publisher m_filter_pub;
  ros::Publisher m_filtered_imu_pub;
  ros::Publisher m_filter_relative_pos_pub;

  //Device Status Publisher
  ros::Publisher m_device_status_pub;
   
  //ZUPT subscribers
  ros::Subscriber m_filter_vel_state_sub;
  ros::Subscriber m_filter_ang_state_sub;

  //External GNSS subscriber
  ros::Subscriber m_external_gps_time_sub;

  }; //Microstrain class


  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {

#ifdef __cplusplus
  }
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
