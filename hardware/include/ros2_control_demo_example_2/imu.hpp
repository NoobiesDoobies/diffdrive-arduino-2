#ifndef IMU_HPP
#define IMU_HPP

#include <string>
#include <cmath>

#include "sensor_msgs/msg/imu.hpp"

class Imu
{
    public:

    std::string name = "";
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;

    sensor_msgs::msg::Imu imu_msg;

    Imu() = default;

    Imu(const std::string &imu_name)
    {
      setup(imu_name);
    }

    
    void setup(const std::string &imu_name)
    {
      name = imu_name;
    }

    sensor_msgs::msg::Imu get_imu_msg()
    {
      imu_msg.orientation.x = roll;
      imu_msg.orientation.y = pitch;
      imu_msg.orientation.z = yaw;

      imu_msg.angular_velocity.x = gyro_x;
      imu_msg.angular_velocity.y = gyro_y;
      imu_msg.angular_velocity.z = gyro_z;

      imu_msg.linear_acceleration.x = accel_x;
      imu_msg.linear_acceleration.y = accel_y;
      imu_msg.linear_acceleration.z = accel_z;

      return imu_msg;
    }
};



#endif // IMU_HPP
