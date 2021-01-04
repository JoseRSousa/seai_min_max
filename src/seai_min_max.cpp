#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

typedef struct 
{
  ros::Duration roll_stamp;
  ros::Duration pitch_stamp;
  ros::Duration yaw_stamp;
  tfScalar roll;
  tfScalar pitch;
  tfScalar yaw;
} euler_stamped;

ros::Time zero(0);

ros::Time start_time_mavros = zero;
ros::Time start_time_qualisys = zero;

euler_stamped mavros_min = {};
euler_stamped mavros_max = {};

euler_stamped qualisys_min = {};
euler_stamped qualisys_max = {};


/*
    Qualisys
    Calcular o minimo e máximo para roll, pitch e yaw, com a duração desde a primeira medição
*/
void calcMinMax_Qualisys(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

  if(start_time_qualisys == zero)
  {
    start_time_qualisys = msg->header.stamp;

    qualisys_min.roll_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_min.roll = roll;

    qualisys_min.pitch_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_min.pitch = pitch;

    qualisys_min.yaw_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_min.yaw = yaw;

    qualisys_max.roll_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_max.roll = roll;
    
    qualisys_max.pitch_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_max.pitch = pitch;

    qualisys_max.yaw_stamp = msg->header.stamp - start_time_qualisys;
    qualisys_max.yaw = yaw;

  }
  else
  {
    if(qualisys_max.roll < roll)
    {
      qualisys_max.roll = roll;
      qualisys_max.roll_stamp = msg->header.stamp - start_time_qualisys;
    }
    if(qualisys_max.pitch < pitch)
    {
      qualisys_max.pitch = pitch;
      qualisys_max.pitch_stamp = msg->header.stamp - start_time_qualisys;
    }
    if(qualisys_max.yaw < yaw)
    {
      qualisys_max.yaw = yaw;
      qualisys_max.yaw_stamp = msg->header.stamp - start_time_qualisys;
    }

    if(qualisys_min.roll > roll)
    {
      qualisys_min.roll = roll;
      qualisys_min.roll_stamp = msg->header.stamp - start_time_qualisys;
    }
    if(qualisys_min.pitch > pitch)
    {
      qualisys_min.pitch = pitch;
      qualisys_min.pitch_stamp = msg->header.stamp - start_time_qualisys;
    }
    if(qualisys_min.yaw > yaw)
    {
      qualisys_min.yaw = yaw;
      qualisys_min.yaw_stamp = msg->header.stamp - start_time_qualisys;
    }
  }


}

/*
    Mavros
    Calcular o minimo e máximo para roll, pitch e yaw, com a duração desde a primeira medição
*/
void calcMinMax_Mavros(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

  if(start_time_mavros == zero)
  {
    start_time_mavros = msg->header.stamp;

    mavros_min.roll_stamp = msg->header.stamp - start_time_mavros;
    mavros_min.roll = roll;

    mavros_min.pitch_stamp = msg->header.stamp - start_time_mavros;
    mavros_min.pitch = pitch;

    mavros_min.yaw_stamp = msg->header.stamp - start_time_mavros;
    mavros_min.yaw = yaw;

    mavros_max.roll_stamp = msg->header.stamp - start_time_mavros;
    mavros_max.roll = roll;
    
    mavros_max.pitch_stamp = msg->header.stamp - start_time_mavros;
    mavros_max.pitch = pitch;

    mavros_max.yaw_stamp = msg->header.stamp - start_time_mavros;
    mavros_max.yaw = yaw;
    
  }
  else
  {
    if(mavros_max.roll < roll)
    {
      mavros_max.roll = roll;
      mavros_max.roll_stamp = msg->header.stamp - start_time_mavros;
    }
    if(mavros_max.pitch < pitch)
    {
      mavros_max.pitch = pitch;
      mavros_max.pitch_stamp = msg->header.stamp - start_time_mavros;
    }
    if(mavros_max.yaw < yaw)
    {
      mavros_max.yaw = yaw;
      mavros_max.yaw_stamp = msg->header.stamp - start_time_mavros;
    }

    if(mavros_min.roll > roll)
    {
      mavros_min.roll = roll;
      mavros_min.roll_stamp = msg->header.stamp - start_time_mavros;
    }
    if(mavros_min.pitch > pitch)
    {
      mavros_min.pitch = pitch;
      mavros_min.pitch_stamp = msg->header.stamp - start_time_mavros;
    }
    if(mavros_min.yaw > yaw)
    {
      mavros_min.yaw = yaw;
      mavros_min.yaw_stamp = msg->header.stamp - start_time_mavros;
    }
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seai_min_max");

  ros::NodeHandle nMavros;
  ros::NodeHandle nQualisys;

  ros::Subscriber subMavros = nMavros.subscribe("/mavros/imu/data", 1000, calcMinMax_Mavros);
  ros::Subscriber subQualisys = nQualisys.subscribe("/qualisys/ROV/odom", 1000, calcMinMax_Qualisys);

  ros::spin();
  
  // Imprimir os resultados quando fecharmos este node
  std::cout << "\n";
  std::cout << "Max Mavros Roll: " << mavros_max.roll << " at " << mavros_max.roll_stamp.toSec() << "\n";
  std::cout << "Max Mavros Pitch: " << mavros_max.pitch << " at " << mavros_max.pitch_stamp.toSec() << "\n";
  std::cout << "Max Mavros Yaw: " << mavros_max.yaw << " at " << mavros_max.yaw_stamp.toSec() << "\n";
  std::cout << "Min Mavros Roll: " << mavros_min.roll << " at " << mavros_min.roll_stamp.toSec() << "\n";
  std::cout << "Min Mavros Pitch: " << mavros_min.pitch << " at " << mavros_min.pitch_stamp.toSec() << "\n";
  std::cout << "Min Mavros Yaw: " << mavros_min.yaw << " at " << mavros_min.yaw_stamp.toSec() << "\n";

  std::cout << "Max Qualisys Roll: " << qualisys_max.roll << " at " << qualisys_max.roll_stamp.toSec() << "\n";
  std::cout << "Max Qualisys Pitch: " << qualisys_max.pitch << " at " << qualisys_max.pitch_stamp.toSec() << "\n";
  std::cout << "Max Qualisys Yaw: " << qualisys_max.yaw << " at " << qualisys_max.yaw_stamp.toSec() << "\n";
  std::cout << "Min Qualisys Roll: " << qualisys_min.roll << " at " << qualisys_min.roll_stamp.toSec() << "\n";
  std::cout << "Min Qualisys Pitch: " << qualisys_min.pitch << " at " << qualisys_min.pitch_stamp.toSec() << "\n";
  std::cout << "Min Qualisys Yaw: " << qualisys_min.yaw << " at " << qualisys_min.yaw_stamp.toSec() << "\n";

  std::cout << "\n\n\n";
  std::cout << "Time difference (Mavros - Qualisys) in s:\n";
  std::cout << "Time difference for Max Roll: " << mavros_max.roll_stamp.toSec() - qualisys_max.roll_stamp.toSec() << "\n";
  std::cout << "Time difference for Max Pitch: " << mavros_max.pitch_stamp.toSec() - qualisys_max.pitch_stamp.toSec() << "\n";
  std::cout << "Time difference for Max Yaw: " << mavros_max.yaw_stamp.toSec() - qualisys_max.yaw_stamp.toSec() << "\n";
  std::cout << "Time difference for Min Roll: " << mavros_min.roll_stamp.toSec() - qualisys_min.roll_stamp.toSec() << "\n";
  std::cout << "Time difference for Min Pitch: " << mavros_min.pitch_stamp.toSec() - qualisys_min.pitch_stamp.toSec() << "\n";
  std::cout << "Time difference for Min Yaw: " << mavros_min.yaw_stamp.toSec() - qualisys_min.yaw_stamp.toSec() << "\n";

  return 0;
}