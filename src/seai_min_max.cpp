#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
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

ros::Time start_time = zero;

euler_stamped min = {};
euler_stamped max = {};


/*
    Calcular o minimo e máximo para roll, pitch e yaw, com a duração desde a primeira medição
*/
void calcMinMax(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

  if(start_time == zero)
  {
    start_time = msg->header.stamp;

    min.roll_stamp = msg->header.stamp - start_time;
    min.roll = roll;

    min.roll_stamp = msg->header.stamp - start_time;
    min.pitch = pitch;

    min.roll_stamp = msg->header.stamp - start_time;
    min.yaw = yaw;

    max.roll_stamp = msg->header.stamp - start_time;
    max.roll = roll;
    
    max.pitch_stamp = msg->header.stamp - start_time;
    max.pitch = pitch;

    max.yaw_stamp = msg->header.stamp - start_time;
    max.yaw = yaw;
    
  }
  else
  {
    if(max.roll < roll)
    {
      max.roll = roll;
      max.roll_stamp = msg->header.stamp - start_time;
    }
    if(max.pitch < pitch)
    {
      max.pitch = pitch;
      max.pitch_stamp = msg->header.stamp - start_time;
    }
    if(max.yaw < yaw)
    {
      max.yaw = yaw;
      max.yaw_stamp = msg->header.stamp - start_time;
    }

    if(min.roll > roll)
    {
      min.roll = roll;
      min.roll_stamp = msg->header.stamp - start_time;
    }
    if(min.pitch > pitch)
    {
      min.pitch = pitch;
      min.pitch_stamp = msg->header.stamp - start_time;
    }
    if(min.yaw > yaw)
    {
      min.yaw = yaw;
      min.yaw_stamp = msg->header.stamp - start_time;
    }
  }
  
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called calcMinMax.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, calcMinMax);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  
  // Imprimir os resultados quando fecharmos este node
  std::cout << "\n";
  std::cout << "Max Roll: " << max.roll << " at " << max.roll_stamp.toSec() << "\n";
  std::cout << "Max Pitch: " << max.pitch << " at " << max.pitch_stamp.toSec() << "\n";
  std::cout << "Max Yaw: " << max.yaw << " at " << max.yaw_stamp.toSec() << "\n";
  std::cout << "Min Roll: " << min.roll << " at " << min.roll_stamp.toSec() << "\n";
  std::cout << "Min Pitch: " << min.pitch << " at " << min.pitch_stamp.toSec() << "\n";
  std::cout << "Min Yaw: " << min.yaw << " at " << min.yaw_stamp.toSec() << "\n";

  return 0;
}