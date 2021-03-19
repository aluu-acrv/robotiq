#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

/**
 * @author Baris Balic & Amelia Luu
 * @date 06/09/19
 * @version 1.1 - uses tf frames to tranform the force readings.
 * @brief main: the force torque sensor is mounted to the ur10's last joint. this tf frame is transformed
 * to where the linishing roller's tcp point is, such that the force readings are with respect to the
 * roller's frame.
 */

class FT_Transformer
{
private:
  ros::NodeHandle* m_node;
  ros::Publisher ft_pub;
  ros::Subscriber sub;

  geometry_msgs::TransformStamped transformStamped;

public:
  /**
   * @brief Construct a new ft transformer object
   *
   * @param node
   * @param ft_parent_frame, defaults to fts_frame - where we want ft sensor readings from
   * @param ft_child_frame, defaults to tool0_controller - where the where the force torque readings originate from
   */
  FT_Transformer(ros::NodeHandle* node, std::string ft_parent_frame = "fts_frame",
                 std::string ft_child_frame = "tool0_controller");

  /**
   * @brief Destroy the ft transformer object
   *
   */
  ~FT_Transformer();

  /**
   * @brief Initialises the ROS subscriber and publisher
   *
   */
  void initialiseROS();

  /**
   * @brief Finds the transform from parent to child frame
   *
   * @param parent the original frame
   * @param child the frame we want it in
   * @return geometry_msgs::TransformStamped the transform between frames
   */
  geometry_msgs::TransformStamped getTransform(std::string parent, std::string child);

  /**
   * @brief The force torque data call back from original sensor.
   *
   * @param msg the input geometry_msgs::WrenchStamped message
   */
  void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};

FT_Transformer::FT_Transformer(ros::NodeHandle* node, std::string parentf, std::string childf) : m_node(node)
{
  ROS_INFO_STREAM("Attempting to get the transform from " << parentf << " to " << childf);
  transformStamped = getTransform(parentf, childf);

  ROS_INFO("Got it! Initialising ROS...");
  initialiseROS();

  ROS_INFO("Done");
}

FT_Transformer::~FT_Transformer()
{
}

void FT_Transformer::initialiseROS()
{
  ft_pub = m_node->advertise<geometry_msgs::WrenchStamped>("/ft_transformed", 1);
  sub = m_node->subscribe("/robotiq_ft_wrench", 1, &FT_Transformer::callback, this);
}

geometry_msgs::TransformStamped FT_Transformer::getTransform(std::string parent, std::string child)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped t;
  bool success;
  while (!success)
  {
    try
    {
      t = tfBuffer.lookupTransform(parent, child, ros::Time(0));
      ROS_INFO_STREAM("The transform from " << parent << " to " << child << " is: \n" << t);
      return t;
    }

    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      ros::Duration(0.5).sleep();
    }
  }
}

void FT_Transformer::callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  // do the transform and publish it
  ROS_INFO_STREAM_ONCE("Got first geometry_msgs::WrenchStamped message! \n" << *msg);
  geometry_msgs::WrenchStamped transformed_ft_msg;
  tf2::doTransform(*msg, transformed_ft_msg, transformStamped);
  ROS_INFO_STREAM_ONCE("Transformed it to: \n" << transformed_ft_msg);
  ft_pub.publish(transformed_ft_msg);
  ROS_INFO_STREAM_ONCE("Published transformed data");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "force_torque_transformer");

  ros::NodeHandle nh;

  try
  {
    if (argc == 1)
    {
      ROS_INFO_STREAM("Using default frame names.");
      FT_Transformer ftt(&nh);
    }
    else if (argc == 3)
    {
      ROS_INFO_STREAM("Using specificed frame names.");
      ROS_INFO_STREAM("Parent is /" << argv[1] << " and child is /" << argv[2]);
      FT_Transformer ftt(&nh, argv[1], argv[2]);
    }
    else
    {
      std::stringstream err;
      err << "You did not specify the right arguments.\nEither use defaults by typing: rosrun robotiq_ft_sensor "
             "ft_transform, or\n try: rosrun robotiq_ft_sensor ft_transform <parent_frame> <child_frame>";
      throw std::invalid_argument(err.str());
    }
    ros::spin();
  }
  catch (const std::invalid_argument& e)
  {
    ROS_ERROR_STREAM("Invalid Argument: " << e.what());
    ros::shutdown();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
    ros::shutdown();
  }

  return 0;
};