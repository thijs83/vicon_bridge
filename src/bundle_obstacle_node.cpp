
#include <ros/ros.h>

#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

std::vector<ros::Subscriber> obstacle_subs;
ros::Publisher bundled_object_pub;

derived_object_msgs::ObjectArray bundled_objects;

/** @todo via config */
int dynamic_obstacles_expected = 1;
int static_obstacles_expected = 0;
int obstacles_expected = dynamic_obstacles_expected + static_obstacles_expected;

// Count obstacles
int received_obstacles = 0;

void odometryToObject(nav_msgs::Odometry::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist = msg->twist.twist;
    object.header = msg->header;
}

void poseWithCovarianceStampedToObject(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist.linear.x = 0.;
    object.twist.linear.y = 0.;
    object.twist.linear.z = 0.;

    object.header = msg->header;
}

derived_object_msgs::Object &addOrGetObject(int id)
{
    for (auto &object : bundled_objects.objects)
    {
        if (id == object.id)
        {
            ROS_INFO("Vicon Bundle: Received existing object");
            return object;
        }
    }

    ROS_INFO("Vicon Bundle: Received new object");

    bundled_objects.objects.emplace_back();
    bundled_objects.objects.back().id = id;
    return bundled_objects.objects.back();
}

void increaseObstacleCount()
{
    received_obstacles++;

    if (received_obstacles == obstacles_expected)
    {
        ROS_INFO("Vicon Bundle: Publishing bundled objects");

        bundled_object_pub.publish(bundled_objects);

        received_obstacles = 0;
    }
}

void dynamicObstacleOdomCallback(int id, nav_msgs::Odometry::ConstPtr msg)
{
    auto &object = addOrGetObject(id); // Get the object with this ID (or make it)
    odometryToObject(msg, object);     // Convert the msg to an object msg type

    object.shape.dimensions.resize(2); // Add dimensions (for dynamic objects: disc with radius 0.4)
    object.shape.type = object.shape.CYLINDER;
    object.shape.dimensions[object.shape.CYLINDER_RADIUS] = 0.4;
    object.shape.dimensions[object.shape.CYLINDER_HEIGHT] = 2.2;

    increaseObstacleCount(); // We received one more object, publish if all objects were received

    // std::cout << "dynamic obstacle " << id << std::endl;
}

/** @see dynamicObstacleOdomCallback */
void staticObstacleOdomCallback(int id, geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    auto &object = addOrGetObject(id);
    poseWithCovarianceStampedToObject(msg, object);

    // Todo: Box
    object.shape.dimensions.resize(1);
    object.shape.dimensions[0] = 0.6;

    increaseObstacleCount();
    // std::cout << "static obstacle " << id << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_remap");

    auto nh = std::make_shared<ros::NodeHandle>("~");

    // Make a subscriber for all expected dynamic obstacles
    for (int i = 0; i < dynamic_obstacles_expected; i++)
    {
        obstacle_subs.push_back(
            nh->subscribe<nav_msgs::Odometry>("/dynamic_object" + std::to_string(i + 1) + "/odometry/filtered",
                                              1,
                                              std::bind(dynamicObstacleOdomCallback, i, std::placeholders::_1)));
    }

    // Make a subscriber for all expected static obstacles
    for (int i = 0; i < static_obstacles_expected; i++)
    {
        obstacle_subs.push_back(nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/vicon/static_object" + std::to_string(i + 1) + "_mapped_with_covariance", 1,
            std::bind(staticObstacleOdomCallback, i + dynamic_obstacles_expected, std::placeholders::_1)));
    }

    bundled_object_pub = nh->advertise<derived_object_msgs::ObjectArray>("/dynamic_objects", 1);

    ros::spin();
}