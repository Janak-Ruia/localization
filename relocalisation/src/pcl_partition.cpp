#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <geometry_msgs/Point.h>
class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_map", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_target", 1);
        pcl_pose = nh.advertise<geometry_msgs::Point>("center_point", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);
        
        sensor_msgs::PointCloud2 output;
        geometry_msgs::Point center_msg;
        
        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = 0 ;
        center_point.y = -1;
        center_point.z = 60; // Just for testing.
        // center_point.z = 50;
        //pcl::toROSMsg(center_point, center_msg);
        center_msg.x = center_point.x;
        center_msg.y = center_point.y;
        center_msg.z = center_point.z;
        pcl_pose.publish(center_msg);

        float radius = 40;
        std::vector<int> radiusIdx;
        std::vector<float> radiusSQDist;
        if (octree.radiusSearch (center_point, radius, radiusIdx, radiusSQDist) > 0)
        {
            for (size_t i = 0; i < radiusIdx.size (); ++i)
            {
                cloud_partitioned.points.push_back(cloud.points[radiusIdx[i]]);
            }
        }

        pcl::toROSMsg(cloud_partitioned, output);

        output.header.frame_id = "/camera_init";
        pcl_pub.publish(output);

    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pose;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_partition");

    cloudHandler handler;

    ros::spin();

    return 0;
}
