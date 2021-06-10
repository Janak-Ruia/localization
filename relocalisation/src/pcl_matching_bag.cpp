#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

pcl::PointCloud<pcl::PointXYZ> cloud_scan;
pcl::PointCloud<pcl::PointXYZ> cloud_targ;
pcl::PointCloud<pcl::PointXYZ> cloud_aligned;

PointMatcher<float>::DataPoints scene;
PointMatcher<float>::DataPoints object;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_targ = nh.subscribe("pcl_target", 10, &cloudHandler::cloudCB_targ, this);
        pcl_sub_scan = nh.subscribe("segmented_cloud_pure", 10, &cloudHandler::cloudCB_scan, this);
        pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);
    }

    void cloudCB_targ(const sensor_msgs::PointCloud2 &input)
    {
        
        pcl::fromROSMsg(input, cloud_targ);
        scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input, false);

    }

    void cloudCB_scan(const sensor_msgs::PointCloud2 &input)
    {
        object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input, false);
        // sensor_msgs::PointCloud2 output;

        // pcl::fromROSMsg(input, cloud_scan);

        // for (size_t i = 0; i < cloud_scan.points.size (); ++i)
        // {
        //     cloud_scan.points[i].x = cloud_scan.points[i].x + 10;
        //     // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        // }

        if ((cloud_scan.points.size()>0)){ //&& (cloud_targ.points.size()>0)){
            cloudHandler::perform_icp();
        }
    }

    void perform_icp()        
    {   
        // sensor_msgs::PointCloud2 output;

        // Libpointmatcher
        sensor_msgs::PointCloud2 scene_Cloud_libpointmatcher;
        PointMatcher<float>::DataPoints scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scene_Cloud_libpointmatcher, false);
        sensor_msgs::PointCloud2 obj_Cloud_libpointmatcher;
        PointMatcher<float>::DataPoints object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(obj_Cloud_libpointmatcher, false);

        PointMatcher<float>::ICP icp;
        icp.setDefault();
        PointMatcher<float>::TransformationParameters T = icp(object, scene);

        std::cout << "Transformation Matrix = \n" << T << std::endl;
        PointMatcher<float>::DataPoints transformed_object(object);
        icp.transformations.apply(transformed_object, T);

        sensor_msgs::PointCloud2 transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformed_object, "/camera_frame_id", ros::Time::now());
        pcl_pub_aligned.publish(transformed_pcd);
        /////////////////////////////////////////////

        /*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_scan.makeShared());
        icp.setInputTarget(cloud_targ.makeShared());
 
        icp.setMaxCorrespondenceDistance(30);
        icp.setMaximumIterations(10000);
        icp.setTransformationEpsilon (1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(cloud_aligned);

        pcl::toROSMsg(cloud_aligned, output);
        pcl_pub_aligned.publish(output);*/
    } 

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_aligned;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching");

    cloudHandler handler;
    ROS_INFO("\033[1;32m---->\033[0m pcl_matching started.");
    ros::spin();

    return 0;
}