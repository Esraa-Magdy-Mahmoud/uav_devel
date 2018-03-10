#include "uav_devel/UavPC.h"

namespace uav_pc
{
    UavPC::UavPC(ros::NodeHandle& nodeHandle ):nodeHandle_(nodeHandle)
    {
        pcl_sub_ = nodeHandle_.subscribe("/camera/depth/points",1,&UavPC::pclCallback,this); 
        pcl_filtered_pub_= nodeHandle_.advertise<sensor_msgs::PointCloud2> ("/cloud_filtered", 1);
    }

    UavPC::~UavPC()
    {
    }
    void UavPC::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
    
        // Container for original & filtered data
         pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
         pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
         pcl::PCLPointCloud2 cloud_filtered;

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.1, 0.1, 0.1);
        sor.filter (cloud_filtered);

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_filtered, output);

        // Publish the data
        pcl_filtered_pub_.publish(output);


    }
}

