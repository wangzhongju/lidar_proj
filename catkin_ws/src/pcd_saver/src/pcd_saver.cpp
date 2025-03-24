#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/filters/statistical_outlier_removal.h>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    // Convert the ROS PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*cloud_msg, cloud);

    // // Remove NaN values from the PointCloud
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    
    // Save the PointCloud to a PCD file
    if (pcl::io::savePCDFileASCII("/workspace/data/pointcloud.pcd", cloud) == -1)
    {
        PCL_ERROR("Couldn't save PCD file\n");
        return;
    }
    
    ROS_INFO("PointCloud saved as /workspace/data/pointcloud.pcd");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_saver");
    ros::NodeHandle nh;

    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe("/33/rslidar_points", 1, pointCloudCallback);

    ros::spin();
    
    return 0;
}
