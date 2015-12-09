#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include "surfel_type.h"
#include <pcl_ros/point_cloud.h>

std::pair<pcl::PointCloud<SurfelType>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> thresholdCloud(pcl::PointCloud<SurfelType>::Ptr input, float threshold){

    std::pair<pcl::PointCloud<SurfelType>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    clouds.first = pcl::PointCloud<SurfelType>::Ptr(new pcl::PointCloud<SurfelType>);
    clouds.second = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB temp_point;
    for (auto surfel : input->points){
        if (surfel.confidence > threshold){
            clouds.first->push_back(surfel);

            temp_point.x = surfel.x;
            temp_point.y = surfel.y;
            temp_point.z = surfel.z;
            temp_point.rgba = surfel.rgba;
            clouds.second->push_back(temp_point);
        }
    }

    clouds.first->header.frame_id="/map";
    clouds.second->header.frame_id="/map";
    return clouds;
}

int main(int argc, char **argv)
{
    using namespace std;

    ros::init(argc, argv, "surfel_exporter");
    ros::NodeHandle n;

    double threshold;
    string cloud_name;

    ros::NodeHandle pn("~");
    pn.param<double>("threshold", threshold, 5.0);
    pn.param<std::string>("cloud_name", cloud_name, "surfel_map.pcd");
    //pn.param<std::string>("vocabulary_path", string_path, std::string(""));

    pcl::PointCloud<SurfelType>::Ptr original_cloud( new pcl::PointCloud<SurfelType>);

    ROS_INFO_STREAM("Loading cloud from "<<cloud_name);
    pcl::PCDReader reader;
    reader.read (cloud_name, *original_cloud);
    ROS_INFO_STREAM("Cloud loaded.");

    std::pair<pcl::PointCloud<SurfelType>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds = thresholdCloud(original_cloud, threshold);

    string new_name = (boost::filesystem::path(cloud_name).parent_path() / "point_map.pcd").string();
    pcl::io::savePCDFileBinary(new_name, *clouds.second);

    ROS_INFO_STREAM("Wrote cloud "<<new_name);

    return 0;
}
