#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include "surfel_type.h"
#include <pcl_ros/point_cloud.h>

float threshold;

void confidenceThresholdCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO_STREAM("New threshold set to "<<msg->data);
  threshold = msg->data;
}


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

    threshold = 5.0f;
    float current_threshold = threshold;
    string cloud_name = "surfel_map.pcd";

    if (argc>1) {
        cloud_name = argv[1];
    }

    if (argc>2) {
        stringstream ss; ss<<argv[2]; ss>>threshold;
    }

    ros::init(argc, argv, "surfel_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/surfel_publisher/threshold", 1, confidenceThresholdCallback);
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/surfel_publisher/cloud_surfel", 1);
    ros::Publisher pub_rgb = n.advertise<sensor_msgs::PointCloud2>("/surfel_publisher/cloud_rgb", 1);
    ros::Rate loop_rate(2);

    pcl::PointCloud<SurfelType>::Ptr original_cloud( new pcl::PointCloud<SurfelType>);

    ROS_INFO_STREAM("Loading cloud from "<<cloud_name);
    pcl::PCDReader reader;
    reader.read (cloud_name, *original_cloud);
    ROS_INFO_STREAM("Cloud loaded.");

    std::pair<pcl::PointCloud<SurfelType>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds = thresholdCloud(original_cloud, threshold);

    while (ros::ok()){

        cout<<"Running"<<endl;
        if (current_threshold != threshold){
            current_threshold = threshold;
            clouds = thresholdCloud(original_cloud, threshold);
        }

        pub.publish(clouds.first);
        pub_rgb.publish(clouds.second);

        ros::spinOnce();
        loop_rate.sleep();
    }


}
