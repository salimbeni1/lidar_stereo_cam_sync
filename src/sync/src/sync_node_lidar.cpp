#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/cache.h"



const char* lidar = "/os1_cloud_node/points";
const char* camera0 = "/Camera/camera0_image";
const char* camera1 = "/Camera/camera1_image";

using namespace message_filters;
using namespace sensor_msgs;


int main(int argc, char **argv){

    ros::init(argc, argv, "sync_node");

    ros::NodeHandle n;
    

    Subscriber<PointCloud2> lidar_sub (n,lidar  , 1);
    Subscriber<Image>       cam0_sub  (n,camera0, 1);
    Subscriber<Image>       cam1_sub  (n,camera1, 1);

    Cache<PointCloud2> lidar_cache (lidar_sub, 1);
    Cache<Image>       cam0_cache  (cam0_sub,1);
    Cache<Image>       cam1_cache  (cam1_sub,1);

    ros::Publisher lidar_pub = n.advertise<PointCloud2>("lidar", 1);
    ros::Publisher cam0_pub = n.advertise<Image>("cam0", 1);
    ros::Publisher cam1_pub = n.advertise<Image>("cam1", 1);

    auto callback_sync = [&](const PointCloud2::ConstPtr& lidar){
        
        auto cam0_vec = cam0_cache.getInterval (cam0_cache.getOldestTime(),cam0_cache.getLatestTime());
        auto cam1_vec = cam0_cache.getInterval (cam1_cache.getOldestTime(),cam1_cache.getLatestTime());
        
        Image image_0;
        for(auto el : cam0_vec)
            image_0 = *el;
        Image image_1;
        for(auto el : cam1_vec)
            image_1 = *el;

        cam0_pub.publish(image_0);
        cam1_pub.publish(image_1); 

        PointCloud2 framedPoints(*lidar);
        framedPoints.header.frame_id = "/map";
        lidar_pub.publish(framedPoints); 
        
    };

    lidar_cache.registerCallback(callback_sync);
    
    
    ros::spin();

    return 0;

}





/*
    ros::Publisher pub = n.advertise<PointCloud2>("lidar", 5);
    ros::Publisher pub0 = n.advertise<Image>("cam0", 5);
    ros::Publisher pub1 = n.advertise<Image>("cam1", 5);

    auto callbackL = [&](const PointCloud2::ConstPtr& msg){
        PointCloud2 framedPoints(*msg);
        framedPoints.header.frame_id = "/map";
        pub.publish(framedPoints);
        };

    auto callbackC0 = [&](const Image::ConstPtr& msg){
        pub0.publish(*msg);
        };
    
    auto callbackC1 = [&](const Image::ConstPtr& msg){
        pub1.publish(*msg);
        };

    ros::Subscriber sub = n.subscribe<PointCloud2>(lidar, 10, callbackL);
    ros::Subscriber sub0 = n.subscribe<Image>(camera0, 10, callbackC0);
    ros::Subscriber sub1 = n.subscribe<Image>(camera0, 10, callbackC1);
    */