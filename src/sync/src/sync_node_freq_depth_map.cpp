#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/cache.h"



const char* lidar = "/os1_cloud_node/points";
const char* imu = "/os1_cloud_node/imu";
const char* camera0 = "/Camera/camera0_image";
const char* camera1 = "/Camera/camera1_image";
const char* depth_map = "/img_node/range_image";
const char* noise_img = "/img_node/noise_image";
const char* intensity_img = "/img_node/intensity_image";




using namespace message_filters;
using namespace sensor_msgs;


int main(int argc, char **argv){

    ros::init(argc, argv, "sync_node");

    ros::NodeHandle n("~");
    double freq;

    n.param<double>("freq", freq , 10.);

    ROS_INFO_STREAM("Sample rate : "<<freq);
    

    Subscriber<PointCloud2> lidar_sub (n,lidar  , 1);
    Subscriber<Image>       cam0_sub  (n,camera0, 1);
    Subscriber<Image>       cam1_sub  (n,camera1, 1);
    Subscriber<Image>       dpim_sub  (n,depth_map, 1);
    Subscriber<Image>       intim_sub (n,intensity_img, 1);
    Subscriber<Image>       noisim_sub(n,noise_img, 1);
    Subscriber<Imu>         imu_sub   (n,imu, 1);

    Cache<PointCloud2> lidar_cache  (lidar_sub, 1);
    Cache<Image>       cam0_cache   (cam0_sub,1);
    Cache<Image>       cam1_cache   (cam1_sub,1);
    Cache<Image>       dpim_cache   (dpim_sub,1);
    Cache<Image>       intim_cache  (intim_sub,1);
    Cache<Image>       noisim_cache (noisim_sub,1);
    Cache<Imu>         imu_cache    (imu_sub,1);

    ros::Publisher lidar_pub = n.advertise<PointCloud2>("lidar", 1);
    ros::Publisher cam0_pub = n.advertise<Image>("cam0", 1);
    ros::Publisher cam1_pub = n.advertise<Image>("cam1", 1);
    ros::Publisher dpim_pub = n.advertise<Image>("dpim", 1);
    ros::Publisher noisim_pub = n.advertise<Image>("noiseim", 1);
    ros::Publisher intim_pub = n.advertise<Image>("intensityim", 1);
    ros::Publisher imu_pub = n.advertise<Imu>("imu", 1);

    

    auto timerCallback = [&](const ros::TimerEvent & timestep){
        
        auto cam0_vec = cam0_cache.getInterval (cam0_cache.getOldestTime(),cam0_cache.getLatestTime());
        auto cam1_vec = cam1_cache.getInterval (cam1_cache.getOldestTime(),cam1_cache.getLatestTime());
        auto lidar_vec = lidar_cache.getInterval (lidar_cache.getOldestTime(),lidar_cache.getLatestTime());
        auto dpim_vec = dpim_cache.getInterval (dpim_cache.getOldestTime(),dpim_cache.getLatestTime());

        auto intim_vec = intim_cache.getInterval (intim_cache.getOldestTime(),intim_cache.getLatestTime());
        auto noisim_vec = noisim_cache.getInterval (noisim_cache.getOldestTime(),noisim_cache.getLatestTime());

        auto imu_vec = imu_cache.getInterval (imu_cache.getOldestTime(),imu_cache.getLatestTime());
        

        PointCloud2 framedPoints;
        for(auto el : lidar_vec)
            framedPoints = *el;
        Image image_0;
        for(auto el : cam0_vec)
            image_0 = *el;
        Image image_1;
        for(auto el : cam1_vec)
            image_1 = *el;

        Image image_2;
        for(auto el : dpim_vec)
            image_2 = *el;

        Image image_3;
        for(auto el : intim_vec)
            image_3 = *el;

        Image image_4;
        for(auto el : noisim_vec)
            image_4 = *el;

        Imu imu_1;
        for(auto el : imu_vec)
            imu_1 = *el;


        cam0_pub.publish(image_0);
        cam1_pub.publish(image_1); 
        dpim_pub.publish(image_2);
        noisim_pub.publish(image_4);
        intim_pub.publish(image_3);
        imu_pub.publish(imu_1);
        framedPoints.header.frame_id = "/map";
        lidar_pub.publish(framedPoints); 
        
    };


    ros::Timer timer = n.createTimer(ros::Duration(1./freq), timerCallback);
    
    
    ros::spin();

    return 0;

}
