//
// Created by larr-planning on 24. 8. 19.
//
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 clickMap_pcd;

pcl::PointCloud<pcl::PointXYZ> cloudMap;

void ReadObstacleConfiguration(std::string &file_name){
    std::ifstream obstacle_file;
    obstacle_file.open(file_name.c_str());
    double v1[2]{0.0, 0.0};
    double v2[2]{0.0, 0.0};
    double min_x, min_y, max_x, max_y;
    double p1[2], p2[2], p3[2], p4[2];
    int num_point;
    double inflation_size_ = 0.5;
    double point_resolution_ = 0.2;
    /*
     *  p1 --------p2
     *  -          -
     *  -          -
     *  p3---------p4
     */
    pcl::PointXYZ temp_point;
    int id = 0;
    if (obstacle_file.is_open()) {
        while (obstacle_file >> v1[0] >> v1[1] >> v2[0] >> v2[1]) {
            if (v1[0] <= v2[0]) {
                min_x = v1[0];
                max_x = v2[0];
            } else {
                min_x = v2[0];
                max_x = v1[0];
            }
            if (v1[1] <= v2[1]) {
                min_y = v1[1];
                max_y = v2[1];
            } else {
                min_y = v2[1];
                max_y = v1[1];
            }
            p1[0] = min_x - inflation_size_;
            p1[1] = max_y + inflation_size_;
            p2[0] = max_x + inflation_size_;
            p2[1] = max_y + inflation_size_;
            p3[0] = min_x - inflation_size_;
            p3[1] = min_y - inflation_size_;
            p4[0] = max_x + inflation_size_;
            p4[1] = min_y - inflation_size_;
            // p1 - p2
            num_point = floor((p2[0] - p1[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p2[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p2[1] - p1[1]) / num_point * i);
                for(int j=0;j<10;j++){
                    temp_point.z = point_resolution_*j;
                    cloudMap.push_back(temp_point);
                }
            }
            //p1 - p3
            num_point = floor((p1[1] - p3[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p3[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p3[1] - p1[1]) / num_point * i);
                for(int j=0;j<10;j++){
                    temp_point.z = point_resolution_*j;
                    cloudMap.push_back(temp_point);
                }
            }
            // p3 - p4
            num_point = floor((p4[0] - p3[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p3[0] + (p4[0] - p3[0]) / num_point * i);
                temp_point.y = float(p3[1] + (p4[1] - p3[1]) / num_point * i);
                for(int j=0;j<10;j++){
                    temp_point.z = point_resolution_*j;
                    cloudMap.push_back(temp_point);
                }
            }
            // p4 - p2
            num_point = floor((p2[1] - p4[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p4[0] + (p2[0] - p4[0]) / num_point * i);
                temp_point.y = float(p4[1] + (p2[1] - p4[1]) / num_point * i);
                for(int j=0;j<10;j++){
                    temp_point.z = point_resolution_*j;
                    cloudMap.push_back(temp_point);
                }
            }
        }
    }
    obstacle_file.close();
    {
        p1[0] = 0.0;
        p1[1] = 15.0;
        p2[0] = 15.0;
        p2[1] = 15.0;
        p3[0] = 0.0;
        p3[1] = 0.0;
        p4[0] = 15.0;
        p4[1] = 0.0;
        // p1 - p2
        num_point = floor((p2[0] - p1[0]) / point_resolution_ + 0.5);
        for (int i = 0; i <= num_point; i++) {
            temp_point.x = float(p1[0] + (p2[0] - p1[0]) / num_point * i);
            temp_point.y = float(p1[1] + (p2[1] - p1[1]) / num_point * i);
            for(int j=0;j<30;j++){
                temp_point.z = point_resolution_*j;
                cloudMap.push_back(temp_point);
            }
        }
        //p1 - p3
        num_point = floor((p1[1] - p3[1]) / point_resolution_ + 0.5);
        for (int i = 0; i <= num_point; i++) {
            temp_point.x = float(p1[0] + (p3[0] - p1[0]) / num_point * i);
            temp_point.y = float(p1[1] + (p3[1] - p1[1]) / num_point * i);
            for(int j=0;j<30;j++){
                temp_point.z = point_resolution_*j;
                cloudMap.push_back(temp_point);
            }
        }
        // p3 - p4
        num_point = floor((p4[0] - p3[0]) / point_resolution_ + 0.5);
        for (int i = 0; i <= num_point; i++) {
            temp_point.x = float(p3[0] + (p4[0] - p3[0]) / num_point * i);
            temp_point.y = float(p3[1] + (p4[1] - p3[1]) / num_point * i);
            for(int j=0;j<30;j++){
                temp_point.z = point_resolution_*j;
                cloudMap.push_back(temp_point);
            }
        }
        // p4 - p2
        num_point = floor((p2[1] - p4[1]) / point_resolution_ + 0.5);
        for (int i = 0; i <= num_point; i++) {
            temp_point.x = float(p4[0] + (p2[0] - p4[0]) / num_point * i);
            temp_point.y = float(p4[1] + (p2[1] - p4[1]) / num_point * i);
            for(int j=0;j<30;j++){
                temp_point.z = point_resolution_*j;
                cloudMap.push_back(temp_point);
            }
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv, "prior_map_publisher");
    ros::NodeHandle nh("~");

    std::string obstacle_configuration_file_name;
    nh.param<std::string>("obstacle_configuration_file_name",obstacle_configuration_file_name,"");

    ros::Publisher global_map_publisher;
    ros::Publisher local_map_publisher;
    ros::Publisher click_map_publisher;

    global_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
    local_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
    click_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render/node/local_map",1);

    ReadObstacleConfiguration(obstacle_configuration_file_name);

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    pcl::toROSMsg(cloudMap, localMap_pcd);
    pcl::toROSMsg(cloudMap, clickMap_pcd);

    globalMap_pcd.header.frame_id = "world";
    localMap_pcd.header.frame_id = "world";
    clickMap_pcd.header.frame_id= "world";

    ros::Rate loop_rate(20.0);
    while(ros::ok()){
        global_map_publisher.publish(globalMap_pcd);
        local_map_publisher.publish(localMap_pcd);
        click_map_publisher.publish(clickMap_pcd);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}