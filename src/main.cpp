#include <iostream>
#include <string>
#include <ros/ros.h>
#include "template_ws/point.h"
#include <dynamic_reconfigure/server.h>
#include "template_ws/template_wsConfig.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

namespace params{
    int start_index,end_index;
    int point_start,point_end;
    float min_x,min_y,min_z;
    float max_x,max_y,max_z;
}

void setParams(template_ws::template_wsConfig &config,uint32_t level){
    params::start_index = config.start_index;
    params::end_index = config.end_index;
    params::point_start = config.point_start;
    params::point_end = config.point_end;
    params::max_x = config.max_x;
    params::min_x = config.min_x;
    params::max_y = config.max_y;
    params::min_y = config.min_y;
    params::max_z = config.max_z;
    params::min_z = config.min_z;
}

bool comp_up(const ouster::Point& a,const ouster::Point& b){
    return a.y<b.y;
}

class DataTest{
    ros::Subscriber sub;
    ros::Publisher pub,pub_amb,pub_refl,pub_int,pub_z,pub_dist;

    int channel_number;

    void setHistogramData(const pcl::PointCloud<ouster::Point>& cloud,float max_a,float max_i,float max_r,float min_z){
        std_msgs::Float32MultiArray amb_data;
        std_msgs::Float32MultiArray int_data;
        std_msgs::Float32MultiArray refl_data;
        std_msgs::Float32MultiArray z_data;
        std_msgs::Float32MultiArray dist_data;
        
        amb_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

        amb_data.layout.dim[0].label = "ambient";
        amb_data.layout.dim[0].size = cloud.points.size();
        amb_data.layout.dim[0].stride = cloud.points.size();
        amb_data.layout.data_offset = 0;

        int_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

        int_data.layout.dim[0].label = "intensity";
        int_data.layout.dim[0].size = cloud.points.size();
        int_data.layout.dim[0].stride = cloud.points.size();
        int_data.layout.data_offset = 0;

        refl_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

        refl_data.layout.dim[0].label = "reflectivity";
        refl_data.layout.dim[0].size = cloud.points.size();
        refl_data.layout.dim[0].stride = cloud.points.size();
        refl_data.layout.data_offset = 0;

        z_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

        z_data.layout.dim[0].label = "Z";
        z_data.layout.dim[0].size = cloud.points.size();
        z_data.layout.dim[0].stride = cloud.points.size();
        z_data.layout.data_offset = 0;

        dist_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

        dist_data.layout.dim[0].label = "distance";
        dist_data.layout.dim[0].size = cloud.points.size();
        dist_data.layout.dim[0].stride = cloud.points.size();
        dist_data.layout.data_offset = 0;
        

        std::vector<float> amb_vec(cloud.points.size(),0);
        std::vector<float> int_vec(cloud.points.size(),0);
        std::vector<float> refl_vec(cloud.points.size(),0);
        std::vector<float> z_vec(cloud.points.size(),0);
        std::vector<float> dist_vec(cloud.points.size(),0);

        size_t cloud_size = cloud.points.size();
        for(size_t i=0;i<cloud_size;++i){
            amb_vec[i] = cloud.points[i].ambient/max_a;
            int_vec[i] = cloud.points[i].intensity/max_i;
            refl_vec[i] = cloud.points[i].reflectivity/max_r;
            z_vec[i] = cloud.points[i].z+fabs(min_z);
            dist_vec[i] = sqrt(pow(cloud.points[i].x,2)+pow(cloud.points[i].y,2));
        }
        amb_data.data = amb_vec;
        int_data.data = int_vec;
        refl_data.data = refl_vec;
        z_data.data = z_vec;
        dist_data.data = dist_vec;

        pub_amb.publish(amb_data);
        pub_int.publish(int_data);
        pub_refl.publish(refl_data);
        pub_z.publish(z_data);
        pub_dist.publish(dist_data);
    }

    void callback(const pcl::PointCloud<ouster::Point>& cloud){
        std::vector<pcl::PointCloud<ouster::Point>> converted_cloud(channel_number);
        pcl::PointCloud<ouster::Point> temp;
        pcl::PointCloud<ouster::Point> output;
        for(const auto& point:cloud){
            converted_cloud[point.ring].points.push_back(point);
        }
        size_t start = channel_number-1-params::start_index;
        size_t end = channel_number-1-params::end_index;
        float max_a=0,max_i=0,max_r=0,min_z=0;
        for(int i=start;i>end;i--){
            std::sort(converted_cloud[i].points.begin(),converted_cloud[i].points.end(),comp_up);
            for(int j=params::point_start;j<params::point_end;++j){
                auto current_point = converted_cloud[i].points[j];
                if((current_point.y>params::min_y && current_point.y<params::max_y) && 
                (current_point.x>params::min_x && current_point.x<params::max_x) && 
                (current_point.z>params::min_z && current_point.z<params::max_z)){
                    if(current_point.ambient>max_a)
                        max_a = current_point.ambient;
                    if(current_point.intensity>max_i)
                        max_i = current_point.intensity;
                    if(current_point.reflectivity>max_r)
                        max_r = current_point.reflectivity;
                    if(current_point.z<min_z)
                        min_z = current_point.z;
                    temp.points.push_back(current_point);
                }
            }
        }
        setHistogramData(temp,max_a,max_i,max_r,min_z);
        output.header = cloud.header;
        output.points = temp.points;
        pub.publish(output);
    }

    public:
    DataTest(ros::NodeHandlePtr nh){
        sub = nh->subscribe("/os_cloud_node/points",1,&DataTest::callback,this);

        nh->getParam("channel_number",channel_number);

        pub = nh->advertise<pcl::PointCloud<ouster::Point>>("output_points",1);
        pub_amb = nh->advertise<std_msgs::Float32MultiArray>("/ambient_data",1);
        pub_int = nh->advertise<std_msgs::Float32MultiArray>("/intensity_data",1);
        pub_refl = nh->advertise<std_msgs::Float32MultiArray>("/reflectivity_data",1);
        pub_z = nh->advertise<std_msgs::Float32MultiArray>("/z_data",1);
        pub_dist = nh->advertise<std_msgs::Float32MultiArray>("/distance_data",1);
    }
};

int main(int argc,char** argv){
    ros::init(argc,argv,"template_ws_node");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    dynamic_reconfigure::Server<template_ws::template_wsConfig> server;
    dynamic_reconfigure::Server<template_ws::template_wsConfig>::CallbackType f;

    f = boost::bind(&setParams,_1,_2);
    server.setCallback(f); 

    DataTest dt(nh);

    ros::spin();
}