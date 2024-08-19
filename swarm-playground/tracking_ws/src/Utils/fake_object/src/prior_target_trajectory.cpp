//
// Created by larr-planning on 24. 8. 19.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <istream>
#include <fstream>

using namespace std;
struct history_list {
    vector<double> t;
    vector<double> px;
    vector<double> py;
    vector<double> pz;
    vector<double> vx;
    vector<double> vy;
    vector<double> vz;
};
history_list target_history;

void ReadTargetTrajectory(const std::string &file_name) {
    ifstream object_trajectory_file;
    object_trajectory_file.open(file_name.c_str());
    string line, word;
    vector <string> row;
    vector <vector<string>> content;
    if (object_trajectory_file.is_open()) {
        while (getline(object_trajectory_file, line)) {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
    } else
        printf("UNABLE TO READ TARGET TRAJECTORY HISTORY\n");
    for (int j = 0; j < content.size(); j++) {
        target_history.t.push_back(stod(content[j][1]));   // t
        target_history.px.push_back(stod(content[j][2]));   // px
        target_history.py.push_back(stod(content[j][3]));   // py
        target_history.pz.push_back(stod(content[j][4]));   // pz
        target_history.vx.push_back(stod(content[j][5]));   // vx
        target_history.vy.push_back(stod(content[j][6]));   // vy
        target_history.vz.push_back(stod(content[j][7]));   // vz
    }
}

nav_msgs::Odometry target_odometry;

double interpolate(const std::vector<double> &xData, const std::vector<double> &yData, const double &x) {
    int size = (int) xData.size();
    if (xData.size() < 2)
        return yData[0];

    if (x < xData[0])
        return yData[0];

    if (x > xData[size - 1])
        return yData.back();

    double xL = xData[0], yL = yData[0], xR = xData.back(), yR = yData.back();

    for (int i = 0; i < size - 2; i++) {
        if (xData[i] <= x and x < xData[i + 1]) {
            xL = xData[i];
            yL = yData[i];
            xR = xData[i + 1], yR = yData[i + 1];
            break;
        }
    }
    double dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
}

void UpdateDynamics(const double &t) {
    target_odometry.header.frame_id = "world";
    target_odometry.pose.pose.orientation.w = 1.0;
    target_odometry.pose.pose.orientation.x = 0.0;
    target_odometry.pose.pose.orientation.y = 0.0;
    target_odometry.pose.pose.orientation.z = 0.0;

    target_odometry.pose.pose.position.x = interpolate(target_history.t, target_history.px, t);
    target_odometry.pose.pose.position.y = interpolate(target_history.t, target_history.py, t);
    target_odometry.pose.pose.position.z = interpolate(target_history.t, target_history.pz, t);
    target_odometry.twist.twist.linear.x = interpolate(target_history.t, target_history.vx, t);
    target_odometry.twist.twist.linear.y = interpolate(target_history.t, target_history.vy, t);
    target_odometry.twist.twist.linear.z = interpolate(target_history.t, target_history.vz, t);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prior_target_trajectory");
    ros::NodeHandle nh("~");

    ros::Publisher target_odom_publisher;
    target_odom_publisher = nh.advertise<nav_msgs::Odometry>("/object_odom", 1);
    std::string target_trajectory_file_name;
    nh.param<std::string>("target_trajectory_file_name", target_trajectory_file_name, "");
    ReadTargetTrajectory(target_trajectory_file_name);
    ros::Rate loop_rate(100.0);
    double t0 = ros::Time::now().toSec();
    while (ros::ok()) {
        if(ros::Time::now().toSec()-t0>3.0){
            static double t0_real =ros::Time::now().toSec();
            UpdateDynamics(ros::Time::now().toSec() - t0_real);
            target_odom_publisher.publish(target_odometry);
        }
        else{
            target_odometry.header.frame_id = "world";
            target_odometry.pose.pose.orientation.w = 1.0;
            target_odometry.pose.pose.orientation.x = 0.0;
            target_odometry.pose.pose.orientation.y = 0.0;
            target_odometry.pose.pose.orientation.z = 0.0;
            target_odometry.pose.pose.position.x = interpolate(target_history.t, target_history.px, 0);
            target_odometry.pose.pose.position.y = interpolate(target_history.t, target_history.py, 0);
            target_odometry.pose.pose.position.z = interpolate(target_history.t, target_history.pz, 0);
            target_odometry.twist.twist.linear.x = interpolate(target_history.t, target_history.vx, 0);
            target_odometry.twist.twist.linear.y = interpolate(target_history.t, target_history.vy, 0);
            target_odometry.twist.twist.linear.z = interpolate(target_history.t, target_history.vz, 0);
            target_odom_publisher.publish(target_odometry);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}