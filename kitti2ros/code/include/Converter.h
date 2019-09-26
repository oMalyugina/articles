//
// Created by olga on 23.09.19.
//

#ifndef ARTICLES_CONVERTER_H
#define ARTICLES_CONVERTER_H


#include <string>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <tf_conversions/tf_eigen.h>

using std::string;
using std::vector;

class PoseOxts {
public:
    sensor_msgs::Imu imu_msg;
//    sensor_msgs::NavSatFix gps_msg;

    PoseOxts(const string &line);
};

class KittiConverter {
    string pathToFolder;
    geometry_msgs::Transform calibrationImuToVelo;

    rosbag::Bag bag;

    std::vector<uint64_t> loadTimestampsIntoVector(const std::string &filename) const;

    std::string getFilenameForEntry(uint64_t frame) const;


public:
    explicit KittiConverter(string &inputPath, const string &outputPath) : pathToFolder(inputPath) {
        bag.open(outputPath, rosbag::bagmode::Write);
    };

    void convertImuAndGps();

    void convertPointCloud();

    ~KittiConverter() { bag.close(); }
};



#endif //ARTICLES_CONVERTER_H
