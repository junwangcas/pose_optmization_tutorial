#ifndef BEGINNER_TUTORIALS_LAND_MARK_VIS_H
#define BEGINNER_TUTORIALS_LAND_MARK_VIS_H
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include "sensor_msgs/PointCloud.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class LandMarkVis {
public:
    std::string m_marker_ns;
    LandMarkVis(float r, float g, float b, float a);
    void setScale(double s);
    void AddPoints(const std::vector<Eigen::Vector3d> &pts);
    void reset();
    void publish_by(ros::Publisher& pub, const std_msgs::Header& header);
private:
    void Eigen2PtsCloud(const std::vector<Eigen::Vector3d> &pts, sensor_msgs::PointCloud &ptsMsg);
    sensor_msgs::PointCloud ptsMsg_;
};


#endif //BEGINNER_TUTORIALS_CAMERA_POSE_VIS_H
