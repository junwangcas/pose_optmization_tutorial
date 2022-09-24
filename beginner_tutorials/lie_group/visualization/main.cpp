#include "camera_pose_vis.h"
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include "land_mark_vis.h"

std::vector<Eigen::Vector3d> GetLandmarks() {
    Eigen::Vector3d pt1{1.0, 0, 0};
    Eigen::Vector3d pt2{1.0, 1, 0};
    Eigen::Vector3d pt3{0, 0, 0};
    Eigen::Vector3d pt4{0, 1, 0};
    std::vector<Eigen::Vector3d> pts;
    pts.emplace_back(pt1);
    pts.emplace_back(pt2);
    pts.emplace_back(pt3);
    pts.emplace_back(pt4);
    return pts;
}

int main(int argc, char **argv ) {
    ros::init(argc, argv, "pose_visualize");
    ros::NodeHandle n;
    CamPoseVis cameraposevisual(0, 1, 0, 1);
    LandMarkVis landmark_vis(1, 0, 0, 1);

    int quesize = 1000;
    auto pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", quesize);
    auto pub_land_mark = n.advertise<sensor_msgs::PointCloud>("land_mark_visual", quesize);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        Eigen::Vector3d P{1, 0, 0};
        Eigen::Quaterniond R;
        R.setIdentity();

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "world";

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, header);

        landmark_vis.reset();
        landmark_vis.AddPoints(GetLandmarks());
        landmark_vis.publish_by(pub_land_mark, header);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}