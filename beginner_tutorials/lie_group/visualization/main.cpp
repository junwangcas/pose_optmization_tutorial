#include "camera_pose_vis.h"
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>


int main(int argc, char **argv ) {
    ros::init(argc, argv, "pose_visualize");
    ros::NodeHandle n;
    CamPoseVis cameraposevisual(0, 1, 0, 1);

    auto pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
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

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}