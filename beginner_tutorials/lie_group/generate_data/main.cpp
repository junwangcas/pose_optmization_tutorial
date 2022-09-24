#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include "generate_data.h"

int main(int argc, char **argv ) {
//    ros::init(argc, argv, "pose_visualize");
//    ros::NodeHandle n;
    GenData gen_data;
    std::vector<LandMark> land_marks;
    std::vector<Pose> poses;
    std::vector<Observation> observations;
    gen_data.GetData(land_marks, poses, observations);
    std::cout << land_marks.size() << ", " << poses.size() << ", " << observations.size() << "\n";
}