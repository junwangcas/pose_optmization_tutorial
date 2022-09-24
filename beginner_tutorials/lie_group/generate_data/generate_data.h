#ifndef BEGINNER_TUTORIALS_GENERATE_DATA_H
#define BEGINNER_TUTORIALS_GENERATE_DATA_H
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <chrono>

struct LandMark {
    Eigen::Vector3d pt_gt;
    uint64_t land_mark_id{0};
    LandMark(const Eigen::Vector3d pt, const uint64_t id) : pt_gt(pt), land_mark_id(id) {}
};

struct Pose {
    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    Eigen::Matrix3d rot;
    uint64_t id;
    Pose(const Eigen::Vector3d &ttmp, const Eigen::Quaterniond &qtmp, const uint64_t idtmp) : trans(ttmp), q(qtmp), id(idtmp) { rot = qtmp.toRotationMatrix();}
};

struct Observation {
    uint64_t pose_id{0};
    uint64_t land_mark_id{0};
    uint64_t obs_id{0};
    Eigen::Vector3d measure;
};

class GenData {
public:
    GenData();
    ~GenData();
    void GetData(std::vector<LandMark> &land_marks, std::vector<Pose> &poses, std::vector<Observation> &observations);

private:
    int land_mark_size_{4};
    int pose_size_{1};
    int frame_size_{10};
    std::vector<LandMark> land_marks_;
    std::vector<Pose> poses_;
    std::vector<Observation> observations_;
    double stddev_{0.5};

private:
    void GenGroundTruth();
    void GenObservs();
    Observation GenObservation(const Pose &pose, const LandMark &land_mark);
    double GaussianNoise(double mean, double stddev);
};


#endif //BEGINNER_TUTORIALS_GENERATE_DATA_H
