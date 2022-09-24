//
// Created by junwangcas on 2022/9/24.
//

#include "generate_data.h"

GenData::GenData()
{
}

GenData::~GenData()
{

}

void GenData::GenGroundTruth()
{
    // landmarks
    Eigen::Vector3d pt1{1.0, 0, 0};
    Eigen::Vector3d pt2{1.0, 1, 0};
    Eigen::Vector3d pt3{0, 0, 0};
    Eigen::Vector3d pt4{0, 1, 0};

    LandMark land_mark1(pt1, 0);
    LandMark land_mark2(pt2, 1);
    LandMark land_mark3(pt3, 2);
    LandMark land_mark4(pt4, 3);

    land_marks_.emplace_back(land_mark1);
    land_marks_.emplace_back(land_mark2);
    land_marks_.emplace_back(land_mark3);
    land_marks_.emplace_back(land_mark4);

    // pose
    Eigen::Vector3d p{1, 0, 0};
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Pose pose(p, q, 0);
    poses_.emplace_back(pose);
}

void GenData::GenObservs()
{
    int count = 0;
    while (count < frame_size_) {
        for (int idpose = 0; idpose < poses_.size(); idpose++) {
            auto pose = poses_[idpose];
            for (int id_landmark = 0; id_landmark < land_marks_.size(); id_landmark++) {
                observations_.emplace_back(GenObservation(pose, land_marks_[id_landmark]));
            }
        }
        count++;
    }
}

Observation GenData::GenObservation(const Pose &pose, const LandMark &land_mark)
{
    Eigen::Vector3d measure;
    measure = pose.rot.transpose() * (land_mark.pt_gt - pose.trans);

    measure.x() += GaussianNoise(0.0, stddev_);
    measure.y() += GaussianNoise(0.0, stddev_);
    measure.z() += GaussianNoise(0.0, stddev_);

    Observation observation;
    observation.measure = measure;
    observation.land_mark_id = land_mark.land_mark_id;
    observation.pose_id = pose.id;
    return observation;
}

double GenData::GaussianNoise(double mean, double stddev)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> dist(mean, stddev);
    return dist(generator);
}

void GenData::GetData(std::vector<LandMark> &land_marks, std::vector<Pose> &poses, std::vector<Observation> &observations)
{
    GenGroundTruth();
    GenObservs();
    land_marks = land_marks_;
    poses = poses_;
    observations = observations_;
}