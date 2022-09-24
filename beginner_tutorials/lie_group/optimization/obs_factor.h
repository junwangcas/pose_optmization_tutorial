
#ifndef BEGINNER_TUTORIALS_OBS_FACTOR_H
#define BEGINNER_TUTORIALS_OBS_FACTOR_H
#include <Eigen/Core>

struct ObsFactor {
    ObsFactor(Eigen::Vector3d measure) measure_(measure): {
        
    }

    template<typename T> bool operator () (const T* const pose, const T* const landmark, T* residual) const {
        Eigen::Quaternion<T> q{pose[0], pose[1], pose[2], pose[3]};
        Eigen::Matrix<T, 3, 1> trans{pose[4], pose[5], pose[6]};
        Eigen::Matrix<T, 3, 1> landmark_eigen{landmark[0], landmark[1], landmark[2]};

        Eigen::Matrix<T, 3, 1> measure_pred = q.toRotationMatrix().transpose() * (landmark_eigen - trans);
        residual[0] = measure_pred - static_cast<T>(measure_[0]);
        residual[1] = measure_pred - static_cast<T>(measure_[1]);
        residual[2] = measure_pred - static_cast<T>(measure_[2]);
    }
private:
    Eigen::Vector3d measure_;

};
#endif //BEGINNER_TUTORIALS_OBS_FACTOR_H
