//
// Created by junwangcas on 2022/8/27.
//

#include "quaternion_test_chapters.h"

QuaternionTestChapters::QuaternionTestChapters()
{

}

QuaternionTestChapters::~QuaternionTestChapters()
{

}

void QuaternionTestChapters::Chapter12()
{
    Eigen::Quaterniond test1 = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond test2 = Eigen::Quaterniond::Identity();
    // sum is not defined
    //    auto test3 = test1 + test2;
    //    std::cout << test3.coeffs().transpose() << "\n";
}