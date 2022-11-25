//
// Created by yuan on 3/4/21.
//
#include <Eigen/Dense>
#ifndef DIGIT_KF_DIGIT_FORWARD_KINEMATICS_H
#define DIGIT_KF_DIGIT_FORWARD_KINEMATICS_H
#include "InEKF.h"
#endif //DIGIT_KF_DIGIT_FORWARD_KINEMATICS_H

Eigen::Matrix<double,4,4> leftFoot(Eigen::Matrix<double,1,30> joint_position);

Eigen::Matrix<double,4,4> rightFoot(Eigen::Matrix<double,1,30> joint_position);

std::vector<std::pair<int,bool>> contact_detection(Eigen::Matrix<double,1,10> joint_position);

inekf::vectorKinematics generate_vector_kinematics(Eigen::Matrix<double,1,30> q);
