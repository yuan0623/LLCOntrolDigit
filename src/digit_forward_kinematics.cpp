//
// Created by yuan on 3/4/21.
//
#include<iostream>
#include "digit_forward_kinematics.h"
Eigen::Matrix<double,4,4> leftFoot(Eigen::Matrix<double,1,30> q){
    Eigen::Matrix<double,4,4> Aw2bp,Rb_roll,Rb_pitch,Rb_yaw,Rb_rpy,Aw2b,Rb2hip_abduction_left1,Rb2hip_abduction_left2,
    Rhip_abduction_left2hip_rotation_left1,Rhip_abduction_left2hip_rotation_left2,Rhip_rotation_left2hip_flexion_left1,Rhip_rotation_left2hip_flexion_left2,
    Rhip_flexion_left2knee_joint_left1,Rhip_flexion_left2knee_joint_left2,Rknee_joint_left2knee_to_shin_left1,Rknee_joint_left2knee_to_shin_left2,
    Rknee_to_shin_left2shin_to_tarsus_left1,Rknee_to_shin_left2shin_to_tarsus_left2,Rshin_to_tarsus_left2toe_pitch_joint_left1,Rshin_to_tarsus_left2toe_pitch_joint_left2,
    Rtoe_pitch_joint_left2toe_roll_joint_left1,Rtoe_pitch_joint_left2toe_roll_joint_left2, Rtoe_roll_joint_left2bottom_feet;

    Aw2bp << 1, 0, 0, q(0),
            0, 1, 0, q(1),
            0, 0, 1, q(2),
            0, 0, 0, 1;

    Rb_roll << 1, 0,          0,         0,
            0, cos(q(3)), -sin(q(3)), 0,
            0, sin(q(3)),  cos(q(3)), 0,
            0, 0,          0,         1;
    Rb_pitch << cos(q(4)), 0, sin(q(4)), 0,
                0,         1, 0,         0,
                -sin(q(4)), 0, cos(q(4)), 0,
                0,         0, 0,         1;
    Rb_yaw << cos(q(5)), -sin(q(5)), 0, 0,
                sin(q(5)),  cos(q(5)), 0, 0,
                0,          0,         1, 0,
                0,          0,         0, 1;

    Rb_rpy = Rb_roll*Rb_pitch*Rb_yaw;
    Aw2b = Aw2bp*Rb_rpy;

    // base to left leg
    Rb2hip_abduction_left1 << 0,0,-1, -1e-03,
                            -0.366501000000000,0.930418000000000,0,0.091,
                            0.930418000000000,0.366501000000000,0,0,
                            0,                 0,                 0,1;
    Rb2hip_abduction_left2 << cos(q(6)), -sin(q(6)), 0, 0,
                            sin(q(6)),  cos(q(6)), 0, 0,
                            0,          0,         1, 0,
                            0,          0,         0, 1;
    auto Rb2hip_abduction_left = Rb2hip_abduction_left1*Rb2hip_abduction_left2;

    Rhip_abduction_left2hip_rotation_left1 << 0, 0, -1,  -0.0505,
                                            0, 1,  0,  0,
                                            1, 0,  0,  0.0440,
                                            0, 0,  0,  1;
    Rhip_abduction_left2hip_rotation_left2 << cos(q(7)), -sin(q(7)), 0, 0,
                                        sin(q(7)),  cos(q(7)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;
    auto Rhip_abduction_left2hip_rotation_left = Rhip_abduction_left2hip_rotation_left1*Rhip_abduction_left2hip_rotation_left2;

    Rhip_rotation_left2hip_flexion_left1 << -0.707107000000000,-0.707107000000000,0,0,
                                        0,0,-1,0.004,
                                        0.707107000000000,-0.707107000000000,0,0.068,
                                        0,                 0,                0, 1;
    Rhip_rotation_left2hip_flexion_left2 << cos(-q(8)), -sin(-q(8)), 0, 0,
                                    sin(-q(8)),  cos(-q(8)), 0, 0,
                                    0,          0,         1, 0,
                                    0,          0,         0, 1;
    auto Rhip_rotation_left2hip_flexion_left = Rhip_rotation_left2hip_flexion_left1*Rhip_rotation_left2hip_flexion_left2;

    Rhip_flexion_left2knee_joint_left1 << 0, 1, 0, 0.12,
                                        -1, 0, 0, 0,
                                        0, 0, 1, 0.0045,
                                        0, 0, 0, 1;
    Rhip_flexion_left2knee_joint_left2 << cos(q(9)), -sin(q(9)), 0, 0,
                                        sin(q(9)),  cos(q(9)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;
    auto Rhip_flexion_left2knee_joint_left = Rhip_flexion_left2knee_joint_left1*Rhip_flexion_left2knee_joint_left2;

    Rknee_joint_left2knee_to_shin_left1 << 1, 0, 0, 0.0607,
                                        0, 1, 0, 0.0474,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1;
    Rknee_joint_left2knee_to_shin_left2 << cos(q(10)), -sin(q(10)), 0, 0,
                                        sin(q(10)),  cos(q(10)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;
    auto Rknee_joint_left2knee_to_shin_left = Rknee_joint_left2knee_to_shin_left1*Rknee_joint_left2knee_to_shin_left2;

    Rknee_to_shin_left2shin_to_tarsus_left1 << -0.224951000000000,-0.974370000000000,0, 0.4348,
                                            0.974370000000000, -0.224951000000000,0, 0.02,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1;
    Rknee_to_shin_left2shin_to_tarsus_left2 << cos(q(11)), -sin(q(11)), 0, 0,
                                        sin(q(11)),  cos(q(11)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;

    auto Rknee_to_shin_left2shin_to_tarsus_left = Rknee_to_shin_left2shin_to_tarsus_left1*Rknee_to_shin_left2shin_to_tarsus_left2;

    Rshin_to_tarsus_left2toe_pitch_joint_left1 << 0.366455000000000,-0.930436000000000,0, 0.408,
                                                0.930436000000000,0.366455000000000,0,-0.04,
                                                0,0,1,0,
                                                0,0,0,1;
    Rshin_to_tarsus_left2toe_pitch_joint_left2 << cos(q(12)), -sin(q(12)), 0, 0,
                                                sin(q(12)),  cos(q(12)), 0, 0,
                                                0,          0,         1, 0,
                                                0,          0,         0, 1;
    auto Rshin_to_tarsus_left2toe_pitch_joint_left = Rshin_to_tarsus_left2toe_pitch_joint_left1*Rshin_to_tarsus_left2toe_pitch_joint_left2;

    Rtoe_pitch_joint_left2toe_roll_joint_left1 << 0,0,1,0,
                                                0,1,0,0,
                                                -1,0,0,0,
                                                0, 0, 0, 1;
    Rtoe_pitch_joint_left2toe_roll_joint_left2 << cos(q(13)), -sin(q(13)), 0, 0,
                                            sin(q(13)),  cos(q(13)), 0, 0,
                                            0,          0,         1, 0,
                                            0,          0,         0, 1;
    auto Rtoe_pitch_joint_left2toe_roll_joint_left = Rtoe_pitch_joint_left2toe_roll_joint_left1*Rtoe_pitch_joint_left2toe_roll_joint_left2;

    double theta = -M_PI/3;
    Rtoe_roll_joint_left2bottom_feet << 1, 0,           0,          0,
                                    0, cos(theta), -sin(theta), 0,
                                    0, sin(theta),  cos(theta), 0,
                                    0, 0,           0,          1;

    auto Rb2left_toe = Aw2b*Rb2hip_abduction_left*Rhip_abduction_left2hip_rotation_left*
    Rhip_rotation_left2hip_flexion_left*Rhip_flexion_left2knee_joint_left*
    Rknee_joint_left2knee_to_shin_left*Rknee_to_shin_left2shin_to_tarsus_left*
    Rshin_to_tarsus_left2toe_pitch_joint_left*Rtoe_pitch_joint_left2toe_roll_joint_left;
    auto Rb2left_foot_bottom = Rb2left_toe*Rtoe_roll_joint_left2bottom_feet;
    //Eigen::Vector3d left_foot_position = Rb2left_toe.block(0,3,3,1);
    return Rb2left_foot_bottom;
}

Eigen::Matrix<double,4,4> rightFoot(Eigen::Matrix<double,1,30> q){
    Eigen::Matrix<double,4,4> Aw2bp,Rb_roll,Rb_pitch,Rb_yaw,Rb_rpy,Aw2b,Rb2hip_abduction_right1,Rb2hip_abduction_right2,
            Rhip_abduction_right2hip_rotation_right1,Rhip_abduction_right2hip_rotation_right2,Rhip_rotation_right2hip_flexion_right1,Rhip_rotation_right2hip_flexion_right2,
            Rhip_flexion_right2knee_joint_right1,Rhip_flexion_right2knee_joint_right2,Rknee_joint_right2knee_to_shin_right1,Rknee_joint_right2knee_to_shin_right2,
            Rknee_to_shin_right2shin_to_tarsus_right1,Rknee_to_shin_right2shin_to_tarsus_right2,Rshin_to_tarsus_right2toe_pitch_joint_right1,Rshin_to_tarsus_right2toe_pitch_joint_right2,
            Rtoe_pitch_joint_right2toe_roll_joint_right1,Rtoe_pitch_joint_right2toe_roll_joint_right2, Rtoe_roll_joint_right2bottom_feet;

    // world frame to base frame
    Aw2bp << 1, 0, 0, q(0),
                0, 1, 0, q(1),
                0, 0, 1, q(2),
                0, 0, 0, 1;
    Rb_roll << 1, 0,          0,         0,
            0, cos(q(3)), -sin(q(3)), 0,
            0, sin(q(3)),  cos(q(3)), 0,
            0, 0,          0,         1;
    Rb_pitch << cos(q(4)), 0, sin(q(4)), 0,
                0,         1, 0,         0,
                -sin(q(4)), 0, cos(q(4)), 0,
                0,         0, 0,         1;
    Rb_yaw << cos(q(5)), -sin(q(5)), 0, 0,
                sin(q(5)),  cos(q(5)), 0, 0,
                0,          0,         1, 0,
                0,          0,         0, 1;
    Rb_rpy = Rb_roll*Rb_pitch*Rb_yaw;
    Aw2b = Aw2bp*Rb_rpy;

    // base to right leg

    Rb2hip_abduction_right1 << 0,0,-1,-1e-3,
                        0.366501000000000,0.930418000000000,0,-0.091,
                        0.930418000000000,-0.366501000000000,0,0,
                        0,                 0,                  0,1;

    Rb2hip_abduction_right2 << cos(q(18)), -sin(q(18)), 0, 0,
                            sin(q(18)),  cos(q(18)), 0, 0,
                            0,          0,         1, 0,
                            0,          0,         0, 1;
    auto Rb2hip_abduction_right = Rb2hip_abduction_right1*Rb2hip_abduction_right2;

    Rhip_abduction_right2hip_rotation_right1 << 0,0,-1,-0.0505,
                                            0,1,0,0,
                                            1,0,0,0.044,
                                            0,0,0,1;

    Rhip_abduction_right2hip_rotation_right2 << cos(q(19)), -sin(q(19)), 0, 0,
                                            sin(q(19)),  cos(q(19)), 0, 0,
                                            0,          0,         1, 0,
                                            0,          0,         0, 1;
    auto Rhip_abduction_right2hip_rotation_right = Rhip_abduction_right2hip_rotation_right1*Rhip_abduction_right2hip_rotation_right2;

    Rhip_rotation_right2hip_flexion_right1 << -0.707107000000000,0.707107000000000,0,0,
                                        0,0,1,-0.004,
                                        0.707107000000000,0.707107000000000,0,0.068,
                                        0,0,0,1;
    Rhip_rotation_right2hip_flexion_right2 << cos(-q(20)), -sin(-q(20)), 0, 0,
                                        sin(-q(20)),  cos(-q(20)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;
    auto Rhip_rotation_right2hip_flexion_right = Rhip_rotation_right2hip_flexion_right1*Rhip_rotation_right2hip_flexion_right2;

    Rhip_flexion_right2knee_joint_right1 << 0,-1,0,0.12,
                                        1,0,0,0,
                                        0,0,1,0.0045,
                                        0,0,0,1;
    Rhip_flexion_right2knee_joint_right2 << cos(q(21)), -sin(q(21)), 0, 0,
                                    sin(q(21)),  cos(q(21)), 0, 0,
                                    0,          0,         1, 0,
                                    0,          0,         0, 1;
    auto Rhip_flexion_right2knee_joint_right = Rhip_flexion_right2knee_joint_right1*Rhip_flexion_right2knee_joint_right2;

    Rknee_joint_right2knee_to_shin_right1 << 1,0,0,0.0607,
                                        0,1,0,-0.0474,
                                        0,0,1,0,
                                        0,0,0,1;
    Rknee_joint_right2knee_to_shin_right2 <<  cos(q(22)), -sin(q(22)), 0, 0,
                                    sin(q(22)),  cos(q(22)), 0, 0,
                                    0,          0,         1, 0,
                                    0,          0,         0, 1;
    auto Rknee_joint_right2knee_to_shin_right = Rknee_joint_right2knee_to_shin_right1*Rknee_joint_right2knee_to_shin_right2;

    Rknee_to_shin_right2shin_to_tarsus_right1 << -0.224951000000000,0.974370000000000,0,0.4348,
                                        -0.974370000000000,-0.224951000000000,0,-0.02,
                                        0,0,1,0,
                                        0,0,0,1;
    Rknee_to_shin_right2shin_to_tarsus_right2 << cos(q(23)), -sin(q(23)), 0, 0,
                                        sin(q(23)),  cos(q(23)), 0, 0,
                                        0,          0,         1, 0,
                                        0,          0,         0, 1;
    auto Rknee_to_shin_right2shin_to_tarsus_right = Rknee_to_shin_right2shin_to_tarsus_right1*Rknee_to_shin_right2shin_to_tarsus_right2;

    Rshin_to_tarsus_right2toe_pitch_joint_right1 << 0.366455000000000,0.930436000000000,0,0.408,
                                            -0.930436000000000,0.366455000000000,0,0.04,
                                            0,0,1,0,
                                            0,0,0,1;
    Rshin_to_tarsus_right2toe_pitch_joint_right2 << cos(q(24)), -sin(q(24)), 0, 0,
                                            sin(q(24)),  cos(q(24)), 0, 0,
                                            0,          0,         1, 0,
                                            0,          0,         0, 1;
    auto Rshin_to_tarsus_right2toe_pitch_joint_right = Rshin_to_tarsus_right2toe_pitch_joint_right1*Rshin_to_tarsus_right2toe_pitch_joint_right2;

    Rtoe_pitch_joint_right2toe_roll_joint_right1 << 0,0,1,0,
                                                0,1,0,0,
                                                -1,0,0,0,
                                                0,0,0,1;
    Rtoe_pitch_joint_right2toe_roll_joint_right2 << cos(q(25)), -sin(q(25)), 0, 0,
                                                sin(q(25)),  cos(q(25)), 0, 0,
                                                0,          0,         1, 0,
                                                0,          0,         0, 1;
    auto Rtoe_pitch_joint_right2toe_roll_joint_right = Rtoe_pitch_joint_right2toe_roll_joint_right1*Rtoe_pitch_joint_right2toe_roll_joint_right2;
    auto Rb2right_toe = Aw2b*Rb2hip_abduction_right*Rhip_abduction_right2hip_rotation_right*
    Rhip_rotation_right2hip_flexion_right*Rhip_flexion_right2knee_joint_right*
    Rknee_joint_right2knee_to_shin_right*Rknee_to_shin_right2shin_to_tarsus_right*
    Rshin_to_tarsus_right2toe_pitch_joint_right*Rtoe_pitch_joint_right2toe_roll_joint_right;

    double theta = M_PI/3;
    Rtoe_roll_joint_right2bottom_feet << 1, 0,           0,          0,
                                        0, cos(theta), -sin(theta), 0,
                                        0, sin(theta),  cos(theta), 0,
                                        0, 0,           0,          1;
    //Eigen::Vector3d right_foot_position = Rb2right_toe.block(0,3,3,1);
    auto Rb2right_foot_bottom = Rb2right_toe*Rtoe_roll_joint_right2bottom_feet;
    return Rb2right_foot_bottom;
}

std::vector<std::pair<int,bool> > contact_detection(Eigen::Matrix<double,1,10> joint_position){
    auto left_spring_heel = joint_position[4];
    auto right_spring_heel = joint_position[9];
    bool left_contact_indicator, right_contact_indicator;
    int left_contact_id, right_contact_id;
    left_contact_id = 0;
    right_contact_id = 1;
    std::cout<<"left_spring_heel"<<left_spring_heel<<std::endl;
    std::cout<<"right_spring_heel"<<right_spring_heel<<std::endl;
    /*
    if (right_spring_heel > 0.018){
        right_contact_indicator = true;
    }
    else{
        right_contact_indicator = false;
    }

    if(left_spring_heel<-0.038){
        left_contact_indicator = true;
    }
    else{
        left_contact_indicator = false;
    }
     */
    if (right_spring_heel > 0.04){
        right_contact_indicator = false;
    }
    else{
        right_contact_indicator = true;
    }

    if(left_spring_heel<-0.044){
        left_contact_indicator = false;
    }
    else{
        left_contact_indicator = true;
    }
    std::vector<std::pair<int,bool> > contacts;
    contacts.push_back(std::pair<int,bool> (left_contact_id, left_contact_indicator));
    contacts.push_back(std::pair<int,bool> (right_contact_id, right_contact_indicator));
    return contacts;
}

inekf::vectorKinematics generate_vector_kinematics(Eigen::Matrix<double,1,30> q){
    Eigen::Matrix<double,4,4> left_pose = leftFoot(q);
    Eigen::Matrix<double,4,4> right_pose = rightFoot(q);
    //Eigen::Matrix4d left_pose = Eigen::Matrix4d::Identity();
    //Eigen::Matrix4d right_pose = Eigen::Matrix4d::Identity();
    //left_pose.block<3,1>(0,3) = left_foot;
    //right_pose.block<3,1>(0,3) = right_foot;
    Eigen::Matrix<double,6,6> covariance = 0.01*Eigen::Matrix<double,6,6>::Identity();;
    inekf::vectorKinematics measured_kinematics;
    inekf::Kinematics left_frame(0, left_pose, covariance);
    inekf::Kinematics right_frame(1, right_pose, covariance);
    measured_kinematics.push_back(left_frame);
    measured_kinematics.push_back(right_frame);

    return measured_kinematics;
}