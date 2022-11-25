#include <iostream>
#include "lowlevelapi.h"
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <thread>
#include "InEKF.h"
#include "digit_forward_kinematics.h"
#include "digit_json_helper.h"
#include <math.h>
#define DT_MIN 1e-6
#define DT_MAX 1

static double target_position[] = {
        -0.0462933,
        -0.0265814,
        0.19299,
        -0.3,
        -0.0235182,
        -0.0571617,
        0.0462933,
        0.0265814,
        -0.19299,
        0.3,
        -0.0235182,
        0.0571617,
        -0.3,
        0.943845,
        0.0,
        0.3633,
        0.3,
        -0.943845,
        0.0,
        -0.3633,
};


void writeToCSVfile(std::string name, Eigen::MatrixXd matrix)
{
    std::ofstream file(name.c_str());
    if (file.is_open())
    {
        file << matrix << '\n';
        //file << "m" << '\n' <<  colm(matrix) << '\n';
    }
}

int main(int argc, char* argv[]) {
    // The publisher address should be changed to the ip address of the robot
    const char* publisher_address = "10.10.1.1";

    std::string host, motion_file_name, sim_or_hw;
    //

    std::string date = "_1122";
    std::string trial_num = "_v1";
    bool is_sim = false;
    if(is_sim == true)
    {
        publisher_address = "127.0.0.1";
        host = "127.0.0.1";
        sim_or_hw = "sim";
    }
    else
    {
        publisher_address = "10.10.1.1";
        host = "10.10.1.1";
        sim_or_hw = "hw";
    }

    llapi_init(publisher_address);


    //inekf::NoiseParams noise_params;
    inekf::RobotState initial_state;
    //JsonMessageCommunication jsonCom(host,"8080");
    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
            0, -1, 0, // IMU frame is rotated 90deg about the x-axis
            0, 0, -1;
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.01);

    // Initialize filter

    inekf::InEKF filter(initial_state, noise_params);


    // Define inputs and outputs (updated each iteration)
    llapi_command_t command = {0};
    llapi_observation_t observation;

    // Connect to robot (need to send commands until the subscriber connects)
    command.apply_command = true;
    while (!llapi_get_observation(&observation)) llapi_send_command(&command);

    // Get local copy of command limits (torque and damping)
    const llapi_limits_t* limits = llapi_get_limits();

    // store sensor data
    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,3,1> gyro_reading = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> acc_reading = Eigen::Matrix<double,3,1>::Zero();
    Eigen::MatrixXd motor_position_vec = Eigen::Matrix<double,1,20>::Zero();
    Eigen::Matrix<double,1,20> motor_position = Eigen::Matrix<double,1,20>::Zero();
    Eigen::Matrix<double,3,3> IMU_rotate = Eigen::Matrix<double,3,3>::Zero();
    IMU_rotate<<cos(-M_PI/2),0,sin(-M_PI/2),
                    0,1,0,
                    -sin(-M_PI/2),0,cos(-M_PI/2);
    std::ofstream motionFile, joint_vecFile, posEstFile,posGTFile;
    motionFile.open ("../recorded_data/digit_walking_"+sim_or_hw+date+".txt",std::ifstream::app);
    posEstFile.open("../recorded_data/posEstFile.txt",std::ifstream::app);
    posGTFile.open("../recorded_data/posGTFile.txt",std::ifstream::app);
    int loop_count = 0;
    double t_prev = 0;

    while (loop_count<4000) {

        // Update observation
        int return_val = llapi_get_observation(&observation);
        if (return_val < 1) {
            // Error occurred
        } else if (return_val) {
            loop_count++;


            auto acc_ = observation.imu.linear_acceleration;
            auto gyro_ = observation.imu.angular_velocity;
            auto motor_reading_ = observation.motor.position;
            auto joint_reading_ = observation.joint.position;
            auto base_position_ = observation.base.translation;
            auto base_velocity_ = observation.base.linear_velocity;
            auto motor_torque_ = observation.motor.torque;
            auto t = observation.time;
            Eigen::Matrix<double,1,20> motor_position = Eigen::Matrix<double,1,20>::Zero();
            Eigen::Matrix<double,1,10> joint_position = Eigen::Matrix<double,1,10>::Zero();
            Eigen::Matrix<double,1,20> motor_torque = Eigen::Matrix<double,1,20>::Zero();
            Eigen::Matrix<double,1,30> joint_position_kin = Eigen::Matrix<double,1,30>::Zero();
            Eigen::Matrix<double,1,3> base_position = Eigen::Matrix<double,1,3>::Zero();


            motor_position << motor_reading_[0], motor_reading_[1],motor_reading_[2],motor_reading_[3],motor_reading_[4],motor_reading_[5],motor_reading_[6],motor_reading_[7],motor_reading_[8],motor_reading_[9],motor_reading_[10],motor_reading_[11],motor_reading_[12],motor_reading_[13],motor_reading_[14],motor_reading_[15],motor_reading_[16],motor_reading_[17],motor_reading_[18],motor_reading_[19];
            motor_torque << motor_torque_[0], motor_torque_[1], motor_torque_[2], motor_torque_[3], motor_torque_[4], motor_torque_[5], motor_torque_[6], motor_torque_[7], motor_torque_[8], motor_torque_[9], motor_torque_[10], motor_torque_[11], motor_torque_[12], motor_torque_[13], motor_torque_[14], motor_torque_[15], motor_torque_[16], motor_torque_[17], motor_torque_[18], motor_torque_[19];
            joint_position <<joint_reading_[0],joint_reading_[1],joint_reading_[2],joint_reading_[3],joint_reading_[4],joint_reading_[5],joint_reading_[6],joint_reading_[7],joint_reading_[8],joint_reading_[9];
            joint_position_kin << 0, 0, 0, 0, 0, 0, motor_position.block(0,0,1,4),joint_position.block(0,0,1,4),0,0,0,0,motor_position.block(0,6,1,4),joint_position.block(0,5,1,4),0,0,0,0;
            base_position<<base_position_[0],base_position_[1],base_position_[2];
            Eigen::MatrixXd left_foot_pose = leftFoot(joint_position_kin);
            Eigen::MatrixXd right_foot_pose = rightFoot(joint_position_kin);
            std::vector<std::pair<int,bool> > contacts;

            contacts.push_back(std::pair<int,bool> (0, true));
            contacts.push_back(std::pair<int,bool> (1, true));


            // New data received
            gyro_reading << observation.imu.angular_velocity[0],
                    observation.imu.angular_velocity[1],
                    observation.imu.angular_velocity[2];
            acc_reading << observation.imu.linear_acceleration[0],
                    observation.imu.linear_acceleration[1],
                    observation.imu.linear_acceleration[2];
            auto gyro_reading_ = IMU_rotate*gyro_reading;
            auto acc_reading_ = IMU_rotate*acc_reading;
            imu_measurement<< gyro_reading_,
                                acc_reading_;
            motor_position << motor_reading_[0], motor_reading_[1],motor_reading_[2],motor_reading_[3],motor_reading_[4],motor_reading_[5],motor_reading_[6],motor_reading_[7],motor_reading_[8],motor_reading_[9],motor_reading_[10],motor_reading_[11],motor_reading_[12],motor_reading_[13],motor_reading_[14],motor_reading_[15],motor_reading_[16],motor_reading_[17],motor_reading_[18],motor_reading_[19];
            //motionFile<<t<<" "<<motor_position<<std::endl;
            double dt = t - t_prev;

            // filter propogate and update
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(imu_measurement, dt);
                filter.setContacts(contacts);
                std::cout<<"filter propagation"<<std::endl;


                Eigen::Matrix<double,6,6> covariance = 0.01*Eigen::Matrix<double,6,6>::Identity();
                inekf::Kinematics frame_left(0, left_foot_pose, covariance);
                inekf::Kinematics frame_right(1, right_foot_pose, covariance);
                inekf::vectorKinematics measured_kinematics;
                measured_kinematics.push_back(frame_left);
                measured_kinematics.push_back(frame_right);
                filter.CorrectKinematics(measured_kinematics);
                std::cout<<"filter update"<<std::endl;
                observation.base.translation;

                auto current_state = filter.getState();
                auto current_pos = current_state.getPosition();
                auto current_vel = current_state.getVelocity();

                posEstFile<<t<<" "<<current_pos.transpose()<<std::endl;
                posGTFile<<t<<" "<<base_position<<std::endl;

                motionFile << "IMU "<<t<<" "<<gyro_reading.transpose()<<" "<<acc_reading.transpose()<<std::endl;
                motionFile << "CONTACTS "<<t<<" " << contacts[0].first<<" "<<contacts[0].second<<" "<<contacts[1].first<<" "<<contacts[1].second <<std::endl;
                motionFile << "KINEMATICS "<<t<<" "<< joint_position_kin<<std::endl;
            }




            t_prev = t;
        } else {
            // No new data
        }

        // PD control on positions
        for (int i = 0; i < NUM_MOTORS; ++i) {
            command.motors[i].torque =
                    150.0 * (target_position[i] - observation.motor.position[i]);
            command.motors[i].velocity = 0.0;
            command.motors[i].damping = 0.75 * limits->damping_limit[i];
        }
        command.fallback_opmode = Damping;
        command.apply_command = true;

        llapi_send_command(&command);

        // Check if llapi has become disconnected
        if (!llapi_connected()) {
            // Handle error case. You don't need to re-initialize subscriber
            // Calling llapi_send_command will keep low level api open
        }

        // Sleep to keep to a reasonable update rate
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    //motionFile.close();
    posEstFile.close();
    posGTFile.close();
    motionFile.close();
    return 0;
}
