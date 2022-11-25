//
// Created by yuan on 11/22/22.
//

#include <iostream>
#include "lowlevelapi.h"
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <thread>
#include "digit_forward_kinematics.h"
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



int main(int argc, char* argv[])
{
    // The publisher address should be changed to the ip address of the robot
    const char* publisher_address = "10.10.1.1";
    llapi_init(publisher_address);

    // Define inputs and outputs (updated each iteration)
    llapi_command_t command = {0};
    llapi_observation_t observation;

    // Connect to robot (need to send commands until the subscriber connects)
    command.apply_command = false;
    while (!llapi_get_observation(&observation)) llapi_send_command(&command);

    // Get local copy of command limits (torque and damping)
    const llapi_limits_t* limits = llapi_get_limits();
    int loop_count = 0;
    std::ofstream joint_vecFile;
    joint_vecFile.open ("../recorded_data/LLJointCtrTest.txt",std::ifstream::app);
    Eigen::Matrix<double,1,20> motor_position = Eigen::Matrix<double,1,20>::Zero();

    while (loop_count<4000) {
        // Update observation


        int return_val = llapi_get_observation(&observation);
        if (return_val < 1) {
            // Error occurred
        } else if (return_val) {
            loop_count++;
            std::cout<<loop_count<<std::endl;
            auto motor_reading_ = observation.motor.position;

            motor_position<<motor_reading_[0],motor_reading_[1],motor_reading_[2],motor_reading_[3],motor_reading_[4],motor_reading_[5],motor_reading_[6],motor_reading_[7],motor_reading_[8],motor_reading_[9],motor_reading_[10],motor_reading_[11],motor_reading_[12],motor_reading_[13],motor_reading_[14],motor_reading_[15],motor_reading_[16],motor_reading_[17],motor_reading_[18],motor_reading_[19];


            joint_vecFile<<motor_position<<std::endl;;
            // New data received
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
        //usleep(1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    joint_vecFile.close();
}
