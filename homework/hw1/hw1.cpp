// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>


// sai main libraries includes
#include "SaiModel.h"

// sai utilities from sai-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm_controller.urdf";

// Function to log joint trajectories
void logJointTrajectory(std::ofstream &log_file, double time, const VectorXd &q) {

    log_file << time;
    for (int i = 0; i < q.size(); ++i) {
        log_file << "," << q(i);
    }
    log_file << std::endl;
}


int main(int argc, char** argv) {
    SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER} {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 5) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new SaiModel::SaiModel(robot_file);

    // prepare controller
    int dof = robot->dof();
    VectorXd control_torques = VectorXd::Zero(dof);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = VectorXd::Zero(dof);
    VectorXd robot_dq = VectorXd::Zero(dof);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);

    std::string filename = "joint_trajectory_q" + std::to_string(controller_number) + ".csv";
    
    std::ofstream log_file(filename);

    log_file << "time";
    for (int i = 0; i < dof; ++i) {
        log_file << ",q" << i + 1;
    }
    log_file << std::endl;

    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {

            double kp = 400.0;      // chose your p gain
            double kv = 50.0;     // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI / 2, -M_PI / 4, 0.0, -125.0 * M_PI / 180.0, 0.0, 80.0 * M_PI / 180.0, 0.0; 

            control_torques = -kp * (robot_q - q_desired) - kv * robot_dq;  // change to the control torques you compute
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            double kp = 400.0;      // chose your p gain
            double kv = 50.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI / 2, -M_PI / 4, 0.0, -125.0 * M_PI / 180.0, 0.0, 80.0 * M_PI / 180.0, 0.0; 

            control_torques = -kp * (robot_q - q_desired) - kv * robot_dq + robot->jointGravityVector();
            
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            double kp = 400;      // chose your p gain
            double kv = 38.0;     // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI / 2, -M_PI / 4, 0.0, -125.0 * M_PI / 180.0, 0.0, 80.0 * M_PI / 180.0, 0.0; 

            control_torques = robot->M() * (-kp * (robot_q - q_desired) - kv * robot_dq) + robot->jointGravityVector();


        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            double kp = 400.0;
            double kv = 38.0;  // critical damping for joint 0

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI / 2, -M_PI / 4, 0.0, -125.0 * M_PI / 180.0, 0.0, 80.0 * M_PI / 180.0, 0.0; 

            control_torques = robot->M() * (-kp * (robot_q - q_desired) - kv * robot_dq) + robot->coriolisForce() + robot->jointGravityVector();

        }

        // ---------------------------  question 5 ---------------------------------------
        else if(controller_number == 5) {

            double kp = 400.0;
            double kv = 38.0;  // critical damping for joint 0

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI / 2, -M_PI / 4, 0.0, -125.0 * M_PI / 180.0, 0.0, 80.0 * M_PI / 180.0, 0.0; 

            // MatrixXd J_payload = robot->Jv("link7");
            // double payload_mass = 2.5;
            // MatrixXd M_payload = payload_mass * J_payload.transpose() * J_payload;
            // MatrixXd M_augmented = robot->M() + M_payload;

            control_torques = M_augmented * (-kp * (robot_q - q_desired) - kv * robot_dq) + robot->coriolisForce() + robot->jointGravityVector();

            // === Q6 Payload Compensation ===
            // Vector3d g_vector(0, 0, -9.81);
            // VectorXd tau_payload = J_payload.transpose() * (payload_mass * g_vector);
            // control_torques += tau_payload;

        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.setInt("sai::simviz::gravity_comp_enabled", 0);
        redis_client.sendAllFromGroup();
        logJointTrajectory(log_file, time, robot_q);
    }

    control_torques.setZero();
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    log_file.close();
    return 0;
}
