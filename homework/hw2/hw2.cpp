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
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

// Function to log joint trajectories
void logJointAndTaskTrajectory(std::ofstream &log_file, double time, 
                               const VectorXd &q, 
                               const Vector3d &x, 
                               const Vector3d &dx) {
    log_file << time;

    // joint positions
    for (int i = 0; i < q.size(); ++i) {
        log_file << "," << q(i);
    }
    // task positions
    for (int i = 0; i < 3; ++i) {
        log_file << "," << x(i);
    }
    // task velocities
    for (int i = 0; i < 3; ++i) {
        log_file << "," << dx(i);
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
        else if (controller_number < 1 || controller_number > 4) {
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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
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

    // Setup Log File
    std::string filename = "joint_trajectory_q" + std::to_string(controller_number) + ".csv";
    std::ofstream log_file(filename);
    log_file << "time";
    for (int i = 0; i < dof; ++i) {
        log_file << ",q" << i + 1;
    }
    for (int i = 0; i < 3; ++i) {
        log_file << ",x" << i + 1;
    }
    for (int i = 0; i < 3; ++i) {
        log_file << ",dx" << i + 1;
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
            
            MatrixXd Kp = MatrixXd::Zero(7, 7);
            MatrixXd Kv = MatrixXd::Zero(7, 7);

            VectorXd q_desired = robot_q;
            q_desired(6) = 0.1;

            for (int i = 0; i < 6; ++i) {
                Kp(i, i) = 400.0;
                Kv(i, i) = 50.0;
            }

            // Joint 7:
            Kp(6, 6) = 50.0;
            Kv(6, 6) = -0.153;

            control_torques = -Kp * (robot_q - q_desired) - Kv * robot_dq + robot->coriolisForce() + robot->jointGravityVector();

        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {
            Vector3d xd(0.3, 0.1, 0.5);

            double kp = 200.0;
            double kv = 29.0;
            double kvj = 10.0;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);             // EE pos
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);      // EE vel

            // Task-space control force
            Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);                 // 3x1

            // Map to joint torques
            control_torques = Jv.transpose() * F + robot->jointGravityVector();               // Nx1

            // Q2c add kvj, joint space damping
            // control_torques += -kvj * robot->dq();

            // Q2d add joint space damping in nullspace task control
            control_torques += N.transpose() * robot->M() * (-kvj * robot->dq());


        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            Vector3d xd(0.3, 0.1, 0.5);

            double kp = 200.0;
            double kv = 29.0;
            double kvj = 10.0;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);             // EE pos
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);      // EE vel

            // Operational Space Gravity
            Vector3d p = J_bar.transpose() * robot->jointGravityVector(); 

            // Task-space control force
            Vector3d F = Lambda * (-kp * (x - xd) - kv * dx) + p;                 // 3x1

            // Map to joint torques
            control_torques = Jv.transpose() * F;                                 // Nx1
            control_torques += N.transpose() * robot->M() * (-kvj * robot->dq());
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {
            Vector3d xd;
            xd(0) = 0.3 + 0.1 * std::sin(M_PI * time);
            xd(1) = 0.1 + 0.1 * std::cos(M_PI * time);
            xd(2) = 0.5;

            double kp = 200.0;
            double kv = 29.0;
            double kpj = 10.0;
            double kvj = 10.0;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);             // EE pos
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);      // EE vel

            // Operational Space Gravity
            Vector3d p = J_bar.transpose() * robot->jointGravityVector(); 

            // Task-space control force
            Vector3d F = Lambda * (-kp * (x - xd) - kv * dx) + p;                 // 3x1

            // Q4b remove lambda
            // Vector3d F = (-kp * (x - xd) - kv * dx) + p;                 // 3x1

            // Map to joint torques
            control_torques = Jv.transpose() * F;                                 // Nx1
            // control_torques += N.transpose() * robot->M() * (-kvj * robot->dq());

            // Q4c add PD control in nullspace task control
            VectorXd q_desired = VectorXd::Zero(dof);
            // control_torques += N.transpose() * robot->M() * (-kpj * (robot_q - q_desired) - kvj * robot_dq);

            // Q4e add joint space gravity compensation
            control_torques += N.transpose() * (robot->M() * (-kpj * (robot_q - q_desired) - kvj * robot_dq) + robot->jointGravityVector());
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
        
        // Current state
        Vector3d x = robot->position(link_name, pos_in_link);             // EE pos
        Vector3d dx = robot->linearVelocity(link_name, pos_in_link);      // EE vel
        logJointAndTaskTrajectory(log_file, time, robot_q, x, dx);
    }

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
