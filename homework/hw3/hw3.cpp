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

double sat(double x) {
    if (std::abs(x) <= 1.0) {
        return x;
    } else {
        return (x > 0.0) ? 1.0 : -1.0;
    }
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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
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

    std::ofstream torque_log_file;
    std::ofstream delta_phi_log_file;
    if (controller_number == 1) {
        std::string torque_filename = "NT_torque_joint_q1.csv";
        torque_log_file.open(torque_filename);
        torque_log_file << "time";
        for (int i = 0; i < dof; ++i) {
            torque_log_file << ",NT_torque_joint_" << i + 1;
        }
        torque_log_file << std::endl;
    }
    else if (controller_number == 2) {
        std::string torque_filename = "NT_torque_posture_q2.csv";
        torque_log_file.open(torque_filename);
        torque_log_file << "time";
        for (int i = 0; i < dof; ++i) {
            torque_log_file << ",NT_torque_posture_" << i + 1;
        }
        torque_log_file << std::endl;
    }
    else if (controller_number == 3) {
        delta_phi_log_file.open("delta_phi_log.csv");
        delta_phi_log_file << "time,delta_phi_x,delta_phi_y,delta_phi_z" << std::endl;
    }


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

            double kp = 100.0;
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;

            Vector3d xd;
            xd(0) = 0.3 + 0.1 * std::sin(M_PI * time);
            xd(1) = 0.1 + 0.1 * std::cos(M_PI * time);
            xd(2) = 0.5;

            Vector3d dxd;
            dxd(0) = 0.1 * M_PI * std::cos(M_PI * time);
            dxd(1) = 0.1 * M_PI * -std::sin(M_PI * time);
            dxd(2) = 0;

            Vector3d ddxd;
            ddxd(0) = 0.1 * M_PI * M_PI * -std::sin(M_PI * time);
            ddxd(1) = 0.1 * M_PI * M_PI * -std::cos(M_PI * time);
            ddxd(2) = 0;

            VectorXd q_desired = VectorXd::Zero(dof);

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);             // EE pos
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);      // EE vel

            // Task-space control force
            // Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);                 // 3x1

            // 1c improved trajectory tracking
            Vector3d F = Lambda * (ddxd - kp * (x - xd) - kv * (dx - dxd));

            // Map to joint torques
            control_torques = Jv.transpose() * F;                                 // Nx1
            control_torques += N.transpose() * (-kpj * (robot_q - q_desired) - kvj * robot_dq) + robot->jointGravityVector();

            torque_log_file << time;
            VectorXd NT_torque_joint(dof);
            NT_torque_joint = N.transpose() * (-kpj * (robot_q - q_desired) - kvj * robot_dq);
            for (int i = 0; i < dof; ++i) {
                torque_log_file << "," << NT_torque_joint(i);
            }
            torque_log_file << std::endl;

        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            double kp = 100.0;
            double kv = 20.0;
            double kpj = 50.0;
            double kvj = 14.0;

            const double DEG2RAD = M_PI / 180.0;
            VectorXd q_upper(dof);
            q_upper << 165 * DEG2RAD, 100 * DEG2RAD, 165 * DEG2RAD, -30 * DEG2RAD, 165 * DEG2RAD, 210  * DEG2RAD, 165 * DEG2RAD;

            VectorXd q_lower(dof);
            q_lower << -165 * DEG2RAD, -100 * DEG2RAD, -165 * DEG2RAD, -170 * DEG2RAD, -165 * DEG2RAD, 0  * DEG2RAD, -165 * DEG2RAD;

            double k_mid = 25.0;
            VectorXd torque_mid(dof);
            torque_mid = - 2.0 * k_mid * (robot_q - (q_upper + q_lower) / 2.0);

            double k_damp = 14.0;
            VectorXd torque_damp(dof);
            torque_damp = - k_damp * robot_dq;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);

            ////////////////////////////////////////////
            // // Problem 2b
            // Vector3d xd;
            // xd(0) = 0.3 + 0.1 * std::sin(M_PI * time);
            // xd(1) = 0.1 + 0.1 * std::cos(M_PI * time);
            // xd(2) = 0.5;

            // // Task-space control force
            // Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            // control_torques = Jv.transpose() * F;
            // control_torques += N.transpose() * torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();

            // torque_log_file << time;
            // VectorXd NT_torque_posture(dof);
            // NT_torque_posture = N.transpose() * torque_mid + N.transpose() * torque_damp;
            // for (int i = 0; i < dof; ++i) {
            //     torque_log_file << "," << NT_torque_posture(i);
            // }
            // torque_log_file << std::endl;

            //////////////////////////////////////////////
            //  Problem 2d
            Vector3d xd;
            xd << -0.1, 0.15, 0.2;

            // Task-space control force
            Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            control_torques = Jv.transpose() * F;
            control_torques += N.transpose() * torque_damp + robot->jointGravityVector();

            //////////////////////////////////////////////
            // // Problem 2e
            // Vector3d xd;
            // xd << -0.1, 0.15, 0.2;

            // // Task-space control force
            // Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            // control_torques = Jv.transpose() * F;
            // control_torques += N.transpose() * torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();

            //////////////////////////////////////////////
            // // Problem 2f
            // Vector3d xd;
            // xd << -0.65, -0.45, 0.7;

            // // Task-space control force
            // Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            // control_torques = Jv.transpose() * F;
            // control_torques += N.transpose() * torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();

            ////////////////////////////////////////////
            // // Problem 2g
            // Vector3d xd;
            // xd << -0.65, -0.45, 0.7;

            // // Task-space control force
            // Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            // control_torques = Jv.transpose() * F;
            // control_torques += torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();


        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            double kp = 50.0;
            double kv = 25.0;
            double kvj = 25.0;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);
            Matrix3d R = robot->rotation(link_name);
            Vector3d omega = robot->angularVelocity(link_name);
            MatrixXd J = robot->J(link_name, pos_in_link);

            // desired rotation
            double theta = M_PI / 3.0;
            Matrix3d Rd;
            Rd << 
                std::cos(theta),    0,  std::sin(theta),
                0,                  1,  0,
                -std::sin(theta),   0,  std::cos(theta);

            // desired position
            Vector3d xd;
            xd << 0.6, 0.3, 0.5;

            // delta phi
            Vector3d delta_phi = Vector3d::Zero();
            for (int i=0; i<3; i++){
                delta_phi += R.col(i).cross(Rd.col(i));
            }
            delta_phi  *= -0.5;

            // F
            VectorXd F(6);
            F.head(3) = - kp * (x - xd) - kv * dx;
            F.tail(3) = - kp * (delta_phi) - kv * omega;
            F = Lambda * F;

            // torque
            control_torques = J.transpose() * F;
            control_torques += - N.transpose() * kvj * robot_dq + robot->jointGravityVector();

            // Log delta_phi
            delta_phi_log_file << time << "," 
                            << delta_phi(0) << "," 
                            << delta_phi(1) << "," 
                            << delta_phi(2) << std::endl;



        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            double Vmax = 0.1;
    
            double kp = 200.0;
            double kv = 32.0;
            double kpj = 5.0;
            double kvj = 30.0;

            Vector3d xd(0.6, 0.3, 0.4);

            const double DEG2RAD = M_PI / 180.0;
            VectorXd q_upper(dof);
            q_upper << 165 * DEG2RAD, 100 * DEG2RAD, 165 * DEG2RAD, -30 * DEG2RAD, 165 * DEG2RAD, 210  * DEG2RAD, 165 * DEG2RAD;
            VectorXd q_lower(dof);
            q_lower << -165 * DEG2RAD, -100 * DEG2RAD, -165 * DEG2RAD, -170 * DEG2RAD, -165 * DEG2RAD, 0  * DEG2RAD, -165 * DEG2RAD;
            VectorXd q_desired = (q_upper + q_lower)/2;

            // Current state
            Vector3d x = robot->position(link_name, pos_in_link);
            Vector3d dx = robot->linearVelocity(link_name, pos_in_link);

            ////////////////////////////////////////////
            // Problem 4a
            // Task-space control force
            Vector3d F = Lambda * (-kp * (x - xd) - kv * dx);
            control_torques = Jv.transpose() * F;
            control_torques += N.transpose() * robot->M() * (-kpj * (robot_q - q_desired) - kvj * robot_dq) + robot->jointGravityVector();

            ////////////////////////////////////////////
            // // Problem 4b with velocity saturation
            // Vector3d dxd = - (kp / kv) * (x - xd);
            // double Vxd = dxd.norm();
            // double nu = sat(Vmax/std::abs(Vxd));
            // Vector3d F = Lambda * (-kv * (dx - nu * dxd));
            // control_torques = Jv.transpose() * F;
            // control_torques += N.transpose() * robot->M() * (-kpj * (robot_q - q_desired) - kvj * robot_dq) + robot->jointGravityVector();

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
