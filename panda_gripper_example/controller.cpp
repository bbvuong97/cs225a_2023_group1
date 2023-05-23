/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";

enum State 
{
	BASE = 0, 
	HAND = 1,
	GRASP = 2
};

int main() {

	// initial state 
	int state = BASE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	//robot->_q.head(7) = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);

	//robot->_dq.head(7) = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	cout << "\t joints" << robot->_q.transpose() << "\n";
	robot->updateModel();


	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof + 2);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	VectorXd q_init_desired(dof);
	VectorXd qdot_init_desired(dof);
	q_init_desired << -.2, -.2, 0, -30.0, -15.0, -15.0, -50.0, 0.0, 45.0, 20.0; //*set init config, 3dof base (prismatic), then 7dof arm
	q_init_desired.tail(7) *= M_PI/180.0; //arm joints in radians
	qdot_init_desired << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	joint_task->_desired_position = q_init_desired;
	// joint_task->_desired_velocity = qdot_init_desired;


	// gripper task containers
	VectorXd gripper_command_torques(2);
	VectorXd q_gripper(2), dq_gripper(2);
	VectorXd q_gripper_desired(2);
	q_gripper_desired.setZero();
	double kp_gripper = 100;
	double kv_gripper = 20;


	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_ANGLES_KEY, q_gripper);
	redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_VELOCITIES_KEY, dq_gripper);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	while (runloop) {
		// fTimerDidSleep = timer.waitForNextLoop(); // commented out to let current controller loop finish before next loop

		// read simulation state
		fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

		// run controller loop when simulation loop is done
		if (fSimulationLoopDone) {
			// execute redis read callback
			redis_client.executeReadCallback(0);

			// update model
			robot->updateModel();
		
			if (state == BASE) {
				// update task model and set hierarchy
				//cout<<"base";
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				// compute torques
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = joint_task_torques;

				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(2) = gripper_command_torques;

				if ( (robot->_q - q_init_desired).norm() < 0.005 ) {
					cout << "Posture To Motion" << endl;
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					robot->position(ee_pos, control_link, control_point);
					posori_task->_desired_position = Vector3d(-0.8, -.5, .345); //*desired end effector position
					posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation; //*desired orientation
					// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
					q_gripper_desired << -0.1, 0.1; //*distance between end effectors

					state = HAND;
				}
			} else if (state == HAND) {
				//cout<<"hand";
				// update task model and set hierarchy
				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				joint_task->updateTaskModel(N_prec);

				// compute torques
				posori_task->computeTorques(posori_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = posori_task_torques + joint_task_torques;

				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(2) = gripper_command_torques;

			} else if(state == GRASP){
				cout<<"grasp";

			}

			// execute redis write callback
			redis_client.executeWriteCallback(0);	

			counter++;
		}
		// controller loop is done
		fControllerLoopDone = true;
		redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
	}

	// controller loop is turned off
	fControllerLoopDone = false;
	redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
    assert(x == "false" || x == "true");
    return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
    return b ? "true" : "false";
}
