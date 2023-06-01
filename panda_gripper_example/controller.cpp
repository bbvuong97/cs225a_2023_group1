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
#include <thread>
#include <chrono>

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
	LOWERHAND = 2,
	GRASP = 3,
	LIFT = 4,
	RETURNTOCENTER = 5,
	TURN180 = 6,
	KEYPRESSMVT = 7,
	START_POS = 8,
	POS_TRACK = 9,
	RELEASE_BALL = 10,
	RETURN_HOME_ARM = 11,
	RETURN_HOME_BASE = 12
};

int main() {

	// initial state 
	int state = BASE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();
	redis_client.set("MOVE_LEFT","0");
	redis_client.set("MOVE_RIGHT","0");
	redis_client.set("NEXT_STATE","0");
	redis_client.set("START_TRACKING","0");
	redis_client.set("Y_Centroid","0");
	redis_client.set("Z_Centroid","0");
	redis_client.set(CHANGE_BALL_RESTITUTION_KEY, "false");

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
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof + 4);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task->_use_interpolation_flag = false;
	posori_task->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 24.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 24.0;
	Matrix3d vert_orient;
	vert_orient << 1, 0, 0,
								 0,1, 0,
								 0, 0,-1;

	// joint task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 24.0;

	VectorXd q_init_desired(7);
	VectorXd qdot_init_desired(7);
	VectorXd initial_q = robot->_q.tail(7);
	Vector3d aboveBallPos;
	Vector3d atBallHeight;

	q_init_desired << 180.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0; //*set init config
	q_init_desired *= M_PI/180.0; //arm joints in radians
	qdot_init_desired << 0, 0, 0, 0, 0, 0, 0;
	joint_task->_desired_position = q_init_desired;
	VectorXd desired_q_throwing(7);
	//desired_q_throwing << 0.0, -15.0, -50.0, -105.0, 0.0, 90.0, 45.0; 
	//desired_q_throwing << 0.0, 0.0, -50.0, -105.0, 0.0, 90.0, 90.0; 
	//desired_q_throwing << 0.0, 0.0, -55.0, -105.0, 0.0, 110.0, 100.0; 
	desired_q_throwing << 0.0, 0.0, -55.0, -105.0, 0.0, 110.0, -97.0; 
	desired_q_throwing *= M_PI/180.0;
	//joint_task->_desired_velocity = qdot_init_desired;


	// gripper task containers
	VectorXd gripper_command_torques(4);
	VectorXd q_gripper(4), dq_gripper(4);
	VectorXd q_gripper_desired(4);
	double kp_gripper = 100;
	double kv_gripper = 20;

	// partial joint task to control the mobile base 
	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = true;
	base_task->_saturation_velocity << 0.2, 0.2, 0.2;  // adjust based on speed
	
	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 400;
	base_task->_kv = 40;
	VectorXd base_pose_desired(3);
	VectorXd base_pose_center(3);
	base_pose_desired << -.25, -.25, 0;
	base_pose_center << 0, 0, 0;
	base_task->_desired_position = base_pose_desired;
	double desired_base_x = 0;
	double max_z_arm = .5; //.78 before
	double max_y_arm = .7; //.5 before

	//camera state variables
	Vector3d ee_pos_init = Vector3d::Zero(3);
	Vector3d zero_offset = Vector3d::Zero(3);
	Vector3d centroid_vec = Vector3d::Zero(3);
	const double scale_factor = .001;
	int cameracounter = 0;
	double y_offset;
	double z_offset;
	double y;
	double z;
	auto start = std::chrono::high_resolution_clock::now();
	auto releasetimer_start = std::chrono::high_resolution_clock::now();

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
				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				// compute torques
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = base_task_torques + joint_task_torques;

				q_gripper_desired << -.05, .05, -.05, .05;
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				if ( (robot->_q.head(3) - base_pose_desired).norm() < 0.005 ) {
	
					cout << "Move to state 1: HAND \n" << endl;
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					state = HAND;
				}
			} else if (state == HAND) {
			
				aboveBallPos << -0.8, -.5, 0.745;
				posori_task->_desired_position = aboveBallPos;
				posori_task->_desired_orientation = vert_orient;
				base_task->_desired_position = base_pose_desired;
				
				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
			  joint_task->updateTaskModel(N_prec);

				// compute torques
				posori_task->computeTorques(posori_task_torques);
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = posori_task_torques + joint_task_torques + base_task_torques;

				q_gripper_desired << -0.12, 0.12, -0.12, 0.12; //*distance between end effectors
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				robot->position(ee_pos, control_link, control_point);

				if ( (ee_pos - aboveBallPos).norm() < 0.005 ) {
					cout << "Move to state 2: LOWERHAND \n" << endl;
					state = LOWERHAND;
				}

			} else if(state == LOWERHAND){
				//cout<<"grasp";
				atBallHeight << -0.8, -.5, 0.35;
				posori_task->reInitializeTask();
				posori_task->_desired_position = atBallHeight;
				posori_task->_desired_orientation = vert_orient;

				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				posori_task->computeTorques(posori_task_torques);
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = posori_task_torques + joint_task_torques + base_task_torques;
				
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				robot->position(ee_pos, control_link, control_point);

				if ( (ee_pos - atBallHeight).norm() < 0.01 ) {
					state = GRASP;
					cout << "Move to state 3: GRASP \n" << endl;
				}
				
			} else if(state == GRASP){
				
				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				posori_task->computeTorques(posori_task_torques);
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = posori_task_torques + joint_task_torques + base_task_torques;
			
				q_gripper_desired.setZero();
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				//gripper_command_torques << 10,-10, 10, -10;
				command_torques.tail(4) = gripper_command_torques;
				//cout<< "\t" << gripper_command_torques.transpose() << "\n";

				cout << "gripper joints "<< (q_gripper).transpose() << "\n";

				if ( (q_gripper(2)-q_gripper(1))<.23 ) {
					state = LIFT;
					cout << "Move to state 4: LIFT \n" << endl;
				}

			} else if(state == LIFT){

				posori_task->reInitializeTask();
				posori_task->_desired_position = aboveBallPos;
				posori_task->_desired_orientation = vert_orient;

				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				posori_task->computeTorques(posori_task_torques);
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = posori_task_torques + joint_task_torques + base_task_torques;
				
				q_gripper_desired.setZero();
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				//gripper_command_torques << 10,-10, 10, -10;
				//cout<< "\t" << gripper_command_torques.transpose() << "\n";
				command_torques.tail(4) = gripper_command_torques;

				robot->position(ee_pos, control_link, control_point);

				if ( (ee_pos - aboveBallPos).norm() < 0.01 ) {
					joint_task->reInitializeTask();
					base_task->reInitializeTask();
					//
					posori_task->reInitializeTask();
					//
					base_task->_desired_position = base_pose_center;
					joint_task->_desired_position = q_init_desired;
					cout << "Move to state 5: RETURNTOCENTER\n" << endl;

					state = RETURNTOCENTER;
				}

			} else if(state == RETURNTOCENTER){

				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				// compute torques
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = base_task_torques + joint_task_torques;

				q_gripper_desired.setZero();
				//q_gripper_desired << -0.03, 0.03, -0.03, 0.03;
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				if ( (robot->_q.head(3) - base_pose_center).norm() < 0.005 ) {
						state = TURN180;
						cout << "Move to state 6: TURN180 \n" << endl;
				}

			} else if(state == TURN180){
				joint_task->_desired_position = desired_q_throwing;
				//joint_task->_desired_velocity = qdot_init_desired;
				//posori_task->_desired_orientation = vert_orient;
				
				joint_task->_use_velocity_saturation_flag = true;
				joint_task->_saturation_velocity << 2,2,2,2,2,2,2;  // adjust based on speed

				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);
		
				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = base_task_torques + joint_task_torques;

				q_gripper_desired.setZero();
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				if (abs(robot->_q(3)-desired_q_throwing(0))<.01){
					state = KEYPRESSMVT;
					base_task->reInitializeTask();
					joint_task->_use_velocity_saturation_flag = false;
					cout << "Move to state 7: KEYPRESSMVT \n" << endl;
				}

			} else if (state==KEYPRESSMVT){

				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				N_prec = base_task->_N;	
			  joint_task->updateTaskModel(N_prec);

				base_task->_desired_position(1) = 0;
				base_task->_desired_position(2) = 0;

				if (redis_client.get("MOVE_LEFT") == "1"){
					base_task->_desired_position(0) -= 0.05;
					redis_client.set("MOVE_LEFT", "0");
				}

				if (redis_client.get("MOVE_RIGHT") == "1"){
					base_task->_desired_position(0) += 0.05;
					redis_client.set("MOVE_RIGHT", "0");
				}

				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = base_task_torques + joint_task_torques;

				//q_gripper_desired << -0.03, 0.03, -0.03, 0.03;
				q_gripper_desired.setZero();
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

				if (redis_client.get("NEXT_STATE") == "1"){
					redis_client.set("NEXT_STATE","0");
					state =  START_POS;
					//posori_task->reInitializeTask();
					//base_task->reInitializeTask();
					
					desired_base_x = robot->_q(0);
					base_task->_desired_position.head(3) = robot->_q.head(3);
					cout << "Move to state 8: START_POS \n" << endl;
				}

			} else if (state==START_POS){

				base_task->computeTorques(base_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques.head(10) = base_task_torques + joint_task_torques;

				if (redis_client.get("START_TRACKING") == "1"){
					y_offset = std::stod(redis_client.get("Y_Centroid"));
					z_offset = std::stod(redis_client.get("Z_Centroid"));
					redis_client.set("START_TRACKING","0");
					cameracounter++;
					base_pose_desired = robot->_q.head(3);

					posori_task->reInitializeTask();
					
					cout << "Move to state 9: POS_TRACK\n" << endl;

					state = POS_TRACK;
				}

				q_gripper_desired.setZero();
				//q_gripper_desired << -0.03, 0.03, -0.03, 0.03;
				gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
				command_torques.tail(4) = gripper_command_torques;

			} else if (state==POS_TRACK){

					auto end = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

					// if (duration >= 2000)
					// {
						y = std::stod(redis_client.get("Y_Centroid"));
						z = std::stod(redis_client.get("Z_Centroid"));
						centroid_vec << 0, y, -1*z;
						zero_offset << 0, y_offset, -1*z_offset;
						
						if (cameracounter == 1)
						{
							robot->position(ee_pos_init, control_link, control_point);
							// cout << "EE X: " << ee_pos_init(0) << "\n";
							// cout << "EE Y: " << ee_pos_init(1)<< "\n";
							// cout << "EE Z: " << ee_pos_init(2)<< "\n";
							cout << "ee_pos_init" << ee_pos_init.transpose() <<"\n";
							cameracounter++;
						}
						
						N_prec.setIdentity();
						base_task->updateTaskModel(N_prec);
						N_prec = base_task->_N;	
						posori_task->updateTaskModel(N_prec);
						N_prec = posori_task->_N;
						joint_task->updateTaskModel(N_prec);

						base_task->_desired_position = base_pose_desired;
						posori_task->_desired_position = ee_pos_init + ((centroid_vec - zero_offset) * scale_factor);
						posori_task->_desired_orientation = vert_orient;

						cout << "Difference: " << (centroid_vec - zero_offset).transpose() << "\n";

						posori_task->computeTorques(posori_task_torques);
						base_task->computeTorques(base_task_torques);
						joint_task->computeTorques(joint_task_torques);

						command_torques.head(10) = posori_task_torques + base_task_torques + joint_task_torques;

						gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
						command_torques.tail(4) = gripper_command_torques;

						robot->position(ee_pos, control_link, control_point);

						cout << "ee_pos" << ee_pos.transpose() <<"\n";

						//if ((abs(ee_pos(2)-max_z_arm)<=.005)||(abs(ee_pos(1)-max_y_arm)<=.005)){
						if (abs(ee_pos(1)-max_y_arm)<=.005){
							
							cout << "Move to state 10: RELEASE_BALL\n" << endl;
							releasetimer_start = std::chrono::high_resolution_clock::now();

							redis_client.set(CHANGE_BALL_RESTITUTION_KEY, "true");

							state = RELEASE_BALL;

							//test
							// posori_task->computeTorques(posori_task_torques);
							// base_task->computeTorques(base_task_torques);
							// joint_task->computeTorques(joint_task_torques);

							// command_torques.head(10) = posori_task_torques + base_task_torques + joint_task_torques;

							// gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
							// command_torques.tail(4) = gripper_command_torques;

							//test
						}
					//}
			} else if (state==RELEASE_BALL){

					posori_task->computeTorques(posori_task_torques);
					base_task->computeTorques(base_task_torques);
					joint_task->computeTorques(joint_task_torques);

					command_torques.head(10) = posori_task_torques + base_task_torques + joint_task_torques;
					
					q_gripper_desired << -0.2, 0.2, -0.2, 0.2;
					gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
					command_torques.tail(4) = gripper_command_torques;

					auto releasetimer_end = std::chrono::high_resolution_clock::now();
					auto releasetimer_duration = std::chrono::duration_cast<std::chrono::milliseconds>(releasetimer_end - releasetimer_start).count();

					if (releasetimer_duration>=10000){
						cout << "Move to state 11: RETURN_HOME_ARM\n" << endl;

						joint_task->reInitializeTask();
						joint_task->_desired_position = q_init_desired;
						//joint_task->_desired_position(0) = 0;
						//q_gripper_desired.setZero();
						base_task->_desired_position = robot->_q.head(3);

						state = RETURN_HOME_ARM;
					}


			} else if (state==RETURN_HOME_ARM){

					N_prec.setIdentity();
					base_task->updateTaskModel(N_prec);
					N_prec = base_task->_N;	
			  	joint_task->updateTaskModel(N_prec);

					base_task->computeTorques(base_task_torques);
					joint_task->computeTorques(joint_task_torques);

					command_torques.head(10) = base_task_torques + joint_task_torques;
						
					//q_gripper_desired << -0.12, 0.12, -0.12, 0.12;
					gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
					command_torques.tail(4) = gripper_command_torques;

					if ( (robot->_q.segment(3,7) - q_init_desired).norm() < 0.005 ) {
						cout << "Move to state 12: RETURN_HOME_BASE\n" << endl;

						base_task->reInitializeTask();
						base_task->_desired_position = base_pose_center;
						q_gripper_desired.setZero();
						state = RETURN_HOME_BASE;
					}

			} else if (state==RETURN_HOME_BASE){

					N_prec.setIdentity();
					base_task->updateTaskModel(N_prec);
					N_prec = base_task->_N;	
			  	joint_task->updateTaskModel(N_prec);

					// compute torques
					base_task->computeTorques(base_task_torques);
					joint_task->computeTorques(joint_task_torques);
					command_torques.head(10) = base_task_torques + joint_task_torques;

					gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
					command_torques.tail(4) = gripper_command_torques;
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
