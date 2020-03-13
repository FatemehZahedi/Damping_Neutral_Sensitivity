#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h> // strstr
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>
#include "UdpServer.h"
/* EMG */
#include "TrignoEmgClient.h"
#include <algorithm>
/* Boost filesystem */
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
/* HDF5 */
#include "H5Cpp.h"


using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;
using namespace boost::filesystem;
//using namespace H5;

#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection
#define SHM_SIZE 1024
#define DEFAULT_TRIAL_NUMBER 0
#define DEFAULT_SUB_NUM 0
#define DEFAULT_GP_NUM 1

//FILE *NewDataFile(void);

/* Shared Memory Function Prototype */
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements);
/*int *MakeFloatSharedMemory(int HowBig);
float *MakeFloatSharedMemory2(int HowBig2);*/

void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath);

/* GUI UDP Server Address/Port */
const std::string 	udp_addr_gui("192.168.0.102");
const int 			udp_port_gui = 50000;


//------------------------------------------------------------------------------------------------------------

// Main
int main(int argc, char** argv)
{

	// UDP Server address and port hardcoded now -- change later
	UDPServer udp_server(udp_addr_gui, udp_port_gui);



	//******************---------------------------------------
	//---------------------------------------
	/*double mag = 0.025;
	double dur = 150;*/
	double mag = 0.055;
	double dur = 330;
	double subs = 65;
	double ftx;
	double fty;
	double ftx_un;
	double fty_un;
	double zerox = 0;
	double zeroy = 0;
	double ftx_0 = 0.0;
	double fty_0 = 0.0;
	/*int *data;	//pointer to shared memory
	data = MakeFloatSharedMemory(2);
	float *data2; //pointer to shared memory
	data2 = MakeFloatSharedMemory2(2);*/
	
	/* Shared Memory Setup */
	std::string shmAddr("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile");
	int shmNumElements = 2;
	int * data = InitSharedMemory<int>(shmAddr, shmNumElements);
	
	int cc = 1;
	double al = 0.5;
	int firstIt = 0;
	int trigger = 1;
	int w = 0;
	double random_num;
	double ft[2];
	int steady = 0;
	int steady2 = 0;
	int steady3 = 0;
	int steady4 = 0;
	int perturb_flag = 0;
	int rr = 1;

	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;

	int failure_flag = 0;
	int flag_finish = 0;
	int flag_ex = 0;
	int flag_I = 1;
	int flag_p = 0;
	int flag_p2 = 0;
	int fi1 = 0;
	int fi2 = 0;
	int fi3 = 0;
	int fs = 0;
	int mode = 1;
	int n_v;

	int emg_flag = 0;
	int emg_flag_first = 1;

	double xy_coord[16];
	memset(xy_coord, 0, sizeof(double) * 16);

	//-----------------------------------------------
	//********************-----------------------------------------------



	// DH Parameter----------------------------------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;



	//******************---------------------------------------
	//---------------------------------------
	// May be changed-------------
	MatrixXd x_0(6, 1); x_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_test(6, 1); x_test << 0, 0, 0, 0, 0, 0;
	MatrixXd x_00(6, 1); x_00 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr(6, 1); x_incr << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr_0(6, 1); x_incr_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd test_new(6, 1); test_new << 0, 0, 0, 0, 0, 0;


	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;
	MatrixXd q_freeze(6, 1); q_freeze << 0, 0, 0, 0, 0, 0;

	MatrixXd q_0(6, 1); q_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_0(6, 1); t_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;
	MatrixXd q_delay(6, 1); q_delay << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;
	MatrixXd v(6, 1); v << 0, 0, 0, 0, 0, 0;
	MatrixXd acc(6, 1); acc << 0, 0, 0, 0, 0, 0;
	MatrixXd Fnew(6, 1); Fnew << 0, 0, 0, 0, 0, 0;
	MatrixXd q_old(6, 1); q_old << 0, 0, 0, 0, 0, 0;

	/////////////////
	MatrixXd x_new2(6, 1); x_new2 << 0, 0, 0, 0, 0, 0;
	MatrixXd Fnew2(6, 1); Fnew2 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_02(6, 1); x_02 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_002(6, 1); x_002 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_03(6, 1); x_03 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_003(6, 1); x_003 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new3(6, 1); x_new3 << 0, 0, 0, 0, 0, 0;
	/////////////////

	//-------------------------------------------------
	//*****************************--------------------------
	//points and values-------------------------------------

	double radius_e = 0.005;
	double rangex_ = -0.18;
	double rangex = 0.18;
	double rangey_ = 0.58;
	double rangey = 0.94;
	double d_r = (rangex * 2) / 20;
	double u_r = d_r - radius_e;
	double ex_r = d_r / 4;

	int t_r;
	MatrixXd time_ran(7, 1); time_ran << 0.7, 1, 0.9, 1.2, 0.8, 1.4, 0.5;

	int subject_num;
	int group_num;
	int chosen;
	int i_c;
	int j_c;
	int chosen_damping;
	int chosen_point = 0;
	int flag_chosen = 1;
	int flag_v = 0;
	int random_v;
	MatrixXd desired(2, 1); desired << 0, 0;
	MatrixXd point(2, 1); point << 0, 0;
	MatrixXd P_ex(2, 1); P_ex << 0, 0;
	MatrixXd Vector(40, 1); Vector << MatrixXd::Zero(40, 1);
	
	
	MatrixXd damping_values1(4, 5); damping_values1 << 0, -10, -15, -20, -25,
							 0, -10, -15, -20, -25,
							 0, -10, -15, -20, -25,
							 0, -10, -15, -20, -25;
	
	MatrixXd damping_values2(4, 5); damping_values2 << 0, -20, -30, -40, -50,
							  0, -20, -30, -40, -50,
							  0, -20, -30, -40, -50,
							  0, -20, -30, -40, -50;
	
	MatrixXd damping_values(4, 5);
	    

	MatrixXd random1(40, 1); random1 << 16, 2, 31, 28, 39, 3, 1, 35, 21, 11, 38, 7, 27, 32, 17, 22, 14, 8, 5, 19, 36, 37, 33, 15, 6, 30, 29, 4, 23, 18, 24, 34, 13, 25, 9, 26, 20, 40, 10, 12;
	
	MatrixXd pert_dir1(20, 1); pert_dir1 << 1, 2, 2, 2, 1, 2, 1, 2, 1, 1, 1, 2, 2, 1, 1, 1, 2, 2, 1, 1;
	MatrixXd pert_dir2(20, 1); pert_dir2 << 2, 1, 1, 1, 2, 1, 2, 1, 2, 2, 2, 1, 1, 2, 2, 2, 1, 1, 2, 2;
	MatrixXd pert_dir(20, 1); pert_dir << MatrixXd::Zero(20, 1);
	
	//MatrixXd random2(40, 1); random2 << 6, 27, 21, 4, 2, 5, 15, 24, 3, 8, 28, 22, 12, 11, 17, 16, 18, 23, 10, 39, 1, 19, 20, 30, 13, 25, 14, 7, 9, 26;
	//MatrixXd random3(40, 1); random3 << 1, 25, 9, 17, 14, 23, 26, 18, 12, 2, 30, 10, 27, 16, 20, 24, 3, 7, 28, 19, 4, 8, 13, 11, 5, 15, 6, 21, 22, 29;
	//----------------------------------------------------------
	// Mass and velocity setting

	MatrixXd Mass(1, 4); Mass << 8, 10, 12, 14;
	
	MatrixXd coeff_values(4, 3); coeff_values << 10, 15, 6, //200 mm/s
						    8.58, 14, 6, // 110 mm/s
						    8.73, 14, 6, //150mm/s
						    9.3, 14, 6; // 250 mm/s
						    
	MatrixXd magnitude(1, 4); magnitude << 0.055, 0.0832, 0.074, 0.0521;
	MatrixXd coeff(1, 3); coeff << 10, 15, 6;
	
	// Environmental values
	// Initializing Stiffness Damping and Inertia

	MatrixXd stiffness(6, 6); stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
						0, 10000000, 0, 0, 0, 0, //up
						0, 0, 0, 0, 0, 0, //out toward workshop
						0, 0, 0, 1000000, 0, 0,
						0, 0, 0, 0, 1000000, 0,
						0, 0, 0, 0, 0, 1000000;

	MatrixXd damping(6, 6); damping << 30, 0, 0, 0, 0, 0,
					    0, 100, 0, 0, 0, 0,
					    0, 0, 30, 0, 0, 0,
					    0, 0, 0, 0.5, 0, 0,
					    0, 0, 0, 0, 0.5, 0,
					    0, 0, 0, 0, 0, 0.5;

	MatrixXd inertia(6, 6); inertia << 10, 0, 0, 0, 0, 0,
					    0, 0.000001, 0, 0, 0, 0,
					    0, 0, 10, 0, 0, 0,
					    0, 0, 0, 0.0001, 0, 0,
					    0, 0, 0, 0, 0.0001, 0,
					    0, 0, 0, 0, 0, 0.0001;



	//******************---------------------------------
	int sample = 200;
	MatrixXd tdata(6, sample); tdata.setZero(6, sample);
	//***********************-----------------------------


	//*********************-----------------------------------
	// First angles of Kuka
	MatrixXd qdata(6, sample); qdata.setZero(6, sample);//Initializing joint angles

														
	MatrixXd t_sum(6, 1); t_sum.setZero(6, 1);
	MatrixXd q_sum(6, 1); q_sum.setZero(6, 1);
	MatrixXd t_ave(6, 1); t_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd q_ave(6, 1); q_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd torques_0(6, 1); torques_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_e(6, 1); t_e << 0, 0, 0, 0, 0, 0;
	//***********************-----------------------------

	//**********************-----------------------------
	// Not sure what it is
	struct timespec start2, finish2;
	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);
	//*********************------------------------------

	// parse command line arguments
	if (argc < 3)
	{
		
		printf("Use: DampingNeutralSensitivityMV <pertdir> <randomNumber> <opt: trialNumber>\n");
		return 1;
	}

	int pertdir = atoi(argv[1]);
	int randNum = atoi(argv[2]);
	//int n_v = (argc >= 4) ? atoi(argv[3]) : (int) DEFAULT_TRIAL_NUMBER;

	int trialNumber;
	std::string emgIpAddr;
	bool useEmg = false;
	bool trialNumberInputted = false;
	bool subInputted = false;
	bool gpInputted = false;

	std::string argKey;
	std::string argVal;
	for (int iArg = 3; iArg < (argc - 1); iArg += 2) 
	{
		/* get key and val */
		argKey = std::string(argv[iArg]);
		argVal = std::string(argv[iArg + 1]);
		/* set key uppercase */
		std::transform(argKey.begin(), argKey.end(), argKey.begin(), ::toupper);

		if (argKey.compare("TN") == 0) 
		{
			trialNumberInputted = true;
			n_v = std::stoi(argVal);
		}
		else if (argKey.compare("EMG") == 0) 
		{
			useEmg = true;
			emgIpAddr = argVal;
		}
		else if (argKey.compare("SUB") == 0) {
			subInputted = true;
			subject_num = std::stod(argVal);
		}
		else if (argKey.compare("GP") == 0) {
			gpInputted = true;
			group_num = std::stod(argVal);
		}
		else 
		{
			printf("Key: %s not understood\n", argKey.c_str());
		}
	}

	/* Check subject number */
	if (!subInputted)
	{
		subject_num = (int) DEFAULT_SUB_NUM;
	}
	
	/* Check group number */ // group_num 1 means left ans right, group_num 2 means up and down
	if (!gpInputted)
	{
		group_num = (int) DEFAULT_GP_NUM;
	}

	/* Check Inputted Trial Number */
	if (!trialNumberInputted) 
	{
		n_v = (int) DEFAULT_TRIAL_NUMBER;
	}

	/* Check EMG use */
	TrignoEmgClient emgClient;
	if (useEmg) 
	{
		emgClient.SetIpAddress(emgIpAddr);
		emgClient.ConnectDataPort();
		emgClient.ConnectCommPort();
		if (emgClient.IsCommPortConnected()) 
		{
			/* Check if sensors are paired */
			emgClient.IsSensorPaired(1);
			emgClient.IsSensorPaired(2);
			emgClient.IsSensorPaired(3);
			emgClient.IsSensorPaired(4);
			emgClient.IsSensorPaired(5);
			emgClient.IsSensorPaired(6);

			emgClient.SendCommand(1); // this command signals the emg server to send readings to Data Port
			std::thread emgReceiveThread(&TrignoEmgClient::ReceiveDataStream, &emgClient);
			emgReceiveThread.detach();
		}
	}

	/* Paths for data files */
	path p_base = current_path();
	std::string Datafilename = string("KD_S") + std::to_string(subject_num) + string("_B") + std::to_string(randNum) + string(".txt");
	
	// subject directory
	std::string subjectDir = "Subject" + std::to_string(subject_num);
	path p_subject = path(p_base.string()) /= path(subjectDir);
	create_directory(p_subject);
	
	// group directory
	std::string groupDir = "Group" + std::to_string(group_num);
	path p_group= path(p_subject.string()) /= path(groupDir);
	create_directory(p_group);
	
	std::string emgDir = "EMGData" + std::to_string(randNum);
	path p_emgdata = path(p_group.string()) /= path(emgDir);
	if (useEmg)
	{
	    create_directory(p_emgdata);
	}
	path p_kukadata;
	path p_emg;

	boost::filesystem::ofstream OutputFile;


	const char* hostname =  DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
	int port = DEFAULT_PORTID; //optional comand line argument for port


	bool success = true;
	bool success2 = true;

	int count = 0;
	int enough = 0;
	int i = 0;


	int stime = 0;
	float sampletime = 0; //will be determined from FRI
	double MJoint[7] = { 0 };//{-1.776415,1.025433,-0.064599,1.656986,0.290444,-0.971846,-0.223775}; //last measured joint position (not sure time when controller measured, be careful using this for feedback loop)
	double ETorque[7] = { 0 }; //External torque :supposedly the torque applied to the robot (subtracts the torque due to the robot, ackording to Kuka)

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)
																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };//will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits in radians (can be changed to further restrict motion of robot)
	double FirstPositionDelta[7] = { 0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175 }; //maximum deviation from initial position in trajectory from start position in robot(radians)


																						 //Get value for the time step of the command cycle (used for determining max velocity)
																						 // sampletime=client.GetTimeStep();
	sampletime = 0.001;
	fprintf(stdout, "Sample Time:%f seconds\n", sampletime);

	//calculate max step value
	for (i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime * MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}


	// create new joint position client
	PositionControlClient client;
	client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);




	// create new udp connection
	UdpConnection connection;


	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);




	//create file for output
	/*OutputFile = NewDataFile();*/
	p_kukadata = path(p_group.string()) /= path(Datafilename);
	CreateOrOpenKukaDataFile(OutputFile, p_kukadata);


	client.NextJoint[0] = -1.5708;
	client.NextJoint[1] = 1.5708;
	client.NextJoint[2] = 0;
	client.NextJoint[3] = 1.5708;
	client.NextJoint[4] = 0;
	client.NextJoint[5] = -1.5708;
	client.NextJoint[6] = -0.958709;


	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));

	//*******************************---------------------------------------
	// I don't understand them
	while (!enough)
	{
		clock_gettime(CLOCK_MONOTONIC, &start2);
		//---------------------------timestamp----------------------------------
		gettimeofday(&end, NULL);
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		//----------------------------------------------------------------------


		success = app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0
			if (count == 1)//first time inside
			{
				sampletime = client.GetTimeStep();
				//calculate max step value
				for (i = 0; i < 7; i++)
				{
					client.MaxRadPerStep[i] = sampletime * MaxRadPerSec[i]; //converts angular velocity to discrete time step
				}

			}
			//*********************-------------------------------------------------------------


			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7); //gets the most recent measured joint value
			memcpy(ETorque, client.GetExtTor(), sizeof(double) * 7);//gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)

																	// Forward Kinematic
			theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

			MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
				sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
				0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
				0, 0, 0, 1;
			MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
				sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
				0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
				0, 0, 0, 1;
			MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
				sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
				0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
				0, 0, 0, 1;
			MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
				sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
				0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
				0, 0, 0, 1;
			MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
				sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
				0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
				0, 0, 0, 1;
			MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
				sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
				0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
				0, 0, 0, 1;

			MatrixXd T01(4, 4); T01 << A1;
			MatrixXd T02(4, 4); T02 << T01 * A2;
			MatrixXd T03(4, 4); T03 << T02 * A3;
			MatrixXd T04(4, 4); T04 << T03 * A4;
			MatrixXd T05(4, 4); T05 << T04 * A5;
			MatrixXd T06(4, 4); T06 << T05 * A6;


			//*******************--------------------------

			point(0) = T06(0, 3);
			point(1) = T06(2, 3);

			//*******************----------------------------

			// Inverse Kinematic

			phi_euler = atan2(T06(1, 2), T06(0, 2));
			theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
			psi_euler = atan2(T06(2, 1), -T06(2, 0));

			MatrixXd z0(3, 1); z0 << 0, 0, 1;
			MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
			MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
			MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
			MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
			MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
			MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

			MatrixXd p0(3, 1); p0 << 0, 0, 0;
			MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
			MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
			MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
			MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
			MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
			MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

			MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
				-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
				z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
				z0(0, 0), z0(1, 0), z0(2, 0);
			MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
				-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
				z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
				z1(0, 0), z1(1, 0), z1(2, 0);
			MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
				-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
				z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
				z2(0, 0), z2(1, 0), z2(2, 0);
			MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
				-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
				z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
				z3(0, 0), z3(1, 0), z3(2, 0);
			MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
				-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
				z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
				z4(0, 0), z4(1, 0), z4(2, 0);
			MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
				-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
				z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
				z5(0, 0), z5(1, 0), z5(2, 0);

			MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6;

			//***********

			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
				0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
				0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;

			//*****************


			//**************************--------------------
			if (firstIt == 0)//first time inside
			{
				firstIt = 1;
				std::cout << firstIt << std::endl;

				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_03 << x_e;
				x_003 << x_e;
				x_new << x_e;
			}
			//**********************------------------------

			//*********************-------------------------
			//??
			ftx = (double)data[0] / 1000000 - zerox; //toward varun desk
			ftx_un = (double)data[0] / 1000000 - zerox;

			fty = (double)data[1] / 1000000 - zeroy;
			fty_un = (double)data[1] / 1000000 - zeroy;


			ftx = al * ftx + (1 - al)*ftx_0;
			ftx_0 = ftx;
			fty = al * fty + (1 - al)*fty_0;
			fty_0 = fty;




			force << ftx, 0 , fty, 0, 0, 0;

			steady = steady + 1;

			perturb_flag = 0;

			//**********************---------------------------

			if (flag_I == 1)
			{
				damping(0, 0) = 30;
				damping(2, 2) = 30;
				flag_I = 0;
				failure_flag = 0;
			}

			if (flag_v == 1)
			{
				flag_v = 0;
				flag_finish = 1;
				q_freeze << q_new;
			}
			/*if (flag_v == 1)
			{
			random_v = rand() % 3 + 1;
			flag_v = 0;
			n_v = 0;
			if (random_v == 3)
			{
			Vector << random3;
			}
			else if (random_v == 2)
			{
			Vector << random2;
			}
			else
			{
			Vector << random1;
			}
			}*/
			
			Vector << random1;

			/*if (vectorNum == 1)
			{
			  Vector << random1;
			}
			else if (vectorNum == 2)
			{
			  Vector << random2;
			}
			else if (vectorNum == 3)
			{
			  Vector << random3;
			}
			else
			{
			  printf("You done messed up! Vector number needs to be between 1-3.\n");
			  exit(1);
			}*/
			
			if (pertdir == 1)
			{
			  pert_dir << pert_dir1;
			}
			else if (pertdir == 2)
			{
			  pert_dir << pert_dir2;
			}
			else
			{
			  printf("You done messed up! perturbation direction needs to be 1 or 2. \n");
			  exit(1);
			}

			

			if (flag_chosen == 1)
			{
				printf("Trial Number: %d\n", n_v);
				chosen = Vector(n_v);
				if (chosen % 5 == 0)
				{
					i_c = chosen / 5 - 1;
					j_c = chosen % 5 + 4;
				}
				else
				{
					i_c = chosen / 5;
					j_c = chosen % 5 - 1;
				}
				
				if (chosen <= 20)
				{
				  damping_values << damping_values1;
				}
				else
				{
				  i_c = i_c - 4;
				  damping_values << damping_values2;
				}

				chosen_damping = damping_values(i_c, j_c);
				if (group_num == 1)
				{
				  inertia(0, 0) = Mass(i_c);
				  inertia(2, 2) = Mass(i_c);
				}
				else
				{
				  coeff << coeff_values.row(i_c);
				  mag = magnitude(i_c);
				}
				//chosen_point = i_c;

				/*std::cout << "Point" << std::endl;
				std::cout << chosen_point << std::endl;*/
				std::cout << "B" << std::endl;
				std::cout << chosen_damping << std::endl;

				flag_chosen = 0;
			}
			//********************-----------------------------

			std::string emgfilename = string("ED_S") + std::to_string(subject_num) + string("_B") + std::to_string(randNum) + string("_T") + std::to_string(chosen) + string(".txt");

			// Defining each point

			if (chosen_point == 0)
			{
				desired(0) = 0;
				desired(1) = 0.76;
			}
			else if (chosen_point == 1)
			{
				desired(0) = 0.06;
				desired(1) = 0.82;
			}
			else if (chosen_point == 2)
			{
				desired(0) = -0.06;
				desired(1) = 0.82;
			}
			else if (chosen_point == 3)
			{
				desired(0) = 0.06;
				desired(1) = 0.7;
			}
			else if (chosen_point == 4)
			{
				desired(0) = -0.06;
				desired(1) = 0.7;
			}


			//-----------------------------------------------------------

			if (steady > 2000 && -radius_e <= point(0) - desired(0) && point(0) - desired(0) <= radius_e)
			{
				if (-radius_e <= point(1) - desired(1) && point(1) - desired(1) <= radius_e)
				{
					steady4++;
					if (steady4 > 1000)
					{
						if (chosen <= 20)
						{
							damping(0, 0) = chosen_damping;
						}
						else
						{
							damping(2, 2) = chosen_damping;
						}
						flag_ex = 1;
						//flag_p = 0;
						/*std::cout << "damping" << std::endl;
						std::cout << damping(0,0) << std::endl;*/

					}

				}
			}
			/*std::cout << "position" << std::endl;
			std::cout << point << std::endl;*/

			if (steady < 2000)
			{
				force << 0, 0, 0, 0, 0, 0;

				zerox = al * (double)data[0] / 1000000 + (1 - al)*zerox;
				zeroy = al * (double)data[1] / 1000000 + (1 - al)*zeroy;

				q_freeze << q_new;

			}
			// Step 2--------------------
			//
			/*if (steady > 6000)
			{
			damping(0, 0) = 0;
			damping(2, 2) = 0;
			}*/
			//
			//----------------------------
			/*std::cout << "mode" << std::endl;
			std::cout << mode << std::endl;*/

			// Step 4-------------------------------------
			if (w == dur - subs + 1)
			{
				trigger = 1;
				q_freeze << q_new;
				delta_q << 0, 0, 0, 0, 0, 0;
				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_new << x_e;
				w = 0;
				flag_p2 = 1;
			}
			if (chosen <= 20)
			{
				if (flag_ex == 1 && flag_p == 0 && fi1 == 0)
				{
					mode = 2;
					P_ex(0) = desired(0) + 0.05;
					P_ex(1) = desired(1);
				}
				else if (flag_p == 1 && fi2 == 0)
				{
					mode = 2;
					P_ex(0) = desired(0) - 0.05;
					P_ex(1) = desired(1);
				}
				else
				{
					mode = 1;
					P_ex(0) = desired(0);
					P_ex(1) = desired(1);
				}

				// Step 3 -----------------------------------------------
				if (flag_ex == 1)
				{
					if (fi1 == 0 && point(0) - desired(0) >= 0.05)
					{
						fi1 = 1;
						flag_p++;
						fs = 1;
					}
					if (fs == 1 && fi2 == 0 && point(0) - desired(0) <= -0.05)
					{
						fi2 = 1;
						flag_p++;
						fs = 0;
					}

					if (fi3 == 0 && flag_p == 2 && -radius_e <= point(0) - desired(0) && point(0) - desired(0) <= radius_e)
					{
						fi3 = 1;
						flag_p++;
					}

					if (point(0) - desired(0) >= 0.13 || point(0) - desired(0) <= -0.13)
					{
						failure_flag = 1;
						flag_I = 1;
						flag_chosen = 1;
						n_v++;
						flag_p = 0;
						flag_p2 = 0;
						fi1 = 0;
						fi2 = 0;
						fi3 = 0;
						steady2 = 0;
						steady3 = 0;
						steady4 = 0;
						cc = 1;
						flag_ex = 0;
					}
				}
			}
			else
			{
				if (flag_ex == 1 && flag_p == 0 && fi1 == 0)
				{
					mode = 2;
					P_ex(1) = desired(1) + 0.05;
					P_ex(0) = desired(0);
				}
				else if (flag_p == 1 && fi2 == 0)
				{
					mode = 2;
					P_ex(1) = desired(1) - 0.05;
					P_ex(0) = desired(0);
				}
				else
				{
					mode = 1;
					P_ex(0) = desired(0);
					P_ex(1) = desired(1);
				}

				// Step 3 -----------------------------------------------
				if (flag_ex == 1)
				{
					if (fi1 == 0 && point(1) - desired(1) >= 0.05)
					{
						fi1 = 1;
						flag_p++;
						fs = 1;
					}
					if (fs == 1 && fi2 == 0 && point(1) - desired(1) <= -0.05)
					{
						fi2 = 1;
						flag_p++;
						fs = 0;
					}

					if (fi3 == 0 && flag_p == 2 && -radius_e <= point(1) - desired(1) && point(1) - desired(1) <= radius_e)
					{
						fi3 = 1;
						flag_p++;
					}

					if (point(1) - desired(1) >= 0.13 || point(1) - desired(1) <= -0.13)
					{
						failure_flag = 1;
						flag_I = 1;
						flag_chosen = 1;
						n_v++;
						flag_p = 0;
						flag_p2 = 0;
						fi1 = 0;
						fi2 = 0;
						fi3 = 0;
						steady2 = 0;
						steady3 = 0;
						steady4 = 0;
						cc = 1;
						flag_ex = 0;
					}
				}
			}
			


			if (flag_p < 3)
			{
				if (chosen <= 20)
				{
				  random_num = pert_dir(chosen-1);
				}
				else
				{
				  random_num = pert_dir(chosen-21) + 2;
				}
				t_r = rand() % 6 + 0;
			}

			if (flag_p == 3)
			{
				steady2++;
				flag_ex = 0;
			}



			if (flag_p == 3 && steady2 >= time_ran(t_r) * 1000 && cc == 1 && steady > 2000)
			{
				

				if (trigger == 1)
				{
					trigger = 0;
					q_freeze << q_new;
					delta_q << 0, 0, 0, 0, 0, 0;
					x_incr_0 << 0, 0, 0, 0, 0, 0;
				}

				if (random_num == 1)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << -mag * (coeff(0)*pow(w / dur, 3.0) - coeff(1)*pow(w / dur, 4.0) + coeff(2)*pow(w / dur, 5.0)), 0, 0, 0, 0, 0;
						perturb_flag = 7;
						if (w == dur - subs)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;

				}
				// next number----------------------------------
				if (random_num == 2)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << mag * (coeff(0)*pow(w / dur, 3.0) - coeff(1)*pow(w / dur, 4.0) + coeff(2)*pow(w / dur, 5.0)), 0, 0, 0, 0, 0;
						perturb_flag = 7;
						if (w == dur - subs)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;

				}
				
				// next number----------------------------------
				if (random_num == 3)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << 0, 0, mag * (coeff(0)*pow(w / dur, 3.0) - coeff(1)*pow(w / dur, 4.0) + coeff(2)*pow(w / dur, 5.0)), 0, 0, 0;
						perturb_flag = 7;
						if (w == dur - subs)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;
				}
				// next number----------------------------------
				if (random_num == 4)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << 0, 0, -mag * (coeff(0)*pow(w / dur, 3.0) - coeff(1)*pow(w / dur, 4.0) + coeff(2)*pow(w / dur, 5.0)), 0, 0, 0;
						perturb_flag = 7;
						if (w == dur - subs)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;
				}

				w = w + 1;
			}
			//--------------------------------------------------------------

			stime = client.GetTimeStamp();

			// Step 5-----------------------------------------

			if (flag_p2 == 1)
			{
				steady3++;
			}
			if (steady3 > 3000)
			{
				flag_I = 1;
				flag_chosen = 1;
				n_v++;
				steady3 = 0;
				flag_p2 = 0;
				flag_p = 0;
				fi1 = 0;
				fi2 = 0;
				fi3 = 0;
				cc = 1;
				steady2 = 0;
				steady4 = 0;
				emg_flag_first = 1;
				if (useEmg)
				{
					emgClient.StopWritingFileStream();
				}
			}

			if (n_v == Vector.rows())
			{
				n_v = 0;
				flag_v = 1;
			}
			//----------------------------------------------
			if (trigger == 0)
			{
				if (emg_flag_first == 1)
				{
					emg_flag = 1;
					emg_flag_first = 0;
				}
				
			}
			if (emg_flag == 1)
			{
				if (useEmg)
				{
					p_emg = path(p_emgdata.string()) /= path(emgfilename);
					emgClient.StartWritingFileStream(p_emg);
				}
				emg_flag = 0;
			}
			if (trigger == 0 || flag_p2 == 1)
			{
				if (point(0) - desired(0) >= 0.13 || point(0) - desired(0) <= -0.13 || point(1) - desired(1) >= 0.13 || point(1) - desired(1) <= -0.13)
				{
					failure_flag = 1;
					flag_I = 1;
					flag_chosen = 1;
					n_v++;
					flag_p = 0;
					flag_p2 = 0;
					fi1 = 0;
					fi2 = 0;
					fi3 = 0;
					steady2 = 0;
					steady3 = 0;
					steady4 = 0;
					cc = 1;
					flag_ex = 0;
					emg_flag_first = 1;
					if (useEmg) 
					{
						emgClient.StopWritingFileStream();
					}
				}
			}

			OutputFile	<< count << " "	
					<< MJoint[0] << " "
					<< MJoint[1] << " "
					<< MJoint[2] << " "
					<< MJoint[3] << " "
					<< MJoint[4] << " "
					<< MJoint[5] << " "
					<< MJoint[6] << " "
					<< force(0)  << " "
					<< force(2)  << " "
					<< perturb_flag << " "
					<< x_new(0)  << " "
					<< x_new(1)  << " "
					<< x_new(2)  << " "
					<< x_new(3)  << " "
					<< x_new(4)  << " "
					<< x_new(5)  << " "
					<< damping(0, 0) << " "
					<< damping(2, 2) << " "
					<< desired(0)<< " " 
					<< desired(1) << " " 
					<< inertia(0, 0) << " "
					<< mag << " "
					<< trigger << " " 
					<< chosen<< " " 
					<< random_num << " " 
					<< failure_flag << std::endl;

			//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %d\n", count, MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6], force(0), force(2), perturb_flag, x_new(0), x_new(1), x_new(2), x_new(3), x_new(4), x_new(5), damping(0, 0), desired(0), desired(1), trigger, chosen, random_num, failure_flag);

			// Use commanded x--------------------------------------------------------------------------
			x_003 << x_03;
			x_03 << x_new;
			x_new << (inertia / (0.000001) + damping / (0.001) + stiffness).inverse()*(force + (inertia / (0.000001))*(x_03 - x_003) + stiffness * (x_e - x_03)) + x_03;

			//------------------------------------------------------------------------------------------

			if (steady > 2000)
			{
				if (x_new(2) >= 0.94)
				{
					x_new(2) = 0.94;
				}

				if (x_new(2) <= 0.58)
				{
					x_new(2) = 0.58;
				}

				if (x_new(0) >= 0.18)
				{
					x_new(0) = 0.18;
				}

				if (x_new(0) <= -0.18)
				{
					x_new(0) = -0.18;
				}
			}

			if (trigger == 1)
			{
				if (0.58 <= x_new(2) && x_new(2) <= 0.94)
				{
					if (-0.18 <= x_new(0) && x_new(0) <= 0.18)
					{
						qc << Ja.inverse()*(x_new - x_03);
						delta_q << delta_q + qc;
						q_new << delta_q + q_freeze;
					}
				}
			}

			if (steady < 2000)
			{
				q_new(0) = -1.5708;
				q_new(1) = 1.5708;
				q_new(2) = 0;
				q_new(3) = 1.5708;
				q_new(4) = 0;
				q_new(5) = -1.5708;
			}

			if (flag_finish == 1)
			{
				flag_v = 0;
				q_new << q_freeze;
			}


			client.NextJoint[0] = q_new(0);
			client.NextJoint[1] = q_new(1);
			client.NextJoint[2] = q_new(2);
			client.NextJoint[3] = q_new(3);
			client.NextJoint[4] = q_new(4);
			client.NextJoint[5] = q_new(5);
			client.NextJoint[6] = -0.958709;


			//---------------------------------------------
			/*client.NextJoint[0] = -1.5708;
			client.NextJoint[1] = 1.5708;
			client.NextJoint[2] = 0;
			client.NextJoint[3] = 1.5708;
			client.NextJoint[4] = 0;
			client.NextJoint[5] = -1.5708;
			client.NextJoint[6] = -0.958709;*/

			// Send data to visualizer gui
			//if (group_num == 1)
			//{
			xy_coord[0] = (float) mode;
			xy_coord[1] = desired(0);
			xy_coord[2] = desired(1);
			xy_coord[3] = d_r;
			xy_coord[4] = point(0);
			xy_coord[5] = point(1);
			xy_coord[6] = u_r;
			xy_coord[7] = P_ex(0);
			xy_coord[8] = P_ex(1);
			xy_coord[9] = ex_r;
			if (chosen <= 20)
			{
				xy_coord[10] = damping(0, 0);
			}
			else
			{
				xy_coord[10] = damping(2, 2);
			}
			xy_coord[11] = (float) (n_v + 1);
			xy_coord[12] = Vector.rows();
			udp_server.Send(xy_coord, 16);
			/*}
			else if (group_num == 2)
			{
			  xy_coord[0] = (float) mode;
			  xy_coord[1] = desired(1);
			  xy_coord[2] = desired(0);
			  xy_coord[3] = d_r;
			  xy_coord[4] = point(1);
			  xy_coord[5] = point(0);
			  xy_coord[6] = u_r;
			  xy_coord[7] = P_ex(1);
			  xy_coord[8] = P_ex(0);
			  xy_coord[9] = ex_r;
			  xy_coord[10] = damping(0, 0);
			  xy_coord[11] = (float) (n_v + 1);
			  xy_coord[12] = 25;
			  udp_server.Send(xy_coord, 16);
			}*/
			
		}
	}

	//fclose(OutputFile);
	//fprintf(stdout, "File closed.\n\n\n");
	// disconnect from controller

	fprintf(stdout, "Shhh.. I'm sleeping!\n");
	usleep(10000000);//microseconds //wait for close on other side
	app.disconnect();

	sleep(0.5);
	if (useEmg) 
	{
		/* Check if sensors are paired at the end of the block*/
		emgClient.IsSensorPaired(1);
		emgClient.IsSensorPaired(2);
		emgClient.IsSensorPaired(3);
		emgClient.IsSensorPaired(4);
		emgClient.IsSensorPaired(5);
		emgClient.IsSensorPaired(6);

		emgClient.StopReceiveDataStream();
	}
	sleep(0.5);

	gettimeofday(&start, NULL);

	return 1;
}



/*FILE *NewDataFile(void) //this may be specific to linux OS
{
	FILE *fp;
	time_t rawtime;
	struct tm *timeinfo;
	char namer[40];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(namer, 40, "Output%Y-%j-%H_%M_%S.txt", timeinfo);//creates a file name that has year-julian day-hour min second (unique for each run, no chance of recording over previous data)
	fp = fopen(namer, "w");//open output file
	return fp;
}*/

void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath) 
{
	/* deconstruct kuka file path into path, filename, extension */
	path pp = kukaDataFilePath.parent_path();
	path fname_stem = kukaDataFilePath.stem();
	path fname_ext = kukaDataFilePath.extension();

	/* Make a path to rename old file with same path, and rename if necessary */
	path p_unsuc = path(kukaDataFilePath.string());
	int unsuc_count = 1;
	std::string fname_unsuc;
	if (is_regular_file(p_unsuc)) 
	{
		while (is_regular_file(p_unsuc)) {
			//fname_unsuc = fname_stem.string() + std::string("_unsuccessful_") + std::to_string(unsuc_count) + fname_ext.string();
			fname_unsuc = fname_stem.string() + std::string("_") + std::to_string(unsuc_count) + fname_ext.string();
			p_unsuc = path(pp.string()) /= path(fname_unsuc);
			unsuc_count++;
		}
		rename(kukaDataFilePath, p_unsuc);
	}

	/* Make file stream */
	ofs.close();
	ofs.open(kukaDataFilePath);
}

// Shared Memory-------------------------------------------------------
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements){
	key_t key;
	int shmid;
	size_t shmSize = nElements*sizeof(T);
	T * shm = (T *) malloc(shmSize);
	/* make the key */
	if ((key = ftok(shmAddr.c_str(), 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, shmSize, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	shm = (T *) shmat(shmid, (void *)0, 0);

	if (shm == (T *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<nElements; i++)
	{
		shm[i] = 0.0;
	}

	return shm;
}

/*
// Shared Memory-------------------------------------------------------
int *MakeFloatSharedMemory(int HowBig)
{
	key_t key;
	int shmid;
	int *dataShared;

	dataShared = (int *)malloc(HowBig * sizeof(int));
	// make the key 
	if ((key = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared = (int *)shmat(shmid, (void *)0, 0);

	if (dataShared == (int *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig; i++)
	{
		dataShared[i] = 0.0;
	}

	return dataShared;
}


// Shared Memory Display--------------------------------------------------------------
float *MakeFloatSharedMemory2(int HowBig2)
{
	key_t key2;
	int shmid2;
	float *dataShared2;

	dataShared2 = (float *)malloc(HowBig2 * sizeof(float));
	// make the key 
	if ((key2 = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared2 = (float *)shmat(shmid2, (void *)0, 0);

	if (dataShared2 == (float *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig2; i++)
	{
		dataShared2[i] = 0.0;
	}

	return dataShared2;
}
*/