#include "segway_rmp200.h"
#include "ftdiexceptions.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>
#include <math.h>


/** \example motionModelData.cpp
 *
 * This example executes a series of commands to record platform data of interest to build a kinematic/dynammic model of the platform
 *
 * Before running this example it is important to follow the next steps:
 *
 * 	- check that E-Stop lanyard is plugged. (the red cordon). Motors won't
 * 	  start unless it is plugged
 *	- connect segway to USB slot of Pc host
 *	- start Segway UI processor (green button)
 *	- start Motors (yellow button)
 *
 * This example first creates an FTDI server to detect any possible segway
 * platform connected to the computer. It uses the add_custom_PID() function
 * to add the PID and VID combination used by Segway. It then opens the 
 * first device found, if any, using the serial number. At this point it is 
 * important that no other FTDI device is connected to the computer because 
 * the program does not check which kind of device it is.
 *
 * Once the FTDI device is opened and configured, a new CSegwayRMP200 object
 * is created an associated to the FTDI device. Then the robotic platform is
 * configured and commanded to move forward.
 *
 * While moving, every 1 second, the whole information of the platform is 
 * 50displayed on screen. Since the odometry of the robot uses inertial sensors,
 * if the platform is not actually moving, the displayed information will 
 * have no real meaning.
 *
 * It is important to note that it is not possible to execute any function 
 * of the public API of the CSegwayRMP200 class until a valid FTDI object 
 * has been associated to the segway object.
 *
 * If the motor are not enabled (yellow button turned off), the segway only
 * sends heartbeat commands, but it is not possible to send motion commands
 * or receive feedback data. In this case, the program will not fail, but
 * nothing will happen.
 */

using namespace std;

unsigned int iterationPeriod = 50; //msec

int main(int argc, char *argv[])
{
	//operational variables
	CSegwayRMP200 *segway;
	string serial_number;
	unsigned int ii=0;
	unsigned int jj=0;
	unsigned int kk=0;
	int opt;
	ofstream dataFile; //data file
	float vt, vr; //[m/s], [rad/s]
	string fileName = "/home/andreu/Desktop/dataFile.txt";
	timeval tvTimeStamp;
	double timeStamp;
	streamsize nn;
	
	//user variables 
	float duration; //duration of all the experiment
	float vTstepDuration; //duration of a step
	float vRstepDuration; //duration of a step
	float vT0; //initial translational velocity
	float vR0; //initial rotational velocity	
	float vTstepIncrement; //velocity increment at each step
	float vRstepIncrement; //velocity increment at each step
	
	//user derived variables
	unsigned int totalIterations;
	unsigned int jjMax;
	unsigned int kkMax;

	while ((opt = getopt(argc, argv, "f:h?"))!= -1)
	{
		switch (opt)
		{
			case 'f': // file name
				fileName = optarg;
				break;
			case '?': // help
			case 'h':
			default:
				cout << "  USAGE" << endl
				<< "  " << argv[0] << " [options]" << endl
				<< "  OPTIONS" << endl
				<< "   -f FILE_NAME (default: " << fileName << ")" << endl;
				return 1;
		}
	}
	
	//user dialog
	cout << endl << "Duration [s]: "; cin >> duration;
	cout << endl << "vTstepDuration [s]: "; cin >> vTstepDuration;
	cout << endl << "vRstepDuration [s]: "; cin >> vRstepDuration;
	cout << endl << "vT0 [m/s]: "; cin >> vT0;
	cout << endl << "vR0 [rad/s]: "; cin >> vR0;	
	cout << endl << "vTstepIncrement [m/s]: "; cin >> vTstepIncrement;
	cout << endl << "vRstepIncrement [rad/s]: "; cin >> vRstepIncrement;
	cout << endl;
			
	//set user derived variables
	totalIterations = (unsigned int)(duration/(iterationPeriod/1000.0));
	jjMax = (unsigned int)(vTstepDuration/(iterationPeriod/1000.0));
	kkMax = (unsigned int)(vRstepDuration/(iterationPeriod/1000.0));
	
	//prints experiment config
	cout << "*******************************************************************" << endl;
	cout << "Duration: " << duration << endl;
	cout << "vTstepDuration: " << vTstepDuration << endl;
	cout << "vRstepDuration: " << vRstepDuration << endl;
	cout << "vTstepIncrement: " << vTstepIncrement << endl;
	cout << "vRstepIncrement: " << vRstepIncrement << endl;
	cout << "vT0: " << vT0 << endl;
	cout << "vR0: " << vR0 << endl;
	cout << "totalIterations: " << totalIterations << endl;
	cout << "jjMax: " << jjMax << endl;
	cout << "kkMax: " << kkMax << endl;
	cout << "*******************************************************************" << endl;
	cout << "Do you want to continue ? (y/n)" << endl;
	char resp;
	cin >> resp;
	if (resp == 'n')
	{
		cout << "Execution aborted" << endl;
		return 1;
	}
	
	//open data file and prints a header
	dataFile.open(fileName.c_str(), ofstream::out);
	dataFile 	<< "tS[s] vTcmnd[m/s] vRcmnd[rad/s] vT[m/s] vR[rad/s] pitch[rad] roll[rad] pitchRate[rad/s] rollRate[rad/s] "
			<< "leftTorque[Nm] rightTorque[Nm] uiBattery[V] motorBattery[V]" << endl;

	
	//execution
	try{
		segway=new CSegwayRMP200();
		segway->connect();
		usleep(10000);
		segway->unlock_balance();
		segway->reset_right_wheel_integrator();
		segway->reset_left_wheel_integrator();
		segway->reset_yaw_integrator();
		segway->reset_forward_integrator();
		segway->set_velocity_scale_factor(1.0);
		segway->set_acceleration_scale_factor(1.0);
		segway->set_turnrate_scale_factor(1.0);
		//segway->set_operation_mode(balance);

		vt = vT0;
		vr = vR0;
		for (ii=0; ii<totalIterations; ii++)
		{
			jj++; kk++; 
			if ( jj == jjMax )
			{
				jj=0;
				vt = vt + vTstepIncrement;
			}
			if ( kk == kkMax )
			{
				kk=0;
				vr = vr + vRstepIncrement;
			}

			//send commands to platform
			segway->move( vt , vr/(2*M_PI) ); //rotational speed required in revolutions per second
			
			//timeStamp
			gettimeofday(&tvTimeStamp, NULL); 
			timeStamp = (double)(tvTimeStamp.tv_sec + tvTimeStamp.tv_usec/1e6);	
	
			//logging data
			//cout << (*segway) << endl;
			nn=dataFile.precision(15);
			dataFile << timeStamp << " ";
			dataFile.precision(6);
			dataFile 	<< vt << "\t" << vr << "\t"
					<< (segway->get_left_wheel_velocity()+segway->get_right_wheel_velocity())/2.0 << "\t" << segway->get_yaw_rate()*M_PI/180.0 << "\t"
					<< segway->get_pitch_angle()*M_PI/180.0 << "\t" << segway->get_roll_angle()*M_PI/180.0 << "\t"
					<< segway->get_pitch_rate()*M_PI/180.0 << "\t" << segway->get_roll_rate()*M_PI/180.0 << "\t"
					<< segway->get_left_motor_torque() << "\t" << segway->get_right_motor_torque() << "\t"
					<< segway->get_ui_battery_voltage() << "\t" << segway->get_powerbase_battery_voltage() << endl;					
					
			//sleep up to next iteration
			usleep(iterationPeriod*1000);//sending commands @20Hz approx
		}
		
		dataFile.close();
		
/*		segway->reset_right_wheel_integrator();
		segway->reset_left_wheel_integrator();
		segway->reset_yaw_integrator();
		segway->reset_forward_integrator();
		sleep(5);
		cout << (*segway) << endl;*/
		
		segway->stop();
		segway->close();
		delete segway;
		
	}catch(CException &e){
		std::cout << e.what() << std::endl;
	}
}
