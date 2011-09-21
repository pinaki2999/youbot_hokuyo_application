/*
 * hokuyo_sweep_scan.cpp
 *
 *  Created on: Jul 13, 2011
 *      Author: reon
 */

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/io/pcd_io.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl/point_types.h>
#include <ros/publisher.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include "trajectory_msgs/JointTrajectory.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include "brics_actuator/JointPositions.h"

#include <sstream>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/*
* Globally accessible Parameters
*/
ros::Publisher armPositionsPublisher;
ros::Subscriber armJointsStateSubscriber;
ros::Publisher hokuyoAssembledScanPublisher;


PointCloud assembledPointCloud;


//The laser assembler server which is responsible for accumulating the point clouds during the entire scanning episode
laser_assembler::AssembleScans srv;
ros::ServiceClient client;

//The laser assembler server which is responsible for accumulating the point clouds at every step for getting intermediate pointcloud
//ToDo merge the two laser assemblers
laser_assembler::AssembleScans srvPublish;
ros::ServiceClient clientPublish;


//The maximum range for left-right movement of joint 0
double lr_range;
//Indicates if the arm is in its initial position
//ToDo check the actual odometry reading
bool armInitialized;
//Indicates if a scanning episode is completed or not
bool scanningEpisodeCompleted;
//Indicates if the assembled pointclouds should be stored or not
bool doStorePointClouds;
// Stores the recently recieved arm-joint-states
double armJointStates[5];

//-------------------------------------------------------------Method declarations
void startAssemblingLaserScans();
void endAssemblingLaserScans();
void publishNewAssembledScan();
int setArmPosition(double joint_positions[]);
void armIntialize();
void setNextScanPosition();
void armJointsStateCallback(sensor_msgs::JointState armJointState);


/**
 * Starts the assembling process
 */
void startAssemblingLaserScans(){
	srv.request.begin = ros::Time::now();
	srvPublish.request.begin =ros::Time::now();
}


/**
 * Ends the assembling process
 */
void endAssemblingLaserScans() {

	srv.request.end   = ros::Time::now();
	if (client.call(srv)){
		printf("Got cloud with %u points\n", srv.response.cloud.points.size());

		time_t now;
		char timestamp[20];
		timestamp[0] = '\0';
		now = std::time(NULL);
		if (now != -1)
		{
			strftime(timestamp, 20, "%Y%m%d_%H%M%S", gmtime(&now));
		}

        if( doStorePointClouds ) {
            pcl::PCDWriter writer;
            std::stringstream filename;
            filename.clear();
            filename << "./" << "hokuyo" << timestamp <<".asc";

            pcl::PointCloud<pcl::PointXYZ> tempPointCloud;


            tempPointCloud.height = 1;
            tempPointCloud.width = srv.response.cloud.points.size();
            tempPointCloud.is_dense = false;

            tempPointCloud.points.resize(tempPointCloud.height*tempPointCloud.width);


            for (unsigned int i = 0; i < tempPointCloud.width; i++) {
                tempPointCloud.points[i].x = srv.response.cloud.points[i].x;
                tempPointCloud.points[i].y = srv.response.cloud.points[i].y;
                tempPointCloud.points[i].z = srv.response.cloud.points[i].z;
            }

            writer.write(filename.str(),tempPointCloud,false);
        }

		scanningEpisodeCompleted = true;
		srvPublish.request.end   = ros::Time::now();
		armIntialize();
	}
	else {
		printf("Service call failed. Laser-scan assembling was not successful\n");
		scanningEpisodeCompleted = true;
		srvPublish.request.end   = ros::Time::now();
		armIntialize();
	}

}


/**
 * Publishes intermidiate scans
 */
void publishNewAssembledScan(){
	srvPublish.request.end   = ros::Time::now();

	if (clientPublish.call(srvPublish)){
		printf("Got cloud with %u points\n", srvPublish.response.cloud.points.size());
		pcl::PointCloud<pcl::PointXYZ> tempPointCloud;

		tempPointCloud.height = 1;
		tempPointCloud.width = srvPublish.response.cloud.points.size();
		tempPointCloud.is_dense = false;

		tempPointCloud.points.resize(tempPointCloud.height*tempPointCloud.width);


		for (unsigned int i = 0; i < tempPointCloud.width; i++) {
			tempPointCloud.points[i].x = srvPublish.response.cloud.points[i].x;
			tempPointCloud.points[i].y = srvPublish.response.cloud.points[i].y;
			tempPointCloud.points[i].z = srvPublish.response.cloud.points[i].z;
		}
		//ToDo
		assembledPointCloud += tempPointCloud;


		PointCloud::Ptr msgScanHokuyo (new PointCloud);
		msgScanHokuyo->header.frame_id = "base_link";
		msgScanHokuyo->height = 1;
		msgScanHokuyo->width = assembledPointCloud.width;

		for  (unsigned int i =0; i< assembledPointCloud.width ; i++){
			pcl::PointXYZ temp_point;
			temp_point.x = assembledPointCloud.points[i].x;
			temp_point.y = assembledPointCloud.points[i].y;
			temp_point.z = assembledPointCloud.points[i].z;
			msgScanHokuyo->points.push_back(temp_point);

		}
		msgScanHokuyo->header.stamp = ros::Time::now ();
		hokuyoAssembledScanPublisher.publish (msgScanHokuyo);
	}

	srvPublish.request.begin = ros::Time::now();

}


/**
 * Publishes arm position commands
 */
int setArmPosition(double joint_positions[]){
/*

    trajectory_msgs::JointTrajectory armCommand;
    trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

    desiredConfiguration.positions.resize(5);
    armCommand.joint_names.resize(5);


    std::stringstream jointName;
    for (int i = 0; i < 5; ++i) {
    	std::stringstream jointName;
    	jointName.str("");
            jointName << "arm_joint_" << (i + 1);

            desiredConfiguration.positions[i] = joint_positions[i];
            armCommand.joint_names[i] = jointName.str();

    };


    armCommand.header.stamp = ros::Time::now();
    armCommand.header.frame_id = "base_link";
    armCommand.points.resize(1); // only one point so far
    armCommand.points[0] = desiredConfiguration;

    armPositionsPublisher.publish(armCommand);


*/


	brics_actuator::JointPositions command;
	vector <brics_actuator::JointValue> armJointPositions;
	vector <brics_actuator::JointValue> gripperJointPositions;

	armJointPositions.resize(5); //TODO:change that
	gripperJointPositions.resize(2);

	for(int i = 0; i<5; i++){
		std::stringstream jointName;
		jointName.str("");
		jointName << "arm_joint_" << (i + 1);
		armJointPositions[i].joint_uri = jointName.str();
		armJointPositions[i].value = joint_positions[i];
		armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
	}

	command.positions = armJointPositions;
	armPositionsPublisher.publish(command);


	return 1;
}


/**
 * Puts the arm into an initial position
 * joint1: 0
 * joint2: 1.1
 * joint3: -2.7 (change towards -5 if you want to start the scan more towards ground)
 * joint4: 0.023 (between 0.0221239 rad and 3.4292 rad)
 * joint5: 0
 */
void armIntialize(){
	double jointPositions[5];
	jointPositions[0] = 0.1;
	jointPositions[1] = 1.1;
	jointPositions[2] = -2.7;
	jointPositions[3] = 0.5;
	jointPositions[4] = 0.5;        //1.3  if joint-5 working
	printf("Sending Initialization Command..\n");
	setArmPosition(jointPositions);
	armInitialized = true;
}




/**
 * Calculates next arm-position
 */
void setNextScanPosition(){
	if(armJointStates[0] < lr_range){
		double newJointPositions[5];
		newJointPositions[0] = armJointStates[0]+ 0.02;
		newJointPositions[1] = 0;
		newJointPositions[2] = 0;
		newJointPositions[3] = 0;
		newJointPositions[4] = 0;

		setArmPosition(newJointPositions);
		publishNewAssembledScan();
		printf("Checkpoint: new position send \n");
	} else {
		endAssemblingLaserScans();
	}
}


/**
 * Gets the arm joint states
 * Subscribes to the topic "/arm_1/joint_states"
 */
void armJointsStateCallback(sensor_msgs::JointState armJointState){

		printf("Checkpoint: reading joint states \n");
	for(int i= 0 ; i<4 ; i++){
		armJointStates[i] = armJointState.position[i];
	}

	if(armInitialized && !scanningEpisodeCompleted){
		printf("Checkpoint: asking for new position \n");
		setNextScanPosition();
	}
}


int main(int argc, char** argv){


	ros::init(argc, argv, "hokuyo_sweep_scan");
	ros::NodeHandle nh;
	ros::service::waitForService("assemble_scans");
	client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
	clientPublish = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
	lr_range = 1.8;

	armInitialized=false;
	scanningEpisodeCompleted = false;

	//armPositionsPublisher = nh.advertise<trajectory_msgs::JointTrajectory> ("/arm_controller/command", 1);
	armPositionsPublisher = nh.advertise<brics_actuator::JointPositions> ("/arm_1/arm_controller/position_command", 1);
	armJointsStateSubscriber = nh.subscribe("/arm_1/joint_states", 1, &armJointsStateCallback);
	//armJointsStateSubscriber = nh.subscribe("joint_states", 1, &armJointsStateCallback);
	hokuyoAssembledScanPublisher = nh.advertise<PointCloud> ("youbot_hokuyo_assembled_cloud", 1);

	assembledPointCloud.width = 0;
	assembledPointCloud.height = 1;
	assembledPointCloud.points.resize(assembledPointCloud.width * assembledPointCloud.height);

	ros::Rate sleepingRate(2);
	ros::spinOnce();

	ros::Rate initialRate(0.2);
	initialRate.sleep();
	if(!armInitialized)
		armIntialize();
	initialRate.sleep();

	startAssemblingLaserScans();

	bool toExit=false;

	int choice=0;

    printf("\nEnter '1' to also store the assembled pointclouds... ");
    scanf("%d",&choice);

    if( choice==1 ) {
        doStorePointClouds = true;
    } else {
        doStorePointClouds = false;
    }

	while(nh.ok() && !toExit){

		if(scanningEpisodeCompleted){
			printf("\nThe previous scan episode is completed!! "
					"\nDo you want to continue with a new scanning episode?\n"
					"Enter '1' to continue and anything else(from the keyboard) to exit ");
			scanf("%d",&choice);
			if(choice == 1) {
				scanningEpisodeCompleted=false;
				srv.request.begin = ros::Time::now();
				srvPublish.request.begin =ros::Time::now();
				assembledPointCloud.points.clear();
				assembledPointCloud.width = 0;
				assembledPointCloud.height = 1;
				assembledPointCloud.points.resize(assembledPointCloud.width * assembledPointCloud.height);


			} else {
				toExit= true;
			}
		}

		sleepingRate.sleep();
		ros::spinOnce();
	}

}
