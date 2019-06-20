#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include "geometry_msgs/Pose.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fstream>
#include <string>
#include <math.h>
#include <tr1/tuple> // if gcc > 4.7 need to use #include <tuple> // std::tuple, std::make_tuple, std::tie

class vdpgtController{
	private:
		ros::Subscriber pos_sub;
		ros::Publisher cmd_vel_pub;
		int count;
		int move; 
		double trans_x_param , init_x, init_y, pos_y, pos_x, target_x, target_dist;
		double yaw_param , ang_z, ori_z, target_z, init_ang_z,turn_z;
		double PI_0;
		double dest_x = 4.5;
		double dest_y = 4.5;	
		int start_x = 8; // start value for x-coordinate
		int start_y = 0;// start value for x-coordinate
		int orientation = 0; //0 = top, 1 = right, 2 = back, 3= left;
		int cell_value_x = 8;
		int cell_value_y = 0;
		int next_cell_value_x;// predicted value for the next move in x-direction
		int next_cell_value_y;// predicted value for the next move in y-direction
		bool next_step_obtained = false;
		double POS;
		double init_POS;
		float yaw_ref;   //stores the value of that yaw should be
		float yaw_error; // the difference between current value of yaw and yaw_ref
		float yaw_pos; // used to calculate the value of yaw_error 
		bool updated;
/*
distmaze shows the original maze and the number of steps needed to move from the start point to the end.
The starting point is at the bottom left corner, which coordinates represent cell [8][0].
vWall represents the vertical walls in the maze and hWalls the horizontal walls.
*/

		int distmaze[9][9] = {{8,7,6,5,4,5,6,7,8}, 
			{7,6,5,4,3,4,5,6,7},
			{6,5,4,3,2,3,4,5,6},
			{5,4,3,2,1,2,3,4,5},
			{4,3,2,1,0,1,2,3,4},
			{5,4,3,2,1,2,3,4,5},
			{6,5,4,3,2,3,4,5,6},
			{7,6,5,4,3,4,5,6,7},
			{8,7,6,5,4,5,6,7,8}};

		int vWalls[9][10] = {{-1,0,0,0,0,0,0,0,0,-1},
			{-1,0,-1,0,0,0,0,0,0,-1},
			{-1,0,0,0,0,0,0,0,-1,-1},
			{-1,0,-1,0,0,0,-1,0,0,-1},
			{-1,0,-1,0,0,0,-1,0,0,-1},
			{-1,-1,0,0,-1,0,-1,0,-1,-1},
			{-1,-1,0,0,-1,0,-1,0,-1,-1},
			{-1,0,-1,0,-1,0,-1,0,-1,-1},
			{-1,-1,-1,0,-1,0,0,0,0,-1}};	

		int hWalls[10][9] = {{-1,-1,-1,-1,-1,-1,-1,-1,-1},
			{0,0,0,0,0,0,0,0,0},
			{-1,-1,0,0,0,0,0,0,-1},
			{0,0,-1,-1,-1,-1,0,0,0},
			{0,0,0,0,0,0,0,0,0},
			{0,-1,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0,-1},
			{-1,-1,-1,-1,-1,-1,-1,-1,-1}};	
		
	public:
			vdpgtController(ros::NodeHandle &nh){
				count = 0;
				move = 0;
				updated = false;//bool value for checkupdate function()
				trans_x_param = 0;
				PI_0 =3.1415;
				pos_sub = nh.subscribe("/odom",1,&vdpgtController::callback, this);
				cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
			}
			void callback( const nav_msgs::OdometryConstPtr& poseMsg){
				geometry_msgs::Twist base_cmd;
				target_x = 1.0; // Change travel distance here

				//for angle
				double quatx= poseMsg->pose.pose.orientation.x;
				double quaty= poseMsg->pose.pose.orientation.y;
				double quatz= poseMsg->pose.pose.orientation.z;
				double quatw= poseMsg->pose.pose.orientation.w;
				tf::Quaternion q(quatx, quaty, quatz, quatw);
				tf::Matrix3x3 m(q);
				double yaw,pitch,roll;
				m.getRPY(roll, pitch, yaw);

				pos_x = poseMsg->pose.pose.position.x;
				pos_y = poseMsg->pose.pose.position.y;

				checkupdate(updated);// path of is updated 
/*
This section acts as a feedback so that the bot will not drift.It is done by comparing the 
error between the angular position of the bot
*/
				if (yaw_ref > 3.0)  //if yaw_ref has reached 180 degrees, make sure yaw_pos is positive
				{					//otherwise, value will fluctuate between -3.14 and 3.14
					if (yaw < 0.0)
						yaw_pos = yaw + 2*PI_0;
					else
						yaw_pos = yaw;
				}
				else
				{	
					yaw_pos = yaw;
				}

				yaw_error = yaw_pos - yaw_ref;	//calculating the error between reference and current position


				Next_Step(next_step_obtained, yaw, poseMsg, base_cmd);

			}
/*Update() remaps the path needed for the robot to reach the destination. Due to the positions of the walls being
known, the path is being updated recursively. Whenever there is a wall within the space of the map, the path will
update the next value  */
			void update(int i, int j)
			{	

				int cell_val = distmaze[i][j];

				int left = 100;
				int right = 100;
				int front = 100;
				int back = 100;


				if (hWalls[i][j] != -1)
					front = distmaze[i-1][j];		
				if (vWalls[i][j+1] != -1)
					right = distmaze[i][j+1];	
				if (hWalls[i+1][j] != -1)
					back = distmaze[i+1][j];
				if (vWalls[i][j] != -1)
					left  = distmaze[i][j-1];

				int min = front;

				if (front < min)// checking the adjacent cells of the bot for the min value
					min = front;

				if (right < min)
				{	
					min = right;
				}
				if (left < min)
				{	
					min = left;
				}
				if (back <  min)
				{	
					min = back;
				}

				if (cell_val != min + 1 && distmaze[i][j] != 0) 
				{
					distmaze[i][j] = min + 1;
					if (i != 0)
						update(i-1, j);	
					if (i != 8)
						update(i+1, j);
					if (j != 0)
						update(i, j-1);	
					if (j != 8)
						update(i, j+1);
				}

				return;		
			}

/*
checkupdate() checks and update the grid and the path to take with respect to the positions of the wall in place.
The positions of the walls are hardcoded and the path for the robot will be planned before it move.
*/
			void checkupdate(bool &updated){ //check update is only run one and updated starts as false initially
				if (!updated)
				{
					for (int i = 0; i < 9; i++)
					{
						for (int j = 0; j < 9; j++)
						{
							std::cout << distmaze[i][j] << " ";
						}
						std::cout << std::endl;
					}			
//will cout the path for original path 

					for (int i = 0; i < 9; i++)
					{
						for (int j = 0; j < 9; j++)
						{
							update(i, j);
						}
					}

					updated = true; //once true, it will not update the grid again

					for (int i = 0; i < 9; i++)
					{
						for (int j = 0; j < 9; j++)
						{
							std::cout << distmaze[i][j] << " ";
						}
						std::cout << std::endl;
					}
				}//cout new updated path
 			}
			void Next_Step(bool & next_step_obtained, double &yaw, const nav_msgs::OdometryConstPtr &poseMsg, geometry_msgs::Twist &base_cmd)
			{
				if (!(cell_value_x == 4 && cell_value_y == 4))
				{	

					if (next_step_obtained == false)
					{
						int left = 100;
						int right = 100;
						int front = 100;
						int back = 100;
						int current = distmaze[cell_value_x][cell_value_y];
						int min = current; 

						//get the value of the current cell and compare to adjacent cells
						//This depends on the current orientation of the robot with each direction denoted by a number
						// 0 - front, 1 - right, 2 - back, 3 - left
						if (orientation == 0)
						{
							if (hWalls[cell_value_x][cell_value_y] != -1)
								front = distmaze[cell_value_x-1][cell_value_y];		
							if (vWalls[cell_value_x][cell_value_y+1] != -1)
								right = distmaze[cell_value_x][cell_value_y+1];	
							if (hWalls[cell_value_x+1][cell_value_y] != -1)
								back = distmaze[cell_value_x+1][cell_value_y];
							if (vWalls[cell_value_x][cell_value_y] != -1)
								left  = distmaze[cell_value_x][cell_value_y-1];
							//cout the walls output to see 
							// -1 means there is a wall and 0 means there isn't a wall in that direction
							std::cout << "v walls left: "<<vWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "v walls right: "<<vWalls[cell_value_x][cell_value_y+1]<<std::endl;
							std::cout << "h walls front: "<<hWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "h walls back: "<< hWalls[cell_value_x+1][cell_value_y]<<std::endl<<std::endl;
						}
						else if (orientation == 1)
						{
							if (hWalls[cell_value_x][cell_value_y] != -1)
								left = distmaze[cell_value_x-1][cell_value_y];		
							if (vWalls[cell_value_x][cell_value_y+1] != -1)
								front = distmaze[cell_value_x][cell_value_y+1];	
							if (hWalls[cell_value_x+1][cell_value_y] != -1)
								right = distmaze[cell_value_x+1][cell_value_y];
							if (vWalls[cell_value_x][cell_value_y] != -1)
								back  = distmaze[cell_value_x][cell_value_y-1];

							std::cout << "h walls left: "<<hWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "h walls right: "<<hWalls[cell_value_x+1][cell_value_y]<<std::endl;
							std::cout << "v walls front: "<<vWalls[cell_value_x][cell_value_y+1]<<std::endl;
							std::cout << "v walls back: "<<vWalls[cell_value_x][cell_value_y]<<std::endl<<std::endl;

						}
						else if (orientation == 2)
						{
							if (hWalls[cell_value_x][cell_value_y] != -1)
								back = distmaze[cell_value_x-1][cell_value_y];		
							if (vWalls[cell_value_x][cell_value_y+1] != -1)
								left = distmaze[cell_value_x][cell_value_y+1];	
							if (hWalls[cell_value_x+1][cell_value_y] != -1)
								front = distmaze[cell_value_x+1][cell_value_y];
							if (vWalls[cell_value_x][cell_value_y] != -1)
								right  = distmaze[cell_value_x][cell_value_y-1];

							std::cout << "v walls left: "<<vWalls[cell_value_x][cell_value_y+1]<<std::endl;
							std::cout << "v walls right: "<<vWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "h walls front: "<<hWalls[cell_value_x+1][cell_value_y]<<std::endl;
							std::cout << "h walls back: "<<hWalls[cell_value_x][cell_value_y]<<std::endl<<std::endl;
						}
						else 
						{
							if (hWalls[cell_value_x][cell_value_y] != -1)
								right = distmaze[cell_value_x-1][cell_value_y];		
							if (vWalls[cell_value_x][cell_value_y+1] != -1)
								back = distmaze[cell_value_x][cell_value_y+1];	
							if (hWalls[cell_value_x+1][cell_value_y] != -1)
								left = distmaze[cell_value_x+1][cell_value_y];
							if (vWalls[cell_value_x][cell_value_y] != -1)
								front  = distmaze[cell_value_x][cell_value_y-1];

							std::cout << "h walls left: "<<hWalls[cell_value_x+1][cell_value_y]<<std::endl;
							std::cout << "h walls right: "<<vWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "v walls front: "<<vWalls[cell_value_x][cell_value_y]<<std::endl;
							std::cout << "v walls back: "<<vWalls[cell_value_x][cell_value_y+1]<<std::endl<<std::endl;
						}
/*
This section we are planning the movement or the turning phase, by the orientation we determine if the bot should
go forward, left, right or go back
*/

						if (front < min)
						{
							min = front;
							//check the next position to go
							if (orientation == 0) 
							{
								next_cell_value_x = cell_value_x - 1;
								next_cell_value_y = cell_value_y;
							}
							else if (orientation == 1)
							{
								next_cell_value_x = cell_value_x;
								next_cell_value_y = cell_value_y + 1;
							}
							else if (orientation == 2)
							{
								next_cell_value_x = cell_value_x + 1;
								next_cell_value_y = cell_value_y;
							}
							else 
							{
								next_cell_value_x = cell_value_x;
								next_cell_value_y = cell_value_y - 1;
							}
							move = 1;	// determine the way to move(forward or turn)
						}


						if (right < min)
						{
							min = right;
							move = 2;
						}


						if (left < min)
						{
							min = left;
							move = 3;
						}


						if (back < min)
						{
							min = back;
							move = 4;
						}

						std::cout << "front: " << front << " " << "left: " << left << " " << "right: " << right << " " << "back: " <<  back  << std::endl<<std::endl;
						next_step_obtained = true;
					}
					if (next_step_obtained ==  true) //robot will start to take the next step here
					{
						switch (move)
						{
							/*
						case 1 : the bot moves forward. The coordinates to move forward depends on the orientation
						of the bot. More details in the report
						*/
							case 1: 			    	
								if(count == 0){
									init_x = poseMsg->pose.pose.position.x;
									init_y = poseMsg->pose.pose.position.y;

									count = 1;
								}// count = 0 ends here

								if (orientation == 0)//0 = top   
								{
									POS = pos_x;
									init_POS = init_x;
									target_dist = 1.0; //move by 1 unit 
								}
								else if (orientation == 3)//3= left
								{
									POS = pos_y;
									init_POS = init_y;
									target_dist = 1.0;
								}

								if (orientation == 0 || orientation == 3)
								{
									if(POS - init_POS <target_dist){
										trans_x_param = 0.5; // Change robot velocity
									}else{
										trans_x_param = 0;
										next_step_obtained = false;
										count = 0;
										cell_value_x = next_cell_value_x;
										cell_value_y = next_cell_value_y;
										//std::cout << "bot stop"<<std::endl;
										//count = 0;
									}
								}

								if (orientation == 2)//2 = back
								{
									POS = pos_x;
									init_POS = init_x;
									target_dist = -1.0;
								}
								else if (orientation == 1)//1 = right
								{
									POS = pos_y;
									init_POS = init_y;
									target_dist = -1.0;
								}

								if (orientation == 2 || orientation == 1)
								{
									if(POS - init_POS > target_dist){
										trans_x_param = 0.5; // Change robot velocity
									}else{
										trans_x_param = 0;
										next_step_obtained = false; //reset values
										count = 0;
										cell_value_x = next_cell_value_x;
										cell_value_y = next_cell_value_y;
										//count = 0;
									}
								}

								yaw_param = -1.0 * yaw_error;
								base_cmd.linear.x = trans_x_param;

								cmd_vel_pub.publish(base_cmd);

								base_cmd.angular.z = yaw_param;	//apply change to the yaw to keep it close to the reference

								cmd_vel_pub.publish(base_cmd);
								break;
							case 2: //case 2 turns the object to 90 cw
								target_z =-PI_0/2; // Change orientation angle here		

								ang_z = yaw;

								if(count == 0){
									init_ang_z = yaw;

									count = 1;
								}// count = 0 ends here

								turn_z = ang_z - init_ang_z;
								if (turn_z>PI_0){
									turn_z =turn_z - 2*PI_0;
								}	
								if(turn_z > target_z){
									yaw_param = -0.3; // Change robot angular velocity
								}else{
									yaw_param = 0;
									next_step_obtained = false;
									orientation = (orientation + 1) % 4;
									count = 0;
									yaw_ref = yaw_ref + target_z;
								}

								base_cmd.angular.z = yaw_param;

								cmd_vel_pub.publish(base_cmd);
								break;
							case 3: //turn 90 deg acw
								target_z =PI_0/2; // Change orientation angle here		

								ang_z = yaw;
								if(count == 0){
									init_ang_z = yaw;

									count = 1;
								}// count = 0 ends here
								turn_z = ang_z - init_ang_z;
								if (turn_z<-PI_0){
									turn_z =turn_z + 2*PI_0;
								}	
								if(turn_z <target_z){
									yaw_param = 0.3; // Change robot angular velocity
								}else{
									yaw_param = 0;
									next_step_obtained = false;
									if (orientation == 0)
										orientation = 3;
									else
										orientation--;
									count = 0;
									yaw_ref = yaw_ref + target_z;
								}

								base_cmd.angular.z = yaw_param;

								cmd_vel_pub.publish(base_cmd);
								break;
							case 4://turn 180 back
								target_z =-PI_0; // Change orientation angle here		

								ang_z = yaw;

								if(count == 0){
									init_ang_z = yaw;

									count = 1;
								}// count = 0 ends here

								turn_z = ang_z - init_ang_z;
								if (turn_z>PI_0){
									turn_z =turn_z - 2*PI_0;
								}	
								if(turn_z > target_z){
									yaw_param = -0.3; // Change robot angular velocity
								}else{
									yaw_param = 0;
									next_step_obtained = false;
									orientation = (orientation + 2) % 4;
									count = 0;
									yaw_ref = yaw_ref + target_z;
								}

								base_cmd.angular.z = yaw_param;

								cmd_vel_pub.publish(base_cmd);
								break;
						}
					}

				}

			}


};

int main(int argc, char** argv){
	ros::init(argc, argv, "curvature_vdpgt");
	ros::NodeHandle nh;
	vdpgtController vc(nh);

	ros::spin();
	return 0;
}
