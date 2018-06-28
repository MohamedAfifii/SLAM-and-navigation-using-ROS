#ifndef __LOCAL_PLANNERS__
#define __LOCAL_PLANNERS__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf_wrapper.h>

#include <my_types.h>
#include <costmap.h>

#define R 0.0325	//Wheel radius in meters
#define L 0.27		//Distance between the two wheels in meters

class LocalPlanner
{
public:
	int frequency;
	double goal_tolerance;
	double MotorMin, MotorMax;	//Min and max angular velocities in rad/sec
	double angle_tolerance;
	double rmin;				//Minimum radius of curvature
	int thresh;
	double no_rot_angle;

	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("/local_goal", 5);
	ros::Publisher icc_pub = nh.advertise<geometry_msgs::PointStamped>("/icc", 5);
	ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("/my_nav/trajectory", 3);

	geometry_msgs::Twist cmd;

	TFWrapper tf_wrapper;
	//LocalCostmap local_map;

	void publish_command(double v, double w)
	{
		cmd.linear.x = v;
		cmd.angular.z = w;
		cmd_pub.publish(cmd);
	}

	void follow_cricle(double radius_of_curvature, int sign)
	{
		double v = MotorMax*2*R/(2+L/radius_of_curvature);
		double w = sign*v/radius_of_curvature;
		publish_command(v,w);
	}

	void rotate_inplace(WorldPoint goal)
	{
		Pose current_pose = tf_wrapper.getCurrentPose();

		//Get the direction of rotation
		WorldPoint p1 = current_pose.position;
		double theta_heading = tf::getYaw(current_pose.orientation);
		WorldPoint p2 = getAnotherPointOnline(p1, theta_heading);
		double sign = 1;	//CCW
		if(cross_product(p1,p2, p1,goal) < 0)	sign = -1;	//CW

		//Publish the command
		double MotorVel = 0.7*MotorMax;	//Angular velocity
		double w = sign*(R/L)*2*MotorVel;
		publish_command(0, w);


		//Wait until the robot faces the local_goal
		ros::Rate rate(frequency);
		while(!onSight(current_pose, goal, angle_tolerance))
		{
			rate.sleep();
			current_pose = tf_wrapper.getCurrentPose();
		}
	}

	void move_forward()
	{
		double MotorVel = 0.8*MotorMax;	//Angular velocity
		double v = MotorVel*R;
		publish_command(v, 0);
	}


	bool execute_path(GlobalCostmap &global_map, const Path &path)
	{
		Costmap::thresh = thresh;
		
		int path_length = sz(path);
		WorldPoint goal_point = path[path_length-1];

		vector<WorldPoint> trajectory;
		WorldPoint last_position;		//Used for trajectory visualization
		last_position.x = last_position.y = last_position.z = -1;
		int last_goal = -1;

		rmin = (L/2.0)*((MotorMax+MotorMin)/(MotorMax-MotorMin));	//The 1.2 factor is for the safty margin.

		ros::Rate rate(frequency);
		while(1)	
		{
			Pose current_pose = tf_wrapper.getCurrentPose();
			WorldPoint current_position = current_pose.position;
			double theta_heading = tf::getYaw(current_pose.orientation);

			if(EuclideanDistance(current_position, last_position) > 0.15)
			{
				trajectory.push_back(current_position);
				last_position = current_position;
			}

			//Goal reached
			if(EuclideanDistance(current_position, goal_point) <= goal_tolerance)	break;				


			//Iterate over path points
			bool done = false;
			for(int i = path_length-1; i >= 0; i--)
			{
				WorldPoint local_goal = path[i];
				double theta_goal = getAngle(current_position, local_goal);
				
				int old_thresh = Costmap::thresh;
				if(EuclideanDistance(current_position, local_goal) < 1 && absNormalizedDiff(theta_goal, theta_heading) < no_rot_angle)
					Costmap::thresh += 15;

				if(i == last_goal || lineOfSight(current_position, local_goal, global_map))
				{
					//Visualization
					geometry_msgs::PointStamped pt;
					pt.header.frame_id = "/map";
					pt.point = local_goal;
					goal_pub.publish(pt);

					WorldPoint ICC = current_position;

					if(onSight(current_pose, local_goal, angle_tolerance))	//Default: 10 degrees tolerance
						move_forward();

					else if(last_goal == -1 || shouldRotate(current_pose, local_goal)) 
						rotate_inplace(local_goal);
						

					else
					{
						double yg = perpindicularDistance(local_goal, current_position, theta_heading);
						double xg = abs(perpindicularDistance(local_goal, current_position, theta_heading+M_PI/2));
		
						//Get the direction of rotation
						double sign = 1;
						if(yg < 0)	sign = -1, yg = -yg;

						double radius_of_curvature = -1;

						double difference = absNormalizedDiff(theta_heading, theta_goal);
						if(difference < 0.35)
						if(false)
						{
							WorldPoint p1 = current_position;
							WorldPoint p2 = getAnotherPointOnline(p1, theta_heading+M_PI/2);

							WorldPoint p3 = midPoint(current_position, local_goal);
							WorldPoint p4 = getAnotherPointOnline(p3, theta_goal+M_PI/2);

							ICC = getLineIntersection(p1,p2,p3,p4);

							//pt.point = ICC;
							//icc_pub.publish(pt);

							//cout << "Checking 1" << endl;
							//if(i == last_goal || safeCurve(current_position, local_goal, ICC, sign, global_map))
							if(true)
							{
								radius_of_curvature = EuclideanDistance(current_position, ICC);
								//cout << "Safe" << endl;
							}

							//else cout << "Not safe" << endl;
						}

						if(radius_of_curvature == -1 && xg > rmin)
						{
							ICC = getAnotherPointOnline(current_position, theta_heading+sign*M_PI/2, rmin);
							
							//Get the point of tangency
							double length = EuclideanDistance(ICC, local_goal);
							double D = sqrt(length*length - rmin*rmin);

							double x = local_goal.x - ICC.x;
							double y = local_goal.y - ICC.y;
							double alpha = atan2(y,x);
							double beta = atan2(D,rmin);

							double theta1 = -(beta-alpha);
							double theta2 = 2*beta + theta1;
							WorldPoint p1 = getAnotherPointOnline(ICC, theta1, rmin);
							WorldPoint p2 = getAnotherPointOnline(ICC, theta2, rmin);
							WorldPoint pointOfTangancy;
							if(EuclideanDistance(current_position, p1) < EuclideanDistance(current_position, p2))
								pointOfTangancy = p1;
							else
								pointOfTangancy = p2;

							//pt.point = ICC;
							//icc_pub.publish(pt);

							//cout << "Checking 2" << endl;
							bool safe = safeCurve(current_position, pointOfTangancy, ICC, sign, global_map);
							safe &= lineOfSight(pointOfTangancy, local_goal, global_map);
							safe |= (i == last_goal);

							if(safe)	
							{
								radius_of_curvature = rmin;
								//cout << "Safe" << endl;
							}

							//else	cout << "Not safe" << endl;
						}

						if(radius_of_curvature != -1)	follow_cricle(radius_of_curvature, sign);
						else	rotate_inplace(local_goal);
					}

					//Visualization
					pt.point = ICC;
					icc_pub.publish(pt);	
								
					Costmap::thresh = old_thresh;
					last_goal = i;
					done = true;
					break;
				}

				else Costmap::thresh = old_thresh;
			}

			if(!done)
			{
				cout << "Goal unreachable!" << endl;
				publish_command(0,0);
				cout << "Replanning ..." << endl << endl;
				return false;
			}

			rate.sleep();
		}

		cout << "Goal reached!" << endl;
		publish_command(0,0);

		//Publish the trajectory
		nav_msgs::Path msgOut;
		msgOut.header.frame_id = "map";
		int sz = trajectory.size();
		msgOut.poses.resize(sz);
		for(int i = 0; i < sz; i++)
		{
			msgOut.poses[i].header.frame_id = "map";
			msgOut.poses[i].pose.position = trajectory[i];
		}
		trajectory_pub.publish(msgOut);

		return true;
	}
};


#endif