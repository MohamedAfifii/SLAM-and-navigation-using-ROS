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

	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("/local_goal", 5);
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
		double v = 0.9*MotorMax*2*R/(2+L/radius_of_curvature);
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

	bool collision_free_path(Pose current_pose, WorldPoint local_goal, double radius, GlobalCostmap &global_map)
	{
		return true;
	}

	bool execute_path(GlobalCostmap &global_map, const Path &path)
	{
		int path_length = sz(path);
		WorldPoint goal_point = path[path_length-1];

		rmin = 1.1*(L/2.0)*((MotorMax+MotorMin)/(MotorMax-MotorMin));	//The 1.1 factor is for the safty margin.

		ros::Rate rate(frequency);
		while(1)	
		{
			Pose current_pose = tf_wrapper.getCurrentPose();

			//Goal reached
			if(EuclideanDistance(current_pose.position, goal_point) <= goal_tolerance)	break;				


			//Iterate over path points
			bool done = false;
			for(int i = path_length-1; i >= 0; i--)
			{
				WorldPoint local_goal = path[i];

				if(lineOfSight(current_pose.position, local_goal, global_map))
				{
					if(onSight(current_pose, local_goal, angle_tolerance))
						move_forward();

					else if(shouldRotate(current_pose, local_goal)) 
						rotate_inplace(local_goal);
						

					else
					{
						WorldPoint current_position = current_pose.position;
						double theta_heading = tf::getYaw(current_pose.orientation);
						double theta_goal = getAngle(current_position, local_goal);

						double yg = perpindicularDistance(local_goal, current_position, theta_heading);
						double xg = abs(perpindicularDistance(local_goal, current_position, theta_heading+M_PI/2));
		
						//Get the direction of rotation
						double sign = 1;
						if(yg < 0)	sign = -1, yg = -yg;

						double radius_of_curvature = -1;
						if(xg < yg)
						{
							if(xg > rmin)	follow_cricle(rmin, sign);
							else 			rotate_inplace(local_goal);
						}

						else
						{
							WorldPoint p1 = current_position;
							WorldPoint p2 = getAnotherPointOnline(p1, theta_heading+M_PI/2);

							WorldPoint p3 = midPoint(current_position, local_goal);
							WorldPoint p4 = getAnotherPointOnline(p3, theta_goal+M_PI/2);

							WorldPoint ICC = getLineIntersection(p1,p2,p3,p4);
							double radius_of_curvature = EuclideanDistance(current_position, ICC);

							follow_cricle(radius_of_curvature, sign);
						}
					}

					geometry_msgs::PointStamped pt;
					pt.header.frame_id = "/map";
					pt.point = local_goal;
					goal_pub.publish(pt);

					done = true;
					break;
				}
			}

			if(!done)
			{
				cout << "Goal unreachable!" << endl;
				cout << "Replanning ..." << endl << endl;
				return false;
			}

			rate.sleep();
		}

		cout << "Goal reached!" << endl;
		publish_command(0,0);
		return true;
	}
};


#endif