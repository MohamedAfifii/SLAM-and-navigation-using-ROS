#ifndef __LOCAL_PLANNERS__
#define __LOCAL_PLANNERS__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf_wrapper.h>

#include <my_types.h>
#include <costmap.h>

#define R 0.0325	//Wheel radius	
#define L 27		//Distance between the two wheels

class LocalPlanner
{
public:
	int frequency;
	double goal_tolerance;
	double vmin, vmax, dv, wmin, wmax, dw, MotorMin, MotorMax;
	double min_trajectory_time, sim_time, sim_time_step, expected_velocity, angle_tolerance;
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
	geometry_msgs::Twist msgOut;


	Pose forward_simulation(Pose pose, Command cmd)
	{
		double v = cmd.first, w = cmd.second;
		double dc = v*sim_time_step;

		double theta = tf::getYaw(pose.orientation);
		double newTheta = theta + w*sim_time_step;
		double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

		Pose ret = pose;
		ret.position.x += dc*cos(avTheta);
		ret.position.y += dc*sin(avTheta);
		ret.orientation = tf::createQuaternionMsgFromYaw(newTheta);

		return ret;
	}

	bool execute_path(GlobalCostmap &global_map, const Path &path)
	{
		return true;
		
		TFWrapper tf_wrapper;
		LocalCostmap local_map;
		int path_length = path.poses.size();
		WorldPoint goal_point = path.poses[path_length-1].pose.position;

		vector<Command> valid_commands;
		for(double v = vmin; v <= vmax; v += dv)
		{
			for(double w = wmin; w <= wmax; w += dw)
			{
				double wr = (2*v + w*L)/(2*R);
				double wl = (2*v - w*L)/(2*R);

				if(MotorMin <= wr && wr <= MotorMax && MotorMin <= wl && wl <= MotorMax)
					valid_commands.push_back({v,w});
			}
		}

		ros::Rate rate(frequency);
		while(1)	
		{
			Pose current_pose = tf_wrapper.getCurrentPose();

			//Goal reached
			if(EuclideanDistance(current_pose.position, goal_point) <= goal_tolerance)	break;				

			//Active window
			Window w(current_pose.position, local_map.width, local_map.height);
			vector<WorldPoint> v;
			for(int i = path_length-1; i >= 0; i--)
			{
				if(w.pointInside(path.poses[i].pose.position))	v.push_back(path.poses[i].pose.position);
			}
			cout << "Size of active window = " << v.size() << endl;

			//Iterate over points in the active window
			bool done = false;
			for(WorldPoint local_goal: v)
			{
				double cost = -1;
				Command best_command;

				for(Command cmd: valid_commands)
				{
					double t = 0;
					Pose pose = current_pose;

					while(t < sim_time)
					{
						t += sim_time_step;
						pose = forward_simulation(pose, cmd);

						if(onSight(pose, local_goal, angle_tolerance))
						{
							cout << "Here" << endl;
							if(lineOfSight(pose.position, local_goal, global_map))
							{
								double rem_time = EuclideanDistance(pose.position, local_goal)/expected_velocity;

								if(cost == -1 || t+rem_time < cost)
								{
									cost = t+rem_time;
									best_command = cmd;
								}								
							}
						}
					}
				}

				if(cost != -1)
				{
					msgOut.linear.x = best_command.first;
					msgOut.angular.z = best_command.second;
					pub.publish(msgOut);
					done = true;
					break;
				}
			}

			if(!done)
			{
				cout << "Cannot generate a local plan" << endl;
				return 0;
			}


			rate.sleep();
		}

		cout << "Goal reached!" << endl;
		return true;
	}
};


#endif