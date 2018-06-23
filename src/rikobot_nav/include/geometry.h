
#ifndef __GEOMETRY__
#define __GEOMETRY__

#include <my_types.h>
#include <cmath>
#include <costmap.h>
#include <tf_wrapper.h>

#define EPS 1e-8
#define EQ(x, y)	(abs((x)-(y)) < EPS)
#define GT(x, y)	((x)-(y) > EPS)
#define LT(x, y)	((y)-(x) > EPS)

#define vec(a,b)                	((b)-(a))
#define dot(u,v)                 	((conj(u)*(v)).real())	
#define cross(u,v)                 	((conj(u)*(v)).imag())

typedef complex<double> point;
typedef complex<double> vec;
typedef pair<point, point> line;

inline bool onLine(point p, line l)
{
	point u = vec(l.first, l.second), v = vec(l.first, p);
	return EQ(cross(u,v), 0);
}

inline bool toRight(point p, line l)
{
	point u = vec(l.first, l.second), v = vec(l.first, p);
	return LT(cross(u,v), 0);
}

inline bool toLeft(point p, line l)
{
	point u = vec(l.first, l.second), v = vec(l.first, p);
	return GT(cross(u,v), 0);
}


bool lineOfSight(GridPoint a, GridPoint b, Costmap& map)
{
	GridPoint mn = min(a, b), mx = max(a, b);

	while(mn != mx)
	{
		if(map.isOccupied(mn))	return false;

		point upper_corner(mn.first+0.5, mn.second+0.5);
		point lower_corner(mn.first+0.5, mn.second-0.5);
		line l = {point(mn.first, mn.second), point(mx.first, mx.second)};

		if(onLine(upper_corner, l))				mn.first++, mn.second++;
		else if(onLine(lower_corner, l))		mn.first++, mn.second--;

		else if(toRight(upper_corner, l))		mn.second++;
		else if(toLeft(lower_corner, l))		mn.second--;
		else									mn.first++;
	}

	return true;
}

bool lineOfSight(WorldPoint a, WorldPoint b, Costmap& map)
{
	return lineOfSight(map.world_to_grid(a), map.world_to_grid(b), map);
}

bool onSight(Pose pose, WorldPoint local_goal, double angle_tolerance)
{
	double theta_heading = tf::getYaw(pose.orientation);

	double dx = local_goal.x - pose.position.x;
	double dy = local_goal.y - pose.position.y;
	double theta_goal = atan2(dy, dx);

	return abs(theta_heading - theta_goal) <= angle_tolerance;
}

double EuclideanDistance(WorldPoint a, WorldPoint b)
{
	double dx = abs(a.x - b.x);
	double dy = abs(a.y - b.y);
	return hypot(dx, dy);
}

double EuclideanDistance(GridPoint a, GridPoint b)
{
	int dx = abs(a.first - b.first);
	int dy = abs(a.second - b.second);
	return hypot(dx, dy);
}

struct Window
{
	WorldPoint center;
	double width, height;
	double xl, xh, yl, yh;

	Window(WorldPoint center, double width, double height):
	center(center), width(width), height(height)
	{
		xl = center.x-width/2, xh = center.x+width/2;
		yl = center.y-height/2, yh = center.y+height/2;
	}

	bool pointInside(WorldPoint p)
	{
		return (xl <= p.x && p.x <= xh && yl <= p.y && p.y <= yh);
	}
};

#endif