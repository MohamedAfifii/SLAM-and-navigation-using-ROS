
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

//Returns abxcd
double cross_product(WorldPoint A, WorldPoint B, WorldPoint C, WorldPoint D)
{
	vec AB = vec(point(A.x, A.y), point(B.x, B.y));
	vec CD = vec(point(C.x, C.y), point(D.x, D.y));

	return cross(AB,CD);
}

inline bool onLine(point p, line l)
{
	vec u = vec(l.first, l.second), v = vec(l.first, p);
	return EQ(cross(u,v), 0);
}

inline bool toRight(point p, line l)
{
	vec u = vec(l.first, l.second), v = vec(l.first, p);
	return LT(cross(u,v), 0);
}

inline bool toLeft(point p, line l)
{
	vec u = vec(l.first, l.second), v = vec(l.first, p);
	return GT(cross(u,v), 0);
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

double absNormalizedDiff(double theta1, double theta2)
{
	double difference = theta1 - theta2;
	difference = atan2(sin(difference), cos(difference));
	return abs(difference);
}

double getAngle(WorldPoint p1, WorldPoint p2)
{
	vec u = vec(point(p1.x, p1.y), point(p2.x, p2.y));
	return arg(u);
}

WorldPoint getAnotherPointOnline(WorldPoint p, double theta)
{
	vec u = polar(1.0, theta);
	WorldPoint ret = p;
	ret.x += u.real();
	ret.y += u.imag();

	return ret;
}

WorldPoint getAnotherPointOnline(WorldPoint p, double theta, double d)
{
	vec u = polar(d, theta);
	WorldPoint ret = p;
	ret.x += u.real();
	ret.y += u.imag();

	return ret;
}

//Returns the perpindicular distance from the point C to the straight line AB
//The result is negative if the point C lies on the right of the ray AB
double perpindicularDistance(WorldPoint C, WorldPoint A, WorldPoint B)
{
	vec AB = vec(point(A.x, A.y), point(B.x, B.y));
	vec AC = vec(point(A.x, A.y), point(C.x, C.y));

	return cross(AB, AC)/abs(AB);
}

//Returns the perpindicular distance from the point C to the line passing through A and has a certain angle
double perpindicularDistance(WorldPoint C, WorldPoint A, double angle)
{
	WorldPoint B = getAnotherPointOnline(A, angle);
	return perpindicularDistance(C,A,B);
}

bool lineOfSight(GridPoint a, GridPoint b, Costmap& map)
{
	int old_thresh = Costmap::thresh;
	//Costmap::thresh = max({old_thresh, map.getCostVal(a), map.getCostVal(b)});

	GridPoint mn = min(a, b), mx = max(a, b);
	line l = {point(mn.first, mn.second), point(mx.first, mx.second)};

	bool free = true;
	while(mn != mx)
	{
		if(map.isOccupied(mn))
		{
			free = false;
			break;
		}

		point upper_corner(mn.first+0.5, mn.second+0.5);
		point lower_corner(mn.first+0.5, mn.second-0.5);
		
		if(onLine(upper_corner, l))				mn.first++, mn.second++;
		else if(onLine(lower_corner, l))		mn.first++, mn.second--;

		else if(toRight(upper_corner, l))		mn.second++;
		else if(toLeft(lower_corner, l))		mn.second--;
		else									mn.first++;
	}

	Costmap::thresh = old_thresh;
	return free;
}

bool lineOfSight(WorldPoint a, WorldPoint b, Costmap& map)
{
	return lineOfSight(map.world_to_grid(a), map.world_to_grid(b), map);
}

bool onSight(Pose pose, WorldPoint local_goal, double angle_tolerance)
{
	double theta_heading = tf::getYaw(pose.orientation);
	double theta_goal = getAngle(pose.position, local_goal);

	double difference = theta_heading - theta_goal;
	difference = atan2(sin(difference), cos(difference));
	return abs(difference) <= angle_tolerance;
}

bool safeCurve(WorldPoint start, WorldPoint end, WorldPoint center, int sign, Costmap& map)
{
	int old_thresh = Costmap::thresh;
	//Costmap::thresh = max({old_thresh, map.getCostVal(start), map.getCostVal(end)});
	bool free = true;

	start = map.grid_to_world(map.world_to_grid(start));
	end = map.grid_to_world(map.world_to_grid(end));
	center = map.grid_to_world(map.world_to_grid(center));

	double radius = EuclideanDistance(start, center);
	WorldPoint p = start;
	while(EuclideanDistance(p, end) > 0.1)
	{
		if(map.isOccupied(p))
		{
			free = false;
			break;
		}

		double theta_cp = getAngle(center, p);
		double theta_tangent = theta_cp+sign*M_PI/2;

		WorldPoint p2 = getAnotherPointOnline(p, theta_tangent, map.resolution);
		GridPoint p2_grid = map.world_to_grid(p2);
		GridPoint p_grid = map.world_to_grid(p);

		WorldPoint nxt;
		double best = -1;

		int dxs[] = {-1, 0, 1}, dys[] = {-1, 0, 1};
		for(int dx: dxs) for(int dy: dys)
		{
			GridPoint candidate = p2_grid;
			candidate.first += dx, candidate.second += dy;
			if(candidate == p_grid)	continue;

			WorldPoint candidate_world = map.grid_to_world(candidate);
			double dist = EuclideanDistance(candidate_world, center);

			if(best == -1 || abs(dist-radius) < best)
			{
				nxt = candidate_world;
				best = abs(dist-radius);
			}
		}

		p = nxt;
	}

	Costmap::thresh = old_thresh;
	return free;
}

bool shouldRotate(Pose pose, WorldPoint local_goal)
{
	double theta_heading = tf::getYaw(pose.orientation);
	double theta_goal = getAngle(pose.position, local_goal);
	double difference = theta_heading - theta_goal;
	difference = atan2(sin(difference), cos(difference));

	return abs(difference) > M_PI/2;
}


WorldPoint midPoint(WorldPoint a, WorldPoint b)
{
	WorldPoint ret;
	ret.x = (a.x+b.x)/2;
	ret.y = (a.y+b.y)/2;
	ret.z = 0;

	return ret;
}

point intersect(const point &a, const point &b, const point &p, const point &q)
{
    double d1 = cross(p - a, b - a);
    double d2 = cross(q - a, b - a);
    return (d1 * q - d2 * p) / (d1 - d2);
}

WorldPoint getLineIntersection(WorldPoint p1, WorldPoint p2, WorldPoint p3, WorldPoint p4)
{
	point a(p1.x,p1.y), b(p2.x,p2.y), p(p3.x,p3.y), q(p4.x,p4.y);
	point ans = intersect(a,b,p,q);

	WorldPoint ret;
	ret.x = ans.real();
	ret.y = ans.imag();
	ret.z = 0;
	return ret;
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