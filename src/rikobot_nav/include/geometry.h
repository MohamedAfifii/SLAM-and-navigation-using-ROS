
#ifndef __GEOMETRY__
#define __GEOMETRY__

#include <my_types.h>
#include <costmap.h>

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


bool lineOfSight(GridPoint a, GridPoint b, GlobalCostmap& map)
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

#endif