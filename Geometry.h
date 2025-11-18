#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>

// Simple 2D vector used for points.
struct Vec2
{
	double x;
	double y;

	Vec2() : x(0.0), y(0.0) {}
	Vec2(double x_, double y_) : x(x_), y(y_) {}
};

// Undirected line segment between two points.
struct Segment
{
	Vec2 a;
	Vec2 b;

	Segment() {}
	Segment(const Vec2& a_, const Vec2& b_) : a(a_), b(b_) {}
};

inline double distance(const Vec2& p, const Vec2& q)
{
	const double dx = p.x - q.x;
	const double dy = p.y - q.y;
	return std::sqrt(dx * dx + dy * dy);
}

// Loose equality with tolerance, used only if needed.
inline bool almostEqual(const Vec2& p, const Vec2& q, double eps = 1e-6)
{
	return distance(p, q) <= eps;
}

#endif // GEOMETRY_H
