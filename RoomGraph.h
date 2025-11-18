#ifndef ROOMGRAPH_H
#define ROOMGRAPH_H

#include <vector>
#include <map>
#include "Geometry.h"

// The RoomGraph takes a set of line segments and reconstructs
// all closed polygonal regions ("rooms") using a half-edge graph.
class RoomGraph
{
public:
	struct Room
	{
		// Polygon vertices in order (counter-clockwise).
		std::vector<Vec2> polygon;

		// Geometric center (area-weighted centroid).
		Vec2 center;

		// Signed area (always positive in the final result).
		double area;

		Room() : center(), area(0.0) {}
	};

	RoomGraph();

	// Build the internal graph from segments and extract all rooms.
	void build(const std::vector<Segment>& segments);

	const std::vector<Room>& getRooms() const;

private:
	// Node represents a unique point in the graph.
	struct Node
	{
		int id;
		Vec2 pos;

		// Indices of outgoing half-edges.
		std::vector<int> outgoingEdges;

		Node() : id(-1), pos() {}
	};

	// HalfEdge is a directed edge from "from" to "to".
	// Each undirected segment is stored as two opposite half-edges.
	struct HalfEdge
	{
		int id;
		int from;
		int to;
		int twin; // opposite half-edge
		int next; // next edge when walking around a face
		bool used;
		double angle; // direction angle at the "from" node

		HalfEdge()
			: id(-1),
			from(-1),
			to(-1),
			twin(-1),
			next(-1),
			used(false),
			angle(0.0)
		{
		}
	};


	struct EdgeAngleLess
	{
		RoomGraph* graph;

		EdgeAngleLess(RoomGraph* g) : graph(g) {}

		bool operator()(int e1, int e2) const;
	};

	// Internal workflow.
	void clear();
	void buildNodesAndEdges(const std::vector<Segment>& segments);
	int findOrCreateNode(const Vec2& p);
	void sortOutgoingByAngle();
	void buildNextRelations();
	void walkCycles();

	double computeSignedArea(const std::vector<Vec2>& poly) const;
	Vec2 computeCentroid(const std::vector<Vec2>& poly, double signedArea) const;

private:
	std::vector<Node>     m_nodes;
	std::vector<HalfEdge> m_edges;
	std::vector<Room>     m_rooms;

	// Simple grid key for snapping nearby points to a single node.
	struct GridKey
	{
		int ix;
		int iy;

		GridKey() : ix(0), iy(0) {}
		GridKey(int x_, int y_) : ix(x_), iy(y_) {}

		bool operator<(const GridKey& other) const
		{
			if (ix != other.ix) return ix < other.ix;
			return iy < other.iy;
		}
	};

	// Map snapped grid coordinates to node index.
	std::map<GridKey, int> m_nodeIndex;

	// Size of the snap grid in world units.
	double m_snapSize;
};



#endif // ROOMGRAPH_H
