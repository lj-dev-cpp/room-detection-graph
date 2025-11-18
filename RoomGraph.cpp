#include "stdafx.h"
#include "stdarx.h"
#include "RoomGraph.h"

#include <algorithm>
#include <cmath>

RoomGraph::RoomGraph()
: m_nodes(),
m_edges(),
m_rooms(),
m_nodeIndex(),
m_snapSize(1e-3) // grid size for snapping points
{
}

void RoomGraph::clear()
{
	m_nodes.clear();
	m_edges.clear();
	m_rooms.clear();
	m_nodeIndex.clear();
}

const std::vector<RoomGraph::Room>& RoomGraph::getRooms() const
{
	return m_rooms;
}

void RoomGraph::build(const std::vector<Segment>& segments)
{
	clear();

	if (segments.empty())
		return;

	// 1) Build nodes and half-edges from raw segments.
	buildNodesAndEdges(segments);

	// 2) Sort outgoing edges at each node by angle.
	sortOutgoingByAngle();

	// 3) For each half-edge, determine the "next" edge when walking a face.
	buildNextRelations();

	// 4) Walk all closed cycles and turn them into rooms.
	walkCycles();
}

// Snap the point to a discrete grid, and reuse existing node if possible.
// This is enough for typical CAD coordinates that are already consistent.
int RoomGraph::findOrCreateNode(const Vec2& p)
{
	const int ix = static_cast<int>(std::floor(p.x / m_snapSize + 0.5));
	const int iy = static_cast<int>(std::floor(p.y / m_snapSize + 0.5));
	GridKey key(ix, iy);

	std::map<GridKey, int>::iterator it = m_nodeIndex.find(key);
	if (it != m_nodeIndex.end())
	{
		return it->second;
	}

	Node node;
	node.id = static_cast<int>(m_nodes.size());
	node.pos = p;

	m_nodes.push_back(node);
	m_nodeIndex.insert(std::make_pair(key, node.id));

	return node.id;
}

// Convert each input segment into two directed half-edges
// and register them on the corresponding nodes.
void RoomGraph::buildNodesAndEdges(const std::vector<Segment>& segments)
{
	m_nodes.reserve(segments.size() * 2);
	m_edges.reserve(segments.size() * 2);

	for (size_t i = 0; i < segments.size(); ++i)
	{
		const Segment& s = segments[i];

		int a = findOrCreateNode(s.a);
		int b = findOrCreateNode(s.b);

		if (a == b)
			continue;

		HalfEdge e1;
		HalfEdge e2;

		e1.id = static_cast<int>(m_edges.size());
		e1.from = a;
		e1.to = b;

		e2.id = e1.id + 1;
		e2.from = b;
		e2.to = a;

		e1.twin = e2.id;
		e2.twin = e1.id;

		const Vec2& pa = m_nodes[a].pos;
		const Vec2& pb = m_nodes[b].pos;

		// Direction angle at "from" node.
		const double dx1 = pb.x - pa.x;
		const double dy1 = pb.y - pa.y;
		e1.angle = std::atan2(dy1, dx1);

		const double dx2 = pa.x - pb.x;
		const double dy2 = pa.y - pb.y;
		e2.angle = std::atan2(dy2, dx2);

		m_nodes[a].outgoingEdges.push_back(e1.id);
		m_nodes[b].outgoingEdges.push_back(e2.id);

		m_edges.push_back(e1);
		m_edges.push_back(e2);
	}
}

// For each node, sort outgoing half-edges by angle.
// This gives a consistent circular ordering around the point.
// Static compare function for sorting edges by angle.
// VC6 does NOT support lambdas.
// Compare two edge indices by their direction angle.
bool RoomGraph::EdgeAngleLess::operator()(int e1, int e2) const
{
	return graph->m_edges[e1].angle < graph->m_edges[e2].angle;
}

// For each node, sort outgoing half-edges by angle.
// This gives a consistent circular ordering around the point.
void RoomGraph::sortOutgoingByAngle()
{
	EdgeAngleLess cmp(this);

	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		Node& node = m_nodes[i];
		std::vector<int>& out = node.outgoingEdges;

		if (out.size() <= 1)
			continue;

		std::sort(out.begin(), out.end(), cmp);
	}
}


// For a given half-edge e: from A to B,
// we stand at B and take the twin(e) as reference,
// then pick the previous edge in the sorted order (turning "right").
// That edge becomes e.next when walking along a face.
void RoomGraph::buildNextRelations()
{
	for (size_t i = 0; i < m_edges.size(); ++i)
	{
		HalfEdge& e = m_edges[i];
		const int toNode = e.to;

		if (toNode < 0 || toNode >= static_cast<int>(m_nodes.size()))
			continue;

		const Node& node = m_nodes[toNode];
		const std::vector<int>& out = node.outgoingEdges;

		if (out.empty())
			continue;

		const int twinId = e.twin;
		int pos = -1;

		for (size_t k = 0; k < out.size(); ++k)
		{
			if (out[k] == twinId)
			{
				pos = static_cast<int>(k);
				break;
			}
		}

		if (pos < 0)
			continue;

		const int n = static_cast<int>(out.size());
		const int nextPos = (pos - 1 + n) % n;

		e.next = out[nextPos];
	}
}

double RoomGraph::computeSignedArea(const std::vector<Vec2>& poly) const
{
	if (poly.size() < 3)
		return 0.0;

	double area = 0.0;
	const size_t n = poly.size();

	for (size_t i = 0; i < n; ++i)
	{
		const Vec2& p = poly[i];
		const Vec2& q = poly[(i + 1) % n];
		area += p.x * q.y - q.x * p.y;
	}

	return 0.5 * area;
}

// Standard polygon centroid (area-weighted).
Vec2 RoomGraph::computeCentroid(const std::vector<Vec2>& poly, double signedArea) const
{
	Vec2 c(0.0, 0.0);

	if (poly.size() < 3)
		return c;

	const size_t n = poly.size();
	const double factor = 1.0 / (6.0 * signedArea);

	double cx = 0.0;
	double cy = 0.0;

	for (size_t i = 0; i < n; ++i)
	{
		const Vec2& p = poly[i];
		const Vec2& q = poly[(i + 1) % n];

		const double cross = p.x * q.y - q.x * p.y;
		cx += (p.x + q.x) * cross;
		cy += (p.y + q.y) * cross;
	}

	c.x = cx * factor;
	c.y = cy * factor;

	return c;
}

// Walk all half-edges and try to follow e.next until we return
// to the starting edge. Each closed loop becomes a Room.
// Only counter-clockwise faces with positive area are kept.
void RoomGraph::walkCycles()
{
	const int edgeCount = static_cast<int>(m_edges.size());

	for (int i = 0; i < edgeCount; ++i)
	{
		HalfEdge& start = m_edges[i];
		if (start.used)
			continue;

		std::vector<Vec2> poly;
		int currentId = start.id;

		while (true)
		{
			HalfEdge& e = m_edges[currentId];
			if (e.used)
				break;

			e.used = true;

			const int fromNode = e.from;
			if (fromNode < 0 || fromNode >= static_cast<int>(m_nodes.size()))
				break;

			poly.push_back(m_nodes[fromNode].pos);

			if (e.next < 0)
				break;

			if (e.next == start.id)
			{
				// Closed loop detected, do not push start node again.
				break;
			}

			currentId = e.next;
		}

		if (poly.size() < 3)
			continue;


		double signedArea = computeSignedArea(poly);
		if (std::fabs(signedArea) < 1e-6)
			continue;

		// Keep only CCW faces as "rooms".
		if (signedArea <= 0.0)
			continue;

		Room room;
		room.polygon = poly;

		// store positive area for display
		room.area = std::fabs(signedArea);

		// centroid needs the signed area
		room.center = computeCentroid(poly, signedArea);

		m_rooms.push_back(room);

	}
}





//////////////////////////////////////////////////////////////////////////
// Command: select LINE entities, build graph, label room centers.
void Cmd_TestRoomGraph()
{
	ads_name ss;
	struct resbuf filter;

	filter.restype = 0;                 // DXF 0 = entity type
	filter.resval.rstring = _T("LINE"); // only LINE
	filter.rbnext = NULL;

	if (acedSSGet(NULL, NULL, NULL, &filter, ss) != RTNORM)
	{
		acutPrintf(_T("\nNothing selected."));
		return;
	}

	Adesk::Int32 len = 0;
	if (acedSSLength(ss, &len) != RTNORM || len == 0)
	{
		acedSSFree(ss);
		acutPrintf(_T("\nNo line entities."));
		return;
	}

	std::vector<Segment> segments;
	segments.reserve(len);

	// Collect segments from selected lines.
	long i;
	for (i = 0; i < len; ++i)
	{
		ads_name en;
		if (acedSSName(ss, i, en) != RTNORM)
			continue;

		AcDbObjectId id;
		if (acdbGetObjectId(id, en) != Acad::eOk)
			continue;

		AcDbLine* pLine = NULL;
		if (acdbOpenObject(pLine, id, AcDb::kForRead) != Acad::eOk || pLine == NULL)
			continue;

		AcGePoint3d s = pLine->startPoint();
		AcGePoint3d e = pLine->endPoint();

		segments.push_back(
			Segment(Vec2(s.x, s.y), Vec2(e.x, e.y))
			);

		pLine->close();
	}

	acedSSFree(ss);

	if (segments.empty())
	{
		acutPrintf(_T("\nNo valid segments."));
		return;
	}

	RoomGraph graph;
	graph.build(segments);

	const std::vector<RoomGraph::Room>& rooms = graph.getRooms();
	int roomCount = static_cast<int>(rooms.size());

	acutPrintf(_T("\nRooms found: %d"), roomCount);

	if (roomCount == 0)
		return;

	// Label each room center with an index.
	int idx;
	
	const double textHeight = 80.0;    // tune this for your drawings
	const double lineGap = textHeight * 0.9; // vertical gap between id and area

	for (idx = 0; idx < roomCount; ++idx)
	{
		const RoomGraph::Room& r = rooms[idx];

		// base position for the id label
		AcGePoint3d cen(r.center.x, r.center.y, 0.0);

		// --- room index ---
		AcDbText* pId = new AcDbText;
		if (pId == NULL)
			continue;

		TCHAR bufId[32];
		_stprintf(bufId, _T("%d"), idx + 1);

		pId->setPosition(cen);
		pId->setHeight(textHeight);
		pId->setTextString(bufId);
		pId->setColorIndex(1);

		AcDbObjectId idTextId;
		if (AddtoModelSpace(pId, idTextId) != Acad::eOk)
		{
			continue; // addEntityToModelSpace deletes pId on failure
		}
		pId->close();

		// --- room area (second line, below the id) ---
		AcDbText* pArea = new AcDbText;
		if (pArea == NULL)
			continue;

		// format area with two decimals; no unit here, keep it generic
		TCHAR bufArea[64];
		_stprintf(bufArea, _T("%.2f m2"), r.area);


		AcGePoint3d posArea = cen;
		posArea.y -= lineGap; // move down in Y

		pArea->setPosition(posArea);
		pArea->setHeight(textHeight * 0.5);
		pArea->setTextString(bufArea);
		pArea->setHorizontalMode(AcDb::kTextMid);
		pArea->setAlignmentPoint(posArea);
		pArea->setColorIndex(3);

		AcDbObjectId areaTextId;
		if (AddtoModelSpace(pArea, areaTextId) != Acad::eOk)
		{
			continue;
		}
		pArea->close();
	}

}
