#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigid bodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon;
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
	auto polyLines{ m_pNavMeshPolygon->GetLines() };
	for (auto curLine : polyLines)
	{
		//Check if that line is connected to another triangle (GetTrianglesFromLineIndex())
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(curLine->index).size() > 1)
		{
			//Create a NavGraphNode on the graph
			//positioned on the middle of the line
			//has as lineIdx the curLine idx
			NavGraphNode* pNewNode(new NavGraphNode(GetNextFreeNodeIndex(), curLine->index, (curLine->p1 + curLine->p2) / 2.f));
			AddNode(pNewNode);
		}
	}

	//2. Create connections now that every node is created
		//Loop over all the triangles
	auto polyTriangles{ m_pNavMeshPolygon->GetTriangles() };
	for (auto curTriangle : polyTriangles)
	{
		std::vector<int> savedLineIndexes{};
		for (auto lineIdx : curTriangle->metaData.IndexLines)
		{
			for (auto node : m_Nodes)
			{
				if (node->GetLineIndex() == lineIdx)
				{
					savedLineIndexes.push_back(node->GetIndex());
				}
			}
		}
		if (savedLineIndexes.size() == 2)
		{
			GraphConnection2D* connection{ new GraphConnection2D(savedLineIndexes[0], savedLineIndexes[1]) };
			AddConnection(connection);
		}
		else if (savedLineIndexes.size() == 3)
		{
			GraphConnection2D* connection1{ new GraphConnection2D(savedLineIndexes[0], savedLineIndexes[1]) };
			GraphConnection2D* connection2{ new GraphConnection2D(savedLineIndexes[1], savedLineIndexes[2]) };
			GraphConnection2D* connection3{ new GraphConnection2D(savedLineIndexes[2], savedLineIndexes[0]) };

			AddConnection(connection1);
			AddConnection(connection2);
			AddConnection(connection3);
		}
	}

	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistance();
}