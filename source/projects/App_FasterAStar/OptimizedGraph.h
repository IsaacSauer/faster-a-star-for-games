#pragma once

#include "../../../framework/EliteAI/EliteGraphs/EIGraph.h"
//#include "../../../framework/EliteAI/EliteGraphs/EGraph2D.h"
#include <vector>
#include <map>
#include <set>
#include <algorithm>

struct OSquare
{
	OSquare() = default;
	OSquare(float l, float r, float b, float t) :left{ l }, right{ r }, bottom{ b }, top{ t }{}

	bool IsInside(const Elite::Vector2& p)
	{
		return p.x > left && p.x < right && p.y > bottom && p.y < top;
	}
	float GetArea() const { return abs((right - left) * (top - bottom)); }
	float left, right, bottom, top;
};

struct NodeInfo
{
	NodeInfo() = default;
	NodeInfo(int start) :sides{ std::vector<std::pair<int, OSquare>>{} }, optimalStart{ start }{}

	std::vector<std::pair<int, OSquare>> sides{};
	std::vector<int> optimalStart{};
};

template<class T_NodeType, class T_ConnectionType>
class OptimizedGraph
{
public:
	OptimizedGraph(Elite::IGraph<T_NodeType, T_ConnectionType>* pGraph) :m_pGraph{ pGraph } {}

	// stores the optimal connection to a node and its total costs related to the start and end node of the path
	struct NodeRecord
	{
		T_NodeType* pNode = nullptr;
		T_ConnectionType* pConnection = nullptr;
		float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
		float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

		T_ConnectionType* pStartConnection = nullptr;

		bool operator==(const NodeRecord& other) const
		{
			return pNode == other.pNode
				&& pConnection == other.pConnection
				&& costSoFar == other.costSoFar
				&& estimatedTotalCost == other.estimatedTotalCost;
		};

		bool operator<(const NodeRecord& other) const
		{
			return estimatedTotalCost < other.estimatedTotalCost;
		};
	};

	bool ComputeBoundingBoxes(Elite::Polygon* navMesh);
	bool IsWithinBoundingBox(T_NodeType* currentNode, const T_ConnectionType& d, const Elite::Vector2& pos);
	void EnhancedDijkstra(int src, std::vector<T_ConnectionType*>& optimalConnections);

	Elite::IGraph<T_NodeType, T_ConnectionType>* GetGraph() const { return m_pGraph; }
	const std::pair<int, OSquare>& GetConnection(int from, int idx) const;
private:
	Elite::IGraph<T_NodeType, T_ConnectionType>* m_pGraph = nullptr;

	//Linked with each node Idx from m_pGraph
	//vector<vector<pair<"connection->from", OSquare>>> m_BoundingBoxes;
	std::vector<NodeInfo> m_BoundingBoxes;
};

template<class T_NodeType, class T_ConnectionType>
inline bool OptimizedGraph<T_NodeType, T_ConnectionType>::ComputeBoundingBoxes(Elite::Polygon* navMesh)
{
	//nodes[actualNodeIdx][actualNodeIdx] = Which Side is most optimal
	std::vector<std::vector<T_ConnectionType*>> optimalSides(
		m_pGraph->GetNrOfNodes()
		, std::vector<T_ConnectionType*>(m_pGraph->GetNrOfNodes(), nullptr)
	);

	for (int i{}; i < m_pGraph->GetNrOfNodes(); ++i)
	{
		if (!m_pGraph->IsNodeValid(i))
			continue;

		EnhancedDijkstra(i, optimalSides[i]);

		std::vector<T_ConnectionType*> edges = optimalSides[i];
		sort(edges.begin(), edges.end());
		edges.erase(unique(edges.begin(), edges.end()), edges.end());

		// The final task is to iterate through all nodes in the map and build up the bounding boxes that contain each starting node edge.
		Vector2 startPos = m_pGraph->GetNodeWorldPos(i);
		m_BoundingBoxes.push_back(NodeInfo{});

		for (auto& sidePerNode : optimalSides[i])
			m_BoundingBoxes.back().optimalStart.push_back(sidePerNode->GetTo());

		for (auto it : edges)
		{
			m_BoundingBoxes.back().sides.push_back({});

			float left{ startPos.x };
			float right{ startPos.x };
			float bottom{ startPos.y };
			float top{ startPos.y };

			for (size_t j{}; j < optimalSides[i].size(); ++j)
			{
				if (it->GetTo() == optimalSides[i][j]->GetTo())
				{
					Vector2 pos = m_pGraph->GetNodeWorldPos(j);

					left = (pos.x < left) ? pos.x : left;
					right = (pos.x > right) ? pos.x : right;
					bottom = (pos.y < bottom) ? pos.y : bottom;
					top = (pos.y > top) ? pos.y : top;

					m_BoundingBoxes.back().sides.back() = { it->GetTo(), OSquare(left, right, bottom, top) };
				}
			}
		}

		for (auto& box : m_BoundingBoxes.back().sides)
		{
			Elite::Vector2 leftBottom{ box.second.left, box.second.bottom };
			Elite::Vector2 rightTop{ box.second.right, box.second.top };
		
			auto triangleLeftBottom = navMesh->GetTriangleFromPosition(leftBottom);
			if (triangleLeftBottom)
			{
				auto triangleLeftBottomPoints = triangleLeftBottom->GetPointsInVector();
		
				leftBottom = *std::min_element(triangleLeftBottomPoints.begin(), triangleLeftBottomPoints.end(),
					[](const Elite::Vector2& A, const Elite::Vector2& B)
					{return A.x < B.x&& A.y < B.y; });
			}
			else
				leftBottom = { navMesh->GetPosVertMinXPos(), navMesh->GetPosVertMinYPos() };
		
			auto triangleRightTop = navMesh->GetTriangleFromPosition(rightTop);
			if (triangleRightTop)
			{
				auto triangleRightTopPoints = triangleRightTop->GetPointsInVector();
		
				rightTop = *std::max_element(triangleRightTopPoints.begin(), triangleRightTopPoints.end(),
					[](const Elite::Vector2& A, const Elite::Vector2& B)
					{return A.x < B.x&& A.y < B.y; });
			}
			else
				rightTop = { navMesh->GetPosVertMaxXPos(), navMesh->GetPosVertMaxYPos() };
		
			box.second = { leftBottom.x, rightTop.x, leftBottom.y, rightTop.y };
		}
	}

	////sort all boundingboxes of each node from less to greater
	//for (auto& boundingBoxes : m_BoundingBoxes)
	//	std::sort(boundingBoxes.sides.begin(), boundingBoxes.sides.end(), [](const std::pair<int, OSquare>& A, const std::pair<int, OSquare>& B)
	//		{
	//			return A.second.GetArea() < B.second.GetArea();
	//		});
	//
	return true;
}

template<class T_NodeType, class T_ConnectionType>
inline bool OptimizedGraph<T_NodeType, T_ConnectionType>::IsWithinBoundingBox(T_NodeType* currentNode, const T_ConnectionType& d, const Elite::Vector2& pos)
{
	//auto node = m_pGraph->GetClosestNodeFromPosition(pos);

 	auto boundingBox = std::find_if(m_BoundingBoxes[currentNode->GetIndex()].sides.begin(), m_BoundingBoxes[currentNode->GetIndex()].sides.end(),
 		[&d](const std::pair<int, OSquare>& A)
 		{
 			return A.first == d.GetTo();
 		});
 
 	if (boundingBox == m_BoundingBoxes[currentNode->GetIndex()].sides.end())
 		return false;
 
	return boundingBox->second.IsInside(pos);
}

template<class T_NodeType, class T_ConnectionType>
inline void OptimizedGraph<T_NodeType, T_ConnectionType>::EnhancedDijkstra(int src, std::vector<T_ConnectionType*>& optimalConnections)
{
	//Source: http://www.gameaipro.com/GameAIPro3/GameAIPro3_Chapter22_Faster_A_Star_with_Goal_Bounding.pdf
	/* We will start the Dijkstra search at our single node and give it no destination,
	causing it to search all nodes in the map, as if it was performing a floodfill.
	Using Dijkstra to floodfill, the map has the effect of marking every node with the optimal “next step” to optimally get back to the start node.
	*/
	/* This next step is simply the parent pointer that is recorded during the search.
	However, the crucial piece of information that we really want to know for a given node is not the next step to take,
	but which starting node edge was required to eventually get to that node.
	Think of every node in the map as being marked with the starting node’s edge that is on the optimal path back to the starting node.
	
	In a Dijkstra search, this starting node edge is normally not recorded, but now we need to store this information.
	"Every node’s data structure needs to contain a new value representing this starting node edge."
	During the Dijkstra search, "when the neighbors of a node are explored," 
	the starting node edge is passed down to the neighboring nodes as they are placed on the open list.
	This transfers the starting node edge information from node to node during the search.
	*/


	T_NodeType* pStartNode = m_pGraph->GetNode(src);
	T_NodeType* pDestinationNode = pStartNode;
	//Dijkstra  algorithm
	std::vector<T_NodeType*> path;
	std::vector<NodeRecord> openList;
	std::vector<NodeRecord> closedList;
	NodeRecord currentRecord;

	//Add the start node to OPEN
	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.estimatedTotalCost = startRecord.costSoFar;
	openList.push_back(startRecord);

	//Loop
	while (!openList.empty())
	{
		//Set the currentRecord to the best record from the openList (The one with the lowest F-cost)
		currentRecord = *std::min_element(openList.begin(), openList.end(), [](NodeRecord A, NodeRecord B) {return A < B; });

		//Check if that connection leads to the end node
		if (currentRecord.pConnection)
		{
			if (currentRecord.pConnection->GetTo() == pDestinationNode->GetIndex())
				break;
		}

		//Else, we get all the connections of the connection's end node (neighbors of the currentNode.pNode)
		std::list<T_ConnectionType*> connections{ m_pGraph->GetNodeConnections(currentRecord.pNode->GetIndex()) };
		for (auto& connection : connections)
		{
			float totalGCost = connection->GetCost() + currentRecord.costSoFar;

			auto nodeInClosedList{ std::find_if(closedList.begin(), closedList.end(), [&connection](NodeRecord A) {return A.pNode->GetIndex() == connection->GetTo(); }) };
			auto nodeInOpenList{ std::find_if(openList.begin(), openList.end(), [&connection](NodeRecord A) {return A.pNode->GetIndex() == connection->GetTo(); }) };
			if (nodeInClosedList != closedList.end())
			{
				if (nodeInClosedList->costSoFar < totalGCost)
					continue;
				else
					closedList.erase(std::remove(closedList.begin(), closedList.end(), *nodeInClosedList), closedList.end());
			}
			else if (nodeInOpenList != openList.end())
			{
				if (nodeInOpenList->costSoFar < totalGCost)
					continue;
				else
					openList.erase(std::remove(openList.begin(), openList.end(), *nodeInOpenList), openList.end());
			}

//Check if start node
			if (currentRecord.pNode == pStartNode)
				currentRecord.pStartConnection = connection;

			NodeRecord newRecord{};
			newRecord.pConnection = connection;
			newRecord.pNode = m_pGraph->GetNode(connection->GetTo());
			newRecord.costSoFar = totalGCost;
			newRecord.estimatedTotalCost = totalGCost;

//Assign the root connection to the next node
			newRecord.pStartConnection = currentRecord.pStartConnection;

			openList.push_back(newRecord);
		}
		//G Remove NodeRecord from the openList and add it to the closedList.
		openList.erase(std::remove(openList.begin(), openList.end(), currentRecord), openList.end());
		closedList.push_back(currentRecord);
	}
	for (size_t i{}; i < closedList.size(); ++i)
	{
		optimalConnections[i] = closedList[i].pStartConnection;
	}
}

template<class T_NodeType, class T_ConnectionType>
inline const std::pair<int, OSquare>& OptimizedGraph<T_NodeType, T_ConnectionType>::GetConnection(int from, int idx) const
{
	assert((from < (int)m_BoundingBoxes.size()) &&
		(from >= 0) &&
		"<OptimizedGraph::GetConnection>: invalid 'from' index");

	assert((idx < (int)m_BoundingBoxes[from].sides.size()) &&
		(idx >= 0) &&
		"<OptimizedGraph::GetConnection>: invalid 'to' index");

	if(from < (int)m_BoundingBoxes.size())
		if(idx < (int)m_BoundingBoxes[from].sides.size())
			return m_BoundingBoxes[from].sides[idx];

	return {};
}
