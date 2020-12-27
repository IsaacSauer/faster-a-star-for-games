#pragma once

#include "../../../framework/EliteAI/EliteGraphs/EIGraph.h"
#include <vector>
#include <map>
#include <set>
#include <algorithm>

struct OSquare
{
	bool IsInside(const Elite::Vector2& p)
	{
		return p.x > x && p.y > y && p.x < x + width && p.y < y + height;
	}
	float x, y, width, height;
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

	bool ComputeBoundingBoxes();
	bool IsWithinBoundingBox(int goal, const T_ConnectionType& d);
	void EnhancedDijkstra(int src, std::vector<T_ConnectionType*>& optimalConnections);

	Elite::IGraph<T_NodeType, T_ConnectionType>* GetGraph() const { return m_pGraph; }
private:
	Elite::IGraph<T_NodeType, T_ConnectionType>* m_pGraph = nullptr;

	//Linked with each node Idx from m_pGraph
	//vector<map<"connection->from", OSquare>> m_BoundingBoxes;
	std::vector<std::map<int, OSquare>> m_BoundingBoxes;
};

template<class T_NodeType, class T_ConnectionType>
inline bool OptimizedGraph<T_NodeType, T_ConnectionType>::ComputeBoundingBoxes()
{
	//nodes[actualNodeIdx][actualNodeIdx] = Which Side is most optimal
	std::vector<std::vector<T_ConnectionType*>> optimalSides(
		m_pGraph->GetNrOfNodes()
		, std::vector<T_ConnectionType*>(m_pGraph->GetNrOfNodes(), nullptr)
	);

	for (int i{}; i < m_pGraph->GetNrOfNodes(); ++i)
	{
		if(!m_pGraph->IsNodeValid(i))
			continue;

		EnhancedDijkstra(i, optimalSides[i]);

		std::vector<T_ConnectionType*> edges = optimalSides[i];
		sort(edges.begin(), edges.end());
		edges.erase(unique(edges.begin(), edges.end()), edges.end());

		// The final task is to iterate through all nodes in the map and build up the bounding boxes that contain each starting node edge.
		for (auto it : edges)
		{
			for (auto side : optimalSides[i])
			{
				int from = side->GetFrom();

				float left{FLT_MIN};
				float right{FLT_MAX};
				float bottom{FLT_MIN};
				float top{FLT_MAX};
				
				Vector2 pos = m_pGraph->GetNode(from)->GetPosition();

				//left = (pos.x)
			}
		}
		
	}


	return false;
}

template<class T_NodeType, class T_ConnectionType>
inline bool OptimizedGraph<T_NodeType, T_ConnectionType>::IsWithinBoundingBox(int goal, const T_ConnectionType& d)
{
	if (!m_pGraph->IsNodeValid(goal) || m_pGraph->IsNodeValid(d.GetTo()) || m_pGraph->IsNodeValid(d.GetFrom()))
		return false;

	return m_BoundingBoxes[d.GetFrom()][d.GetTo()].IsInside(m_pGraph->GetNodeWorldPos(d.GetTo()));
}

template<class T_NodeType, class T_ConnectionType>
inline void OptimizedGraph<T_NodeType, T_ConnectionType>::EnhancedDijkstra(int src, std::vector<T_ConnectionType*>& optimalConnections)
{
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

			if (currentRecord.pNode == pStartNode)
				currentRecord.pStartConnection = connection;

			NodeRecord newRecord{};
			newRecord.pConnection = connection;
			newRecord.pNode = m_pGraph->GetNode(connection->GetTo());
			newRecord.costSoFar = totalGCost;
			newRecord.estimatedTotalCost = totalGCost;
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
		std::cout << closedList[i].pStartConnection << std::endl;
	}
	std::cout << std::endl;
}
