#pragma once

#include <queue>
#include <set>
#include <utility>
#include <vector>
#include <algorithm>

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class Dijkstra
	{
	public:
		Dijkstra(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

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

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);
		std::vector<T_NodeType*> FindPath(int src);
	private:
		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template <class T_NodeType, class T_ConnectionType>
	Dijkstra<T_NodeType, T_ConnectionType>::Dijkstra(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
    std::vector<T_NodeType*> Dijkstra<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode)
    {
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

				NodeRecord newRecord{};
				newRecord.pConnection = connection;
				newRecord.pNode = m_pGraph->GetNode(connection->GetTo());
				newRecord.costSoFar = totalGCost;
				newRecord.estimatedTotalCost = totalGCost;
				openList.push_back(newRecord);
			}
			//G Remove NodeRecord from the openList and add it to the closedList.
			openList.erase(std::remove(openList.begin(), openList.end(), currentRecord), openList.end());
			closedList.push_back(currentRecord);
		}

		//Reconstruct path from last connection to startNode
		while (currentRecord.pNode != pStartNode)
		{
			path.push_back(currentRecord.pNode);
			currentRecord = *std::find_if(closedList.begin(), closedList.end(), [&currentRecord](NodeRecord A)
				{ return A.pNode->GetIndex() == currentRecord.pConnection->GetFrom(); });
		}
		path.push_back(pStartNode);

		std::reverse(path.begin(), path.end());

		return path;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline std::vector<T_NodeType*> Dijkstra<T_NodeType, T_ConnectionType>::FindPath(int src)
	{
		// Create a set to store vertices that are being 
		// prerocessed 
		set< pair<int, int> > setds;

		// Create a vector for distances and initialize all 
		// distances as infinite (INF) 
		vector<int> dist(m_pGraph->GetNrOfNodes(), INT_MAX);

		// Insert source itself in Set and initialize its 
		// distance as 0. 
		setds.insert(make_pair(0, src));
		dist[src] = 0;

		/* Looping till all shortest distance are finalized
		   then setds will become empty */
		while (!setds.empty())
		{
			// The first vertex in Set is the minimum distance 
			// vertex, extract it from set. 
			pair<int, int> tmp = *(setds.begin());
			setds.erase(setds.begin());

			// vertex label is stored in second of pair (it 
			// has to be done this way to keep the vertices 
			// sorted distance (distance must be first item 
			// in pair) 
			int u = tmp.second;

			// 'i' is used to get all adjacent vertices of a vertex 
			auto adjList = m_pGraph->GetNodeConnections(u);
			for (auto i = adjList.begin(); i != adjList.end(); ++i)
			{
				// Get vertex label and weight of current adjacent 
				// of u. 
				int v = (*i)->GetFrom();
				int weight = (*i)->GetCost();

				//  If there is shorter path to v through u. 
				if (dist[v] > dist[u] + weight)
				{
					/*  If distance of v is not INF then it must be in
						our set, so removing it and inserting again
						with updated less distance.
						Note : We extract only those vertices from Set
						for which distance is finalized. So for them,
						we would never reach here.  */
					if (dist[v] != INT_MAX)
						setds.erase(setds.find(make_pair(dist[v], v)));

					// Updating distance of v 
					dist[v] = dist[u] + weight;
					setds.insert(make_pair(dist[v], v));
				}
			}
		}

		// Print shortest distances stored in dist[]
		printf("Vertex   Distance from Source\n");
		for (int i = 0; i < m_pGraph->GetNrOfNodes(); ++i)
			printf("%d \t\t %d\n", i, dist[i]);

		return std::vector<T_NodeType*>();
	}
}

