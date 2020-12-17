#pragma once
#include "../SteeringHelpers.h"
#include "FlockingSteeringBehaviors.h"

class ISteeringBehavior;
class SteeringAgent;
class BlendedSteering;
class PrioritySteering;
class CellSpace;

class Flock
{
public:
	Flock(
		int flockSize = 50, 
		float worldSize = 100.f, 
		SteeringAgent* pAgentToEvade = nullptr, 
		bool trimWorld = false);

	~Flock();

	void Update(float deltaT, const TargetData& mouseTarget);
	void UpdateAndRenderUI();
	void Render(float deltaT);

	void RegisterNeighbors(SteeringAgent* pAgent);
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const vector<SteeringAgent*>& GetNeighbors() const { return m_Neighbors; }

	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;
	float GetNeighborRadius() const;
	bool DoSpatialPartitioning() const { return m_DoSpatialPartition; }
private:
	// Render Parameters
	bool m_CanRenderSteering{ false };
	bool m_CanRenderNeighborhood{ true };
	bool m_CanRenderPartitions{ true };
	bool m_DoSpatialPartition{ true };

	// Blended Behaviors
	Seperation* m_pSeperation = nullptr;
	Cohesion*   m_pCohesion   = nullptr;
	Alignment*  m_pAlignment  = nullptr;
	Seek*       m_pSeek       = nullptr;
	Wander*     m_pWander     = nullptr;
	// Prioritized Behavior
	Evade* m_pEvade = nullptr;

	// flock agents
	int m_FlockSize = 0;
	vector<SteeringAgent*> m_Agents;
	vector<Elite::Vector2> m_AgentOldPos;

	// neighborhood agents
	vector<SteeringAgent*> m_Neighbors;
	float m_NeighborhoodRadius = 10.f;
	int m_NrOfNeighbors = 0;

	// evade target
	SteeringAgent* m_pAgentToEvade = nullptr;

	// world info
	bool m_TrimWorld = false;
	float m_WorldSize = 0.f;
	
	// steering Behaviors
	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	// private functions
	float* GetWeight(ISteeringBehavior* pBehaviour);

	// Space Partitioning
	CellSpace* m_pCellSpace = nullptr;
private:
	Flock(const Flock& other);
	Flock& operator=(const Flock& other);
};