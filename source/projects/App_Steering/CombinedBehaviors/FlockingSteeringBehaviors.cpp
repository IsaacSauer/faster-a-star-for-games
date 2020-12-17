#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "TheFlock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"
#include "../SpacePartitioning.h"

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Seperation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput result;
	if (m_pFlock->DoSpatialPartitioning())
	{
		if (m_pCellSpace->GetNrOfNeighbors() < 1)
			return result;
	
		Elite::Vector2 tempVel{};
		const auto& neighbors {m_pCellSpace->GetNeighbors()};
		int nrOfNeighbors{ m_pCellSpace->GetNrOfNeighbors() };
	
		for (int i{}; i < nrOfNeighbors; ++i)
		{
			float tempWeight{ 1.f - ((neighbors[i]->GetPosition() - pAgent->GetPosition()).Magnitude() / m_pFlock->GetNeighborRadius()) };
			m_Target.Position = neighbors[i]->GetPosition();
			tempVel += Flee::CalculateSteering(deltaT, pAgent).LinearVelocity * tempWeight;
		}
		result.LinearVelocity = tempVel / float(nrOfNeighbors);
	}
	else
	{
		if (m_pFlock->GetNrOfNeighbors() < 1)
			return result;

		Elite::Vector2 tempVel{};
		const auto& neighbors{m_pFlock->GetNeighbors()};
		int nrOfNeighbors{ m_pFlock->GetNrOfNeighbors() };

		for (int i{}; i < nrOfNeighbors; ++i)
		{
			float tempWeight{ 1.f - ((neighbors[i]->GetPosition() - pAgent->GetPosition()).Magnitude() / m_pFlock->GetNeighborRadius()) };
			m_Target.Position = neighbors[i]->GetPosition();
			tempVel += Flee::CalculateSteering(deltaT, pAgent).LinearVelocity * tempWeight;
		}
		result.LinearVelocity = tempVel / float(nrOfNeighbors);
	}
	return result;
}

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput result;
	m_Target.Position = m_pFlock->GetAverageNeighborPos();
	result = Seek::CalculateSteering(deltaT, pAgent);
	return result;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput result;
	m_Target.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();
	result = Align::CalculateSteering(deltaT, pAgent);
	return result;

}