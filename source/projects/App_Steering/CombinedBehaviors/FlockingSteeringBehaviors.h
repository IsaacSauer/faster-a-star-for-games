#pragma once
#include "../SteeringBehaviors.h"
class Flock;
class CellSpace;

//SEPARATION - FLOCKING
//*********************
class Seperation : public Flee
{
public:
	Seperation() = default;
	virtual ~Seperation() = default;

	//Seperation Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent * pAgent) override;
	//Setters
	void SetFlock(Flock* pFlock) {m_pFlock = pFlock;}
	void SetCellSpace(CellSpace* pCellSpace) { m_pCellSpace = pCellSpace;}
private:
	Flock* m_pFlock = nullptr;
	CellSpace* m_pCellSpace = nullptr;
};

//COHESION - FLOCKING
//*******************
class Cohesion : public Arrive
{
public:
	Cohesion() = default;
	virtual ~Cohesion() = default;

	// Cohesion behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	//Setters
	void SetFlock(Flock* pFlock) { m_pFlock = pFlock; }
	void SetCellSpace(CellSpace* pCellSpace) { m_pCellSpace = pCellSpace; }
private:
	Flock* m_pFlock = nullptr;
	CellSpace* m_pCellSpace = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class Alignment : public Align
{
public:
	Alignment() = default;
	virtual ~Alignment() = default;

	// Alignment behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	//Setter
	void SetFlock(Flock* pFlock) { m_pFlock = pFlock; }
	void SetCellSpace(CellSpace* pCellSpace) { m_pCellSpace = pCellSpace; }
private:
	Flock* m_pFlock = nullptr;
	CellSpace* m_pCellSpace = nullptr;
};