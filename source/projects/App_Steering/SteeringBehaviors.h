/*=============================================================================*/
// Copyright 2017-2018 Elite Engine
// Authors: Matthieu Delaere, Thomas Goussaert
/*=============================================================================*/
// SteeringBehaviors.h: SteeringBehaviors interface and different implementations
/*=============================================================================*/
#ifndef ELITE_STEERINGBEHAVIORS
#define ELITE_STEERINGBEHAVIORS

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "SteeringHelpers.h"
class SteeringAgent;
using namespace Elite;

#pragma region **ISTEERINGBEHAVIOR** (BASE)
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) = 0;

	//Seek Functions
	void SetTarget(const TargetData& target) { m_Target = target; }

	template<class T, typename std::enable_if<std::is_base_of<ISteeringBehavior, T>::value>::type* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	TargetData m_Target;
};
#pragma endregion
///////////////////////////////////////
//SEEK
//****
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	//Seek Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

	//Seek Functions
	virtual void SetTarget(const TargetData& pTarget) { m_Target = pTarget; };

protected:
	const TargetData* m_pTargetRef = nullptr;

};

///////////////////////////////////////
//FLEE
//****
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	//Flee Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

	float GetFleeRadius() const { return radius; }
private:
	float radius = 20.f;
};


///////////////////////////////////////
//FACE
//****
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	//Face Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	float m_RotSpeed = {float(M_PI) * 3.f}; //In radians
	float m_SlowAngle = { 15.f / (180.f * float(M_PI)) };
	float m_TimeToSlow = 0.25f;
};

///////////////////////////////////////
//ARRIVE
//****
class Arrive : public Seek
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	//Arrive Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	void SetSlowRadius(float value) { m_SlowingRadius = value; }
	void SetTargetRadius(float value) { m_ArrivalRadius = value; }
private:
	float m_SlowingRadius = 5.f;
	float m_MaxSpeed = 20.f;
	float m_TimeToTarget = 0.25f;



	float m_ArrivalRadius = 1.f;
	float m_SlowingSpeed = 5.f;
};

//////////////////////////
//WANDER
//******
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	//Wander Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	void SetWanderOffset(float nr) { m_OffSet = nr; }

protected:
	float m_OffSet = 6.f; //Offset (Agent Direction)
	float m_Radius = 4.f; //WanderRadius
	float m_AngelChange = ToRadians(45); //Max WanderAngle change per frame
	float m_WanderAngle = 0.f; //Internal

private:
	//void SetTarget(const TargetData* pTarget) override {} //Hide SetTarget, No Target

};

///////////////////////////////////////
//PURSUIT
//****
class Pursuit : public Seek
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	//Pursuit Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};

///////////////////////////////////////
//EVADE
//****
class Evade : public Flee
{
public:
	Evade() = default;
	virtual ~Evade() = default;

	//Evade Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

private:
	float m_MaxPredTime{ 1.f }; // in seconds
};

///////////////////////////////////////
//ALIGN
//****
class Align : public ISteeringBehavior
{
public:
	Align() = default;
	virtual ~Align() = default;

	//Evade Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

private:
	float m_MaxAngAcc{float(M_PI)};
	float m_MaxRot{ float(M_PI) };
	float m_TargetRadius{ 0.1f};
	float m_SlowRadius{1.f};
	float m_TimeToTarget{0.1f};

	float mapToRange(float value)
	{
		float scalar{ value / m_MaxRot };
		return ((float)M_PI * 2 * scalar) - float(M_PI);
	}
};

#endif


