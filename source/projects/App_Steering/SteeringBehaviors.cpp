//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "SteeringAgent.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition(); //Desired Velocity
	steering.LinearVelocity.Normalize(); //Normalize Desired Velocity
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed(); //Rescale to Max Speed

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetDirection(), 10.f, Elite::Color(0, 1, 0));
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), m_Target.Position, Elite::Color(1, 0, 0));
	}

	return steering;
}

//WANDER (base> SEEK)
//******
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering;
	TargetData target{};

	pAgent->SetAutoOrient(true);

	int randNr{ Elite::randomInt(3) - 1 };
	if (randNr != 0)
	{
		m_WanderAngle += Elite::randomFloat(float(randNr) * m_AngelChange);
	}

	Elite::Vector2 agentDir{ cosf(pAgent->GetOrientation() - ToRadians(90)), sinf(pAgent->GetOrientation() - ToRadians(90)) };
	Elite::Vector2 circlePos{ pAgent->GetPosition() + (agentDir * m_OffSet) };
	Elite::Vector2 randomPos{ circlePos.x + (cosf(m_WanderAngle) * m_Radius), circlePos.y + (sinf(m_WanderAngle) * m_Radius)};

	target.Position = randomPos;
	m_Target = target;
	steering = Seek::CalculateSteering(deltaT, pAgent);

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), agentDir, m_OffSet, Elite::Color(1, 0, 0), 0);
		DEBUGRENDERER2D->DrawCircle(circlePos, m_Radius, Elite::Color(1, 1, 0), 0);
		DEBUGRENDERER2D->DrawPoint(randomPos, 5.f, Elite::Color(0, 0, 1));
	}

	return steering;
}

//FLEE
//******
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	//if ((pAgent->GetPosition() - m_Target.Position).Magnitude() < radius)
	{
		steering.LinearVelocity = pAgent->GetPosition() - m_Target.Position;
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
		steering.IsValid = true;
	}
	//else
	//	steering.IsValid = false;


	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), radius, Elite::Color(1, 0, 0), 0);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetDirection(), 10.f, Elite::Color(0, 1, 0));
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), m_Target.Position, Elite::Color(1, 0, 0));
	}


	return steering;
}

//ARRIVE (base> SEEK)
//******
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	//SteeringOutput steering{};
	//pAgent->SetAutoOrient(false);
	//const float maxSpeed = 50.f;
	//const float arrivalRadius = 1.f;
	//const float slowRadius = 15.f;
	//Elite::Vector2 toTarget = m_Target.Position - pAgent->GetPosition();
	//const float distance = toTarget.Magnitude();
	//if (distance < arrivalRadius)
	//{
	//	pAgent->SetLinearVelocity(Elite::ZeroVector2);
	//	return steering;
	//}
	//toTarget.Normalize();
	//if (distance < slowRadius)
	//	toTarget *= maxSpeed * (distance / slowRadius);
	//else
	//	toTarget *= maxSpeed;
	//pAgent->SetLinearVelocity(toTarget);
	//
	//const auto angle = pAgent->Orientation(pAgent->GetRotation(), pAgent->GetLinearVelocity());
	//pAgent->GetRigidBody()->SetTransform(Transform(pAgent->GetPosition(), Elite::Vector2(angle, angle)));
	//return steering;

	SteeringOutput steering{ Seek::CalculateSteering(deltaT, pAgent) };
	pAgent->SetAutoOrient(true);

	Elite::Vector2 toTarget{ m_Target.Position - pAgent->GetPosition() };

	if (toTarget.Magnitude() < m_SlowingRadius)
		pAgent->SetLinearVelocity(toTarget);


	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), m_Target.Position, Elite::Color(1, 0, 0));
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetDirection(), 10.f, Elite::Color(0, 1, 0));
	}

	return steering;
}

//FACE
//******
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering;

	Elite::Vector2 AgentDir{ Elite::Vector2{cosf(pAgent->GetRotation() - (float)M_PI / 2.f), sinf(pAgent->GetRotation() - (float)M_PI / 2.f)} };
	Elite::Vector2 toTarget{ m_Target.Position - pAgent->GetPosition() };

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), AgentDir, 5.f, Elite::Color(), 0);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), toTarget, 5.f, Elite::Color(), 0);
	}

	float angle{ atan2f(AgentDir.Cross(toTarget), AgentDir.Dot(toTarget)) };
	//std::cout << angle << std::endl;

	return steering;
}

SteeringOutput Align::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput result{};
	Elite::Vector2 toTarget{ (m_Target.Position - pAgent->GetPosition()).GetNormalized() };
	Elite::Vector2 currentDirection{ Elite::Vector2(pAgent->GetDirection().x + cosf(-90.f / 180.f * float(M_PI)), pAgent->GetDirection().y + sinf(-90.f / 180.f * float(M_PI))).GetNormalized() };

	//Get the naive direction to the target
	float agentRotation{ atan2f(currentDirection.y, currentDirection.x) };

	float rotation{ atan2f(Cross(toTarget, currentDirection), Dot(toTarget, currentDirection)) };


	//Map the result to the (-pi, pi) interval.
	rotation = mapToRange(rotation);
	float rotationSize = abs(rotation);

	////Check if we are there, return no steering
	//if (rotationSize < m_TargetRadius)
	//	return SteeringOutput();
	////If we are outside the slowRadius, then use maximum rotation.
	//if (rotationSize > m_SlowRadius)
	//	m_Target.Orientation = m_MaxRot;
	//else // calc scaled rot
	//	m_Target.Orientation = m_MaxRot * rotationSize / m_SlowRadius;
	////The final target rotation combines speed (already in the variable) and direction.
	//m_Target.Orientation *= rotation / rotationSize;
	////Acceleration tries to get to the target rotation.
	//result.AngularVelocity = m_Target.Orientation - pAgent->GetOrientation();
	//result.AngularVelocity /= m_TimeToTarget;
	////Check if the acceleration is too great.
	//float angularAcc = abs(result.AngularVelocity);
	//if (angularAcc > m_MaxAngAcc)
	//{
	//	result.AngularVelocity /= angularAcc;
	//	result.AngularVelocity *= m_MaxAngAcc;
	//}
	//result.LinearVelocity = Elite::ZeroVector2;

	return result;
}

SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Seek::SetTarget(m_Target);
	SteeringOutput result{ Seek::CalculateSteering(deltaT, pAgent) };

	result.LinearVelocity += m_Target.LinearVelocity;

	return result;
}

SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput result{ Flee::CalculateSteering(deltaT, pAgent) };

	//result.LinearVelocity += m_Target.LinearVelocity;

	return result;
}
