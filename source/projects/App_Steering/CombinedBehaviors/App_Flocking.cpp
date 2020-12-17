//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_Flocking.h"
#include "../SteeringAgent.h"
#include "TheFlock.h"

//Destructor
App_Flocking::~App_Flocking()
{	
	SAFE_DELETE(m_pFlock);
	SAFE_DELETE(m_pWander);
	SAFE_DELETE(m_pAgentToEvade);

}

//Functions
void App_Flocking::Start()
{
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(55.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(m_TrimWorldSize / 1.5f, m_TrimWorldSize / 2));



	m_pWander = new Wander();
	m_pAgentToEvade = new SteeringAgent{};
	m_pAgentToEvade->SetPosition({ randomFloat(m_TrimWorldSize), randomFloat(m_TrimWorldSize) });
	m_pAgentToEvade->SetSteeringBehavior(m_pWander);
	m_pAgentToEvade->SetMaxLinearSpeed(15.f);
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetMass(1.f);
	m_pAgentToEvade->SetBodyColor(Elite::Color(1,0,0));

	m_pFlock = new Flock(m_FlockSize, m_TrimWorldSize, m_pAgentToEvade);
}

void App_Flocking::Update(float deltaTime)
{
	//INPUT
	if (INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eLeft) && m_VisualizeMouseTarget)
	{
		auto const mouseData = INPUTMANAGER->GetMouseData(InputType::eMouseButton, InputMouseButton::eLeft);
		m_MouseTarget.Position = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld({ static_cast<float>(mouseData.X), static_cast<float>(mouseData.Y) });
	}


	m_pAgentToEvade->TrimToWorld({ 0,0 }, { m_TrimWorldSize, m_TrimWorldSize });
	m_pAgentToEvade->Update(deltaTime);
	m_pFlock->Update(deltaTime, m_MouseTarget);
	m_pFlock->UpdateAndRenderUI();
}

void App_Flocking::Render(float deltaTime) const
{
	m_pAgentToEvade->Render(deltaTime);
	m_pFlock->Render(deltaTime);

	std::vector<Elite::Vector2> points =
	{
		{ 0,m_TrimWorldSize },
		{ m_TrimWorldSize,m_TrimWorldSize },
		{ m_TrimWorldSize,0 },
		{0,0 }
	};
	DEBUGRENDERER2D->DrawPolygon(&points[0], 4, { 1,0,0,1 }, 0.4f);

	//Render Target
	if(m_VisualizeMouseTarget)
		DEBUGRENDERER2D->DrawSolidCircle(m_MouseTarget.Position, 0.3f, { 0.f,0.f }, { 1.f,0.f,0.f },-0.8f);

}
