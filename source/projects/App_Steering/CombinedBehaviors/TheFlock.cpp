#include "stdafx.h"
#include "TheFlock.h"

#include "../SteeringAgent.h"
#include "../SteeringBehaviors.h"
#include "CombinedSteeringBehaviors.h"
#include "FlockingSteeringBehaviors.h"
#include "../SpacePartitioning.h"
#include <cmath>

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/,
	float worldSize /*= 100.f*/,
	SteeringAgent* pAgentToEvade /*= nullptr*/,
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld{ trimWorld }
	, m_pAgentToEvade{ pAgentToEvade }
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{ 0 }
{
	// Init CellSpace
	m_pCellSpace = new CellSpace(m_WorldSize, m_WorldSize, int(m_WorldSize / 7.f), int(m_WorldSize / 7.f), m_FlockSize);

	// Init behaviors
	m_pSeperation = new Seperation();
	m_pCohesion   = new Cohesion();
	m_pAlignment  = new Alignment();

	m_pSeperation-> SetFlock(this);
	m_pCohesion->   SetFlock(this);
	m_pAlignment->  SetFlock(this);
	m_pSeperation-> SetCellSpace(m_pCellSpace);
	m_pCohesion->   SetCellSpace(m_pCellSpace);
	m_pAlignment->  SetCellSpace(m_pCellSpace);

	m_pSeek = new Seek();
	m_pWander = new Wander();
	m_pEvade = new Evade();

	m_pBlendedSteering = new BlendedSteering({ {m_pSeperation, 1.f}
		,{m_pCohesion, 0.f}
		,{m_pAlignment, 0.f}
		,{m_pSeek, 0.f}
		,{m_pWander, 0.f} });

	m_pPrioritySteering = new PrioritySteering({ m_pEvade, m_pBlendedSteering });
	
	// Init FlockAgents
	for (int i = 0; i < flockSize; ++i)
	{
		m_Agents.push_back(new SteeringAgent());
		m_Agents.at(i)->SetPosition({randomFloat(m_WorldSize), randomFloat(m_WorldSize)});
		m_Agents.at(i)->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents.at(i)->SetMaxLinearSpeed(15.f);
		m_Agents.at(i)->SetAutoOrient(true);
		m_Agents.at(i)->SetMass(1.f);
		m_Agents.at(i)->SetMaxLinearSpeed(200.f);



		m_pCellSpace->AddAgent(m_Agents[i]);
	}
	m_AgentOldPos.resize(m_Agents.size());

	//Resize neighborhood vector
	m_Neighbors.resize(m_Agents.size() - 1);

}

Flock::~Flock()
{
	SAFE_DELETE(m_pSeperation);
	SAFE_DELETE(m_pCohesion);
	SAFE_DELETE(m_pAlignment);
	SAFE_DELETE(m_pSeek);
	SAFE_DELETE(m_pWander);
	SAFE_DELETE(m_pEvade);
	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);

	for (auto agent : m_Agents)
	{
		SAFE_DELETE(agent);
	}

	SAFE_DELETE(m_pCellSpace);
}

void Flock::Update(float deltaT, const TargetData& mouseTarget)
{
	// loop over all the boids
	// register its neighbors
	// update it
	// trim it to the world

	TargetData agentToEvade(m_pAgentToEvade->GetPosition(), {}, m_pAgentToEvade->GetLinearVelocity(), m_pAgentToEvade->GetAngularVelocity());
	m_pSeek->SetTarget(mouseTarget);
	m_pEvade->SetTarget(agentToEvade);

	for (size_t i{}; i < m_Agents.size(); ++i)
	{
		if (m_DoSpatialPartition)
		{
			m_pCellSpace->UpdateAgentCell(m_Agents[i], m_AgentOldPos[i]);
			m_pCellSpace->RegisterNeighbors(m_Agents[i], m_NeighborhoodRadius);

			m_Agents[i]->Update(deltaT);
			m_Agents[i]->TrimToWorld({ 0,0 }, { m_WorldSize, m_WorldSize });
			m_AgentOldPos[i] = m_Agents[i]->GetPosition();
		}
		else
		{
			RegisterNeighbors(m_Agents[i]);
			m_Agents[i]->Update(deltaT);
			m_Agents[i]->TrimToWorld({ 0,0 }, { m_WorldSize, m_WorldSize });
		}
	}
	//m_pCellSpace->RenderAgent(m_Agents[0], deltaT);

	if (!m_DoSpatialPartition)
	{
		m_AgentOldPos.clear();
	}
}

void Flock::Render(float deltaT)
{
	//for (auto agent : m_Agents)
	//{
	//	agent->Render(deltaT);
	//}

	// Debug render steering
	m_Agents[0]->SetRenderBehavior(m_CanRenderSteering);

	// Debug render partitions
	if (m_CanRenderPartitions)
	{
		m_pCellSpace->RenderCells();
		for (int i{}; i < m_pCellSpace->GetNrOfNeighbors(); ++i)
		{
			DEBUGRENDERER2D->DrawCircle(m_Agents[m_FlockSize - 1]->GetPosition(), m_NeighborhoodRadius, Elite::Color(0, 1, 0), 0.8f);
			m_pCellSpace->GetNeighbors()[i]->SetBodyColor(Elite::Color(0, 1, 0));
			m_pCellSpace->GetNeighbors()[i]->Render(deltaT);
		}
	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();

	// Implement checkboxes and sliders here
	ImGui::Checkbox("Trim World", &m_TrimWorld);
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Checkbox("Debug render steering", &m_CanRenderSteering);
	ImGui::Checkbox("Debug render neighborhood", &m_CanRenderNeighborhood);
	ImGui::Checkbox("Debug render partitions", &m_CanRenderPartitions);
	ImGui::Checkbox("Spatial Partition", &m_DoSpatialPartition);

	ImGui::Text("Behavior Weights");
	ImGui::Spacing();
	ImGui::SliderFloat("Seperation", &m_pBlendedSteering->m_WeightedBehaviors.at(0).weight, 0.f, 1.f, "%.2f");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->m_WeightedBehaviors.at(1).weight, 0.f, 1.f, "%.2f");
	ImGui::SliderFloat("Velocity Match", &m_pBlendedSteering->m_WeightedBehaviors.at(2).weight, 0.f, 1.f, "%.2f");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->m_WeightedBehaviors.at(3).weight, 0.f, 1.f, "%.2f");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->m_WeightedBehaviors.at(4).weight, 0.f, 1.f, "%.2f");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	// register the agents neighboring the currently evaluated agent
	// store how many they are, so you know which part of the vector to loop over
	m_NrOfNeighbors = 0;
	for (auto curAgent : m_Agents)
	{
		if (curAgent != pAgent && Elite::Vector2{ curAgent->GetPosition() - pAgent->GetPosition() }.Magnitude() < m_NeighborhoodRadius)
		{
			m_Neighbors.at(m_NrOfNeighbors) = curAgent;
			++m_NrOfNeighbors;
		}
	}
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Elite::Vector2 pos{};
	if (m_DoSpatialPartition)
	{
		for (int i{}; i < m_pCellSpace->GetNrOfNeighbors(); ++i)
		{
			pos += m_pCellSpace->GetNeighbors()[i]->GetPosition();
		}
		pos /= float(m_pCellSpace->GetNrOfNeighbors());
	}
	else
	{
		for (int i{}; i < m_NrOfNeighbors; ++i)
		{
			pos += m_Neighbors[i]->GetPosition();
		}
		pos /= float(m_NrOfNeighbors);
	}
	return pos;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Elite::Vector2 vel{};
	if (m_DoSpatialPartition)
	{
		for (int i{}; i < m_pCellSpace->GetNrOfNeighbors(); ++i)
		{
			vel += m_pCellSpace->GetNeighbors()[i]->GetLinearVelocity();
		}
		vel /= float(m_pCellSpace->GetNrOfNeighbors());
	}
	else
	{
		for (int i{}; i < m_NrOfNeighbors; ++i)
		{
			vel += m_Neighbors[i]->GetLinearVelocity();
		}
		vel /= float(m_NrOfNeighbors);
	}
	return vel;
}

float Flock::GetNeighborRadius() const
{
	return m_NeighborhoodRadius;
}

float* Flock::GetWeight(ISteeringBehavior* pBehavior)
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->m_WeightedBehaviors;
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if (it != weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}