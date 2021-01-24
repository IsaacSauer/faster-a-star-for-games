#ifndef INFLUENCE_MAP_APPLICATION_H
#define INFLUENCE_MAP_APPLICATION_H
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteInterfaces/EIApp.h"
#include "../App_Steering/SteeringHelpers.h"

#include "framework\EliteAI\EliteGraphs\EliteGraphUtilities\EGraphRenderer.h"
#include "framework\EliteAI\EliteNavigation\Algorithms\EPathSmoothing.h"
#include "OptimizedGraph.h"

class NavigationColliderElement;
class SteeringAgent;
class Seek;
class Arrive;

namespace Elite
{
	class NavGraph;
}
//-----------------------------------------------------------------
// Application
//-----------------------------------------------------------------
class App_FasterAStar final : public IApp
{
public:
	//Constructor & Destructor
	App_FasterAStar() = default;
	virtual ~App_FasterAStar();

	//App Functions
	void Start() override;
	void Update(float deltaTime) override;
	void Render(float deltaTime) const override;

	void SaveBoundingBoxes(const std::string& path);
	void LoadBoundingBoxes(const std::string& path);
private:
	//Datamembers
	// --Agents--
	SteeringAgent* m_pAgent = nullptr;
	Seek* m_pSeekBehavior = nullptr;
	Arrive* m_pArriveBehavior = nullptr;
	TargetData m_Target = {};
	float m_AgentRadius = 1.0f;
	float m_AgentSpeed = 16.0f;

	// --Level--
	std::vector<NavigationColliderElement*> m_vNavigationColliders = {};
	OptimizedGraph< Elite::NavGraphNode, Elite::GraphConnection2D>* m_pOptimizedGraph;

	// --Pathfinder--
	std::vector<Elite::Vector2> m_vPath;

	// --Graph--
	Elite::NavGraph* m_pNavGraph = nullptr;
	Elite::EGraphRenderer m_GraphRenderer{};

	// --Debug drawing information--
	std::vector<Elite::Portal> m_Portals;
	std::vector<Elite::Vector2> m_DebugNodePositions;
	static bool sShowPolygon;
	static bool sShowGraph;
	static bool sDrawPortals;
	static bool sDrawFinalPath;
	static bool sDrawNonOptimisedPath;

	void UpdateImGui();
	std::vector<Elite::Vector2> FindPath(Elite::Vector2 startPos, Elite::Vector2 endPos);

	bool m_Save{ false };
	bool m_Load{ false };
private:
	//C++ make the class non-copyable
	App_FasterAStar(const App_FasterAStar&) = delete;
	App_FasterAStar& operator=(const App_FasterAStar&) = delete;
};
#endif