#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\App_Steering\SteeringAgent.h"
#include <algorithm>

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

void Cell::ResetCell()
{
	agents.clear();
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
	, m_CellWidth{width / cols}
	, m_CellHeight{height / rows}
{	 
	for (int i{}; i < rows * cols; ++i)
	{
		m_Cells.push_back(Cell((i % rows) * m_CellWidth, (i / rows) * m_CellHeight, m_CellWidth, m_CellHeight));
	}

	m_Neighbors.resize(maxEntities - 1);
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, const Elite::Vector2& oldPos)
{
	Elite::Vector2 newPos = agent->GetPosition();
	int newIdx = PositionToIndex(newPos);
	int oldIdx = PositionToIndex(oldPos);
	if (newIdx != oldIdx)
	{
		m_Cells[oldIdx].agents.remove(agent);
		m_Cells[newIdx].agents.push_back(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* pAgent, float queryRadius)
{
	Elite::Vector2 pos{ pAgent->GetPosition() };
	//Elite::Vector2 bottomLeft{ pos.x - queryRadius,pos.y - queryRadius };
	//Elite::Vector2 topRight{ pos.x + queryRadius, pos.y + queryRadius};
	////Elite::Vector2 bottomLeft{Elite::Clamp(pos.x - queryRadius, 0.f, m_SpaceWidth), Elite::Clamp(pos.y - queryRadius, 0.f, m_SpaceHeight) };
	////Elite::Vector2 topRight{ Elite::Clamp(pos.x + queryRadius, 0.f, m_SpaceWidth), Elite::Clamp(pos.y + queryRadius, 0.f, m_SpaceHeight) };
	//Elite::Vector2 topLeft{ bottomLeft.x, topRight.y };
	//Elite::Vector2 bottomRight{ topRight.x, bottomLeft.y };
	//Elite::Rect square{ bottomLeft, queryRadius * 2, queryRadius * 2 };

	int bottomRow{ int((pos.y - queryRadius) / m_CellHeight) - 1};
	int topRow{ int((pos.y + queryRadius) / m_CellHeight) + 1};
	int leftCol{ int((pos.x - queryRadius) / m_CellWidth) - 1};
	int rightCol{ int((pos.x + queryRadius) / m_CellWidth) + 1};

	m_NrOfNeighbors = 0;
	for (int r{ bottomRow }; r <= topRow; ++r)
	{
		for (int c{ leftCol }; c <= rightCol; ++c)
		{
			int idx{ (((r + 1) % m_NrOfRows) * m_NrOfCols + ((c + 1) % m_NrOfCols)) };
			if (idx < 0)
				idx = (m_NrOfCols * m_NrOfRows) + idx;

			for (auto& agent : m_Cells[idx].agents)
			{
				if (pAgent != agent && (agent->GetPosition() - pAgent->GetPosition()).Magnitude() < queryRadius)
				{
					m_Neighbors[m_NrOfNeighbors] = agent;
					++m_NrOfNeighbors;
				}
			}
		}
	}
	//int idxLX{ PositionToIndex(bottomLeft) };
	//int idxRX{ PositionToIndex(bottomRight)};
	//int idxBY{ PositionToIndex(bottomLeft)};
	//int idxTY{ PositionToIndex(topLeft)};
	//m_NrOfNeighbors = 0;
	//for (auto cell : m_Cells)
	//{
	//	if (Elite::IsOverlapping(cell.boundingBox, square))
	//	{
	//		for (auto agent : cell.agents)
	//		{
	//			if (pAgent != agent && (agent->GetPosition() - pos).Magnitude() < queryRadius)
	//			{
	//				m_Neighbors[m_NrOfNeighbors] = agent;
	//				++m_NrOfNeighbors;
	//			}
	//		}
	//	}
	//}
}

void CellSpace::RenderCells() const
{
	for (auto& cell : m_Cells)
	{
		if (cell.agents.size() > 0)
		{
			Elite::Polygon poly(cell.GetRectPoints());
			DEBUGRENDERER2D->DrawPolygon(&poly, Elite::Color(1, 0, 0));
			DEBUGRENDERER2D->DrawString(poly.GetCenterPoint(), std::to_string(cell.agents.size()).c_str());
		}
	}
}

void CellSpace::RenderAgent(SteeringAgent* pAgent, float dTime)
{
	DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), 10.f, Elite::Color(0, 1, 0), 0.8f);
	RegisterNeighbors(pAgent, 10.f);
	for (int i{}; i < m_NrOfNeighbors; ++i)
	{
		DEBUGRENDERER2D->DrawPoint(pAgent->GetPosition(), pAgent->GetRadius(), Elite::Color(0, 0, 1));
	}
}

void CellSpace::ResetCells()
{
	for (auto& cell : m_Cells)
	{
		cell.ResetCell();
	}
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	int row{ int(pos.y / m_CellHeight) % m_NrOfRows };
	int col{ int(pos.x / m_CellWidth) % m_NrOfCols };
	if (row < 0)
		row = m_NrOfRows + row;
	if (col < 0)
		col = m_NrOfCols + col;

	return row * m_NrOfCols + col;
}