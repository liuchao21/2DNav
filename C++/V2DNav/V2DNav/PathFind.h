#pragma once
#include "DataType.h"

namespace V2DNav
{
class /*GRIDPATHFIND_API*/ PathFindingHelper
{
public:

	PathFindingHelper() :bInited(false), PathSessionID(0), OpenList(NULL), StartEdge(NULL), EndEdge(NULL) {}
	~PathFindingHelper();
	bool Init(PathFindingConfig& InFindingConfig);
	bool FindPath(std::vector<WayPoint>& OutRoute);
	void GetAdjacentEdges(Edge* InE);
	void PassByLink(Edge* InE,Edge* OutE,const Link* Inlink);
	void PassByCrossLink(Edge* InE,Edge* OutE,const CrossLink* Inlink);
	void PassByWalkLink(Edge* InE,Edge* OutE,const WalkLink* Inlink);
	void PassByJumpLink(Edge* InE,Edge* OutE,const JumpLink* Inlink);

	void AddSuccessor(Edge* ToE, Edge* FromE, float ActualCost, const Vector2D& Entry, WayPoint::EPassBehavior Behavior, const Vector2D& OutPoint);
	bool CanJumpUpTo(const Vector2D& InStart, const Vector2D& InEnd);
	bool CanJumpDownTo(const Vector2D& InStart, const Vector2D& InEnd);

	bool bInited;
	int PathSessionID;
	PathFindingConfig FindingConfig;
	class PriorityQueue* OpenList;

	int JumpPadding;
	Edge* StartEdge, *EndEdge;
	Vector2D Start, End;
};

};
