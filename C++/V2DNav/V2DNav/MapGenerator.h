
#pragma  once
#include "DataType.h"

namespace V2DNav
{

	
/*GRIDPATHFIND_API*/ NavMap* BuildNavMap(const PathBuildConfig& BuildConfig);
void GetFloorEdges(NavMap* InMap);
void MergeEdges(NavMap* InMap);
bool CanMerge(Edge* E1, Edge* E2);
void BuildLinks(NavMap* InMap);
void BuildLinksWithEdge(NavMap* InMap,Edge* E1,Edge* E2);
bool BuildWalkLinks(NavMap* InMap,Edge* EH,Edge* EL);
bool BuildCrossLinks(NavMap* InMap,Edge* EH,Edge* EL);
void GetFloorsIntersectWithAABB(NavMap* InMap, std::vector<Floor*>& OutFloors, Vector2D Min, Vector2D Max, bool bIgnoreCanCross);
void GetFloorsIntersectWithTriangle(NavMap* InMap, std::vector<Floor*>& OutFloors, const Triangle& InTri);
bool BuildJumpLinks(NavMap* InMap,Edge* EH,Edge* EL);
bool BuildSameLineJumpLinks(NavMap* InMap,Edge* EH,Edge* EL);
void AddLink(Edge* E1, Edge* E2, Link* NewLink);

int Dot(const Vector2D& Vec1,const Vector2D& Vec2);
int64_t Cross(const Vector2D& Vec1,const Vector2D& Vec2);
bool AABBIntersectWithLine(const Segment& Seg,const Box& InBox);
bool TriangleIntersectWithFloor(const Triangle& InTri, const Floor* InFloor);

bool CheckMapData(NavMap* InMap);
};