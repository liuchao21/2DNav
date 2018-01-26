
#pragma once
#include <vector>
#include <map>

namespace V2DNav
{

struct SFloor
{
    int Min[2];
    int Max[2];
    bool bCanCross;
};

struct AgentConfig
{
    int AgentHeight;
    int AgentWidth;
    int StepHeight;
};

struct PathBuildConfig
{
    AgentConfig Agent;
    int FloorNum;
    SFloor* Floors;
	PathBuildConfig() :Floors(NULL), FloorNum(0) {}
};

struct PathFindingConfig
{
	typedef bool(*TFuncCanJumpUpReached)(const void* Owner, float &ApexY,float& ApexZ,float StartX, float StartY, float EndX, float EndY);

	int Start[2];
	int End[2];
	int MaxJumpLength;
	int MaxJumpHeight;
	bool bIsAirPath;

	class NavMap* Map;
	const void* Owner;
	TFuncCanJumpUpReached CanJumpReached;
	PathFindingConfig()
	{
		Start[0] = Start[1] = 0;
		End[0] = End[1] = 0;
		MaxJumpLength = 0;
		MaxJumpHeight = 0;
		bIsAirPath = false;
		Map = NULL;
		Owner = NULL;
		CanJumpReached = NULL;
	}
};

struct WayPoint
{
	enum EPassBehavior
	{
		EPassBehavior_Walk,
		EPassBehavior_JumpUp,
		EPassBehavior_JumpDown,
		EPassBehavior_CrossDown,
		EPassBehavior_NavLink,
		EPassBehavior_Fly
	};
	int Point[2];
	EPassBehavior Behavior;
	WayPoint() { Point[0] = Point[1] = 0; Behavior = EPassBehavior_Walk; }
	WayPoint(int x, int y, EPassBehavior InBehavior) { Point[0] = x;Point[1] = y; Behavior = InBehavior; }
};

class Vector2D
{
public:
    Vector2D():x(0),y(0){}
    Vector2D(int InX,int InY):x(InX),y(InY){}
    Vector2D(const Vector2D& InVec):x(InVec.x),y(InVec.y) {}

    Vector2D& operator= (const Vector2D& InVec)
    {
        x = InVec.x;
        y = InVec.y;
		return *this;
    }

    Vector2D operator- (Vector2D& InVec) {return Vector2D(x - InVec.x,y - InVec.y);}
    Vector2D operator- (const Vector2D& InVec) const {return Vector2D(x - InVec.x,y - InVec.y);}
	bool operator == (const Vector2D& InVec) { return x == InVec.x && y == InVec.y; }

    int x,y;
};
enum ELinkType
{
	ELinkType_None,
	ELinkType_Walk,
	ELinkType_Cross,
	ELinkType_Jump
};
class Link
{
	
public:
    Link():bBidirection(true) {}
    bool bBidirection;
	ELinkType LinkType;
    virtual ~Link(){}
};

class JumpLink :public Link
{
public:
	JumpLink() { LinkType = ELinkType::ELinkType_Jump; }
    Vector2D Left,Right;
    Vector2D End;
};

class CrossLink : public Link
{
public:
	CrossLink() { LinkType = ELinkType_Cross; }
    Vector2D LeftStart,LeftEnd;
    Vector2D RightStart,RightEnd;
};

class WalkLink : public Link
{
public:
	WalkLink() { LinkType = ELinkType_Walk; }
    Vector2D Left,Right;
};


class Edge
{
public:
	
    Edge():Owner(NULL),ActualCost(0.0f),EstimatedCost(0.0f),PathSessionID(0) {}
    Edge(class Floor* InOwner,const Vector2D& InPoint1,const Vector2D& InPoint2):Owner(InOwner),Point1(InPoint1),Point2(InPoint2){}

	void ResetPathData() 
	{
		ActualCost = -1.0f;
		EstimatedCost = -1.0f;
		PathSessionID = 0;
		bInOpenList = false;
		EntryBehavior = WayPoint::EPassBehavior_Walk;
		LastPoint = Vector2D(0, 0);
		EntryPoint = Vector2D(0, 0);
		LastEdge = NULL;
	}

    Vector2D Point1,Point2;
    class Floor* Owner;
	typedef std::map<Edge*, std::vector<Link*>* > AdjacentEdgeMap;
    AdjacentEdgeMap AdjacentEdges;


    // temparary data for path finding.
	float ActualCost;
    float EstimatedCost;
	int PathSessionID;
	bool bInOpenList;

	WayPoint::EPassBehavior EntryBehavior;
	Vector2D EntryPoint, LastPoint;

	Edge* LastEdge;
};

class Segment
{
public:
    Segment(){}
    Segment(const Vector2D& InPoint1,const Vector2D& InPoint2):Point1(InPoint1),Point2(InPoint2){}
    Segment(const Segment& InSeg):Point1(InSeg.Point1),Point2(InSeg.Point2){}

	Segment& operator= (const Segment& InSeg)
	{
		Point1 = InSeg.Point1;
		Point2 = InSeg.Point2;
		return *this;
	}

    Vector2D Point1,Point2;
};

class Box
{
public:
    Box(){}
    Box(const Vector2D& InMin,const Vector2D& InMax):Min(InMin),Max(InMax) {}
    Box(const Box& InBox):Min(InBox.Min),Max(InBox.Max){}
    Vector2D Min,Max;
};

class Triangle 
{
public:
    Triangle(){}
    Triangle(const Vector2D& InAcme,const Vector2D& InLeft,const Vector2D& InRight):Acme(InAcme),Left(InLeft),Right(InRight){}
    Triangle(const Triangle& InTriangle):Acme(InTriangle.Acme),Left(InTriangle.Left),Right(InTriangle.Right){}

    Triangle& operator= (const Triangle& InTriangle)
    {
        Acme = InTriangle.Acme;
        Left = InTriangle.Left;
        Right = InTriangle.Right; 
        return *this;
    }
	bool operator == (const Triangle& InTriangle)
	{
		return (Acme == InTriangle.Acme) && (Left == InTriangle.Left) && (Right == InTriangle.Right);
	}
	bool operator != (const Triangle& InTriangle)
	{
		return !(*this == InTriangle);
	}
    Vector2D Acme,Left,Right;
};

class Floor
{
public:
    bool bCanCross;
    Vector2D Min,Max;
    std::vector<Edge*> ValidEdges;
};

class NavMap
{
public:

    AgentConfig Agent;
    // quad tree for intersect test.
    std::vector<Floor*> Floors;
	~NavMap();
private:
};

};
