#include "PathFind.h"
#include <math.h>
#include <assert.h>
#include <algorithm>

namespace V2DNav
{
	class PriorityQueue
	{
	public:
		PriorityQueue(int n);
		~PriorityQueue();

		inline void Clear() { m_size = 0; }

		inline Edge* Top() { return m_heap[0]; }

		inline Edge* Pop()
		{
			Edge* result = m_heap[0];
			result->bInOpenList = false;
			m_size--;
			TrickleDown(0, m_heap[m_size]);
			return result;
		}

		inline void Push(Edge* node)
		{
			m_size++;
			BubbleUp(m_size - 1, node);
			node->bInOpenList = true;
		}

		inline void Modify(Edge* node)
		{
			for (int i = 0; i < m_size; ++i)
			{
				if (m_heap[i] == node)
				{
					BubbleUp(i, node);
					return;
				}
			}
		}

		inline bool Empty() const { return m_size == 0; }

		inline int GetMemUsed() const
		{
			return sizeof(*this) +
				sizeof(Edge*) * (m_capacity + 1);
		}

		inline int GetCapacity() const { return m_capacity; }

	private:

		void BubbleUp(int i, Edge* node);
		void TrickleDown(int i, Edge* node);

		Edge** m_heap;
		const int m_capacity;
		int m_size;
	};


	PriorityQueue::PriorityQueue(int n) :
		m_heap(NULL),
		m_capacity(n),
		m_size(0)
	{
		m_heap = (Edge**)malloc(sizeof(Edge*)*(m_capacity + 1));
	}

	void PriorityQueue::BubbleUp(int i, Edge* node)
	{
		int parent = (i - 1) / 2;
		// note: (index > 0) means there is a parent
		while ((i > 0) && (m_heap[parent]->ActualCost + m_heap[parent]->EstimatedCost > node->ActualCost + node->EstimatedCost))
		{
			m_heap[i] = m_heap[parent];
			i = parent;
			parent = (i - 1) / 2;
		}
		m_heap[i] = node;
	}

	void PriorityQueue::TrickleDown(int i, Edge* node)
	{
		int child = (i * 2) + 1;
		while (child < m_size)
		{
			if (((child + 1) < m_size) &&
				(m_heap[child]->ActualCost + m_heap[child]->EstimatedCost > m_heap[child + 1]->ActualCost + m_heap[child + 1]->EstimatedCost))
			{
				child++;
			}
			m_heap[i] = m_heap[child];
			i = child;
			child = (i * 2) + 1;
		}
		BubbleUp(i, node);
	}

	PriorityQueue::~PriorityQueue()
	{
		free(m_heap);
	}

	bool PointInBox(const Vector2D& Point, const Box& InBox)
	{
		if (Point.x >= InBox.Min.x && Point.x <= InBox.Max.x
			&& Point.y >= InBox.Min.y && Point.y <= InBox.Max.y)
		{
			return true;
		}
		return false;
	}

	Edge* FindClosestEdge(NavMap* InMap, const Vector2D& Point)
	{
		int BestScore = -1;
		Floor* BestFloor = NULL;
		Edge* BestEdge = NULL;
		size_t FloorNum = InMap->Floors.size();
		for (size_t i = 0; i < FloorNum; ++i)
		{
			Floor* pFloor = InMap->Floors[i];

			if (Point.y >= pFloor->Max.y && Point.x >= pFloor->Min.x && Point.x <= pFloor->Max.x)
			{
				if (!BestFloor || BestScore > Point.y - pFloor->Max.y)
				{
					BestScore = Point.y - pFloor->Max.y;
					BestFloor = pFloor;
				}
			}

		}
		if (!BestFloor)
		{
			return NULL;
		}

		size_t ENum = BestFloor->ValidEdges.size();
		std::vector<V2DNav::Edge*>& Edges = BestFloor->ValidEdges;

		for (size_t j = 0; j < ENum; ++j)
		{
			V2DNav::Edge* e = Edges[j];
			if (Point.x >= e->Point1.x && Point.x <= e->Point2.x)
			{
				return e;
			}
		}

		return NULL;
	}

	PathFindingHelper::~PathFindingHelper()
	{
		if (OpenList)
		{
			delete OpenList;
		}
		OpenList = NULL;
	}

	bool PathFindingHelper::Init(PathFindingConfig& InFindingConfig)
	{
		if (!InFindingConfig.Map)
		{
			return false;
		}

		if (!InFindingConfig.bIsAirPath && !InFindingConfig.CanJumpReached)
		{
			return false;
		}

		if (!OpenList)
		{
			OpenList = new class PriorityQueue(10000);
		}
		
		FindingConfig = InFindingConfig;

		int Radius = InFindingConfig.Map->Agent.AgentWidth >> 1;
		int HalfHeight = InFindingConfig.Map->Agent.AgentHeight >> 1;
		int Dir[4][2] = { {0,0}, {0,HalfHeight},{-Radius,0},{Radius,0} };

		for (int i = 0; i < 4; ++i)
		{
			Start.x = FindingConfig.Start[0] + Dir[i][0];
			Start.y = FindingConfig.Start[1] + HalfHeight + Dir[i][1];
			StartEdge = FindClosestEdge(FindingConfig.Map, Start);
			if (StartEdge)
			{
				break;
			}
		}

		if (!StartEdge)
		{
			return false;
		}

		for (int i = 0; i < 4; ++i)
		{
			End.x = FindingConfig.End[0] + Dir[i][0];
			End.y = FindingConfig.End[1] + HalfHeight + Dir[i][1];
			EndEdge = FindClosestEdge(FindingConfig.Map, End);
			if (EndEdge)
			{
				break;
			}
		}

		if (!EndEdge)
		{
			return false;
		}
		// project start and end
		Start.y = StartEdge->Point1.y;
		End.y = EndEdge->Point1.y;

		if (FindingConfig.bIsAirPath)
		{
			JumpPadding = 0;
		}
		else
		{
			JumpPadding = FindingConfig.Map->Agent.AgentWidth;
		}
		bInited = true;
		return true;
	}

	float VSize(const Vector2D& P)
	{
		return sqrt(double(P.x*P.x) + double(P.y*P.y));
	}

	void PrintSegment(FILE* f, int StartX, int StartY, int EndX, int EndY)
	{
		if (!f)
			return;
		fprintf(f,"ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
			StartX, StartY, EndX, EndY);
	}


	bool PathFindingHelper::FindPath(std::vector<WayPoint>& OutRoute)
	{
		if (!bInited)
		{
			return false;
		}
		++PathSessionID;
		OpenList->Clear();

		StartEdge->ResetPathData();
		StartEdge->EntryPoint = Start;
		StartEdge->ActualCost = 0.0f;
		StartEdge->EstimatedCost = VSize(Start - End);
		StartEdge->PathSessionID = PathSessionID;
		StartEdge->LastEdge = NULL;
		StartEdge->LastPoint = Vector2D();
		if (FindingConfig.bIsAirPath)
		{
			StartEdge->EntryBehavior = WayPoint::EPassBehavior_Fly;
		}
		else
		{
			StartEdge->EntryBehavior = WayPoint::EPassBehavior_Walk;
		}
		OpenList->Push(StartEdge);


		FILE* f = NULL;
		if (fopen_s(&f, "NavDebugData.pathfind", "w") != 0)
			f = NULL;

		Edge* LastEdge = NULL;
		while (!OpenList->Empty())
		{
			Edge* CurrentEdge = OpenList->Pop();

			PrintSegment(f, CurrentEdge->Point1.x, CurrentEdge->Point1.y, CurrentEdge->Point2.x, CurrentEdge->Point2.y);

			// check end condition
			if (CurrentEdge == EndEdge)
			{
				LastEdge = EndEdge;
				break;
			}
			GetAdjacentEdges(CurrentEdge);
		}

		if (f)
			fclose(f);

		if (LastEdge)
		{
			bool bIsAirPath = FindingConfig.bIsAirPath;
			OutRoute.clear();

			WayPoint Dest(End.x,End.y,bIsAirPath ? WayPoint::EPassBehavior_Fly :WayPoint::EPassBehavior_Walk);
			OutRoute.push_back(Dest);

			while (LastEdge)
			{
				OutRoute.push_back(WayPoint(LastEdge->EntryPoint.x, LastEdge->EntryPoint.y, bIsAirPath ? WayPoint::EPassBehavior_Fly : LastEdge->EntryBehavior));
				if (LastEdge != StartEdge)
				{
					OutRoute.push_back(WayPoint(LastEdge->LastPoint.x, LastEdge->LastPoint.y, bIsAirPath ? WayPoint::EPassBehavior_Fly : WayPoint::EPassBehavior_Walk));
				}
				LastEdge = LastEdge->LastEdge;
			}

			std::reverse(OutRoute.begin(), OutRoute.end());

			int AdjustHeight = 0;//FindingConfig.Map->Agent.AgentHeight >> 2;
			for (std::vector<WayPoint>::iterator Itr = OutRoute.begin();
				Itr != OutRoute.end(); ++Itr)
			{
				(*Itr).Point[1] -= AdjustHeight;
			}
			return true;
		}
		return false;
	}

	void PathFindingHelper::GetAdjacentEdges(Edge* InE)
	{
		for (Edge::AdjacentEdgeMap::iterator Itr = InE->AdjacentEdges.begin();
			Itr != InE->AdjacentEdges.end(); ++Itr)
		{
			Edge* NextE = Itr->first;
			std::vector<Link*>* Links = Itr->second;

			size_t LinkNum = Links->size();
			for (size_t i = 0; i < LinkNum; ++i)
			{
				Link* link = (*Links)[i];
				PassByLink(InE, NextE, link);
			}
		}
	}


	void PathFindingHelper::PassByLink(Edge* InE, Edge* OutE, const Link* Inlink)
	{
		switch (Inlink->LinkType)
		{
		case ELinkType::ELinkType_Cross:
			PassByCrossLink(InE,OutE,static_cast<const CrossLink*>(Inlink));
			break;
		case ELinkType::ELinkType_Walk:
			PassByWalkLink(InE,OutE,static_cast<const WalkLink*>(Inlink));
			break;
		case ELinkType::ELinkType_Jump:
			PassByJumpLink(InE,OutE,static_cast<const JumpLink*>(Inlink));
			break;
		default:
			break;
		}
	}

	void PathFindingHelper::PassByCrossLink(Edge* InE, Edge* OutE, const CrossLink* Inlink)
	{
		bool bCanJumpUp,bCanCrossDown;
		bCanJumpUp = bCanCrossDown = false;
		if (InE->Point1.y >= OutE->Point1.y)
		{
			// jump down
			bCanCrossDown = true;
			assert(InE->Owner->bCanCross);
		}
		else if (CanJumpUpTo(InE->EntryPoint,Vector2D(InE->EntryPoint.x,OutE->Point1.y)))
		{
			// jump up
			bCanJumpUp = true;
			assert(OutE->Owner->bCanCross);
		}
		if (bCanJumpUp || bCanCrossDown)
		{
			float CostModifier = bCanCrossDown ? 9.0f : 1.0f;
			// can jump to out edge
			if (InE->EntryPoint.x >= Inlink->LeftStart.x
				&& InE->EntryPoint.x <= Inlink->RightStart.x)
			{
				float ActualCost = InE->ActualCost + abs(InE->Point1.y - OutE->Point1.y) * CostModifier ;

				AddSuccessor(OutE, InE, ActualCost,
					Vector2D(InE->EntryPoint.x, OutE->Point1.y),
					bCanJumpUp ? WayPoint::EPassBehavior_JumpUp : WayPoint::EPassBehavior_CrossDown,
					InE->EntryPoint);
			}
			else if (InE->EntryPoint.x < Inlink->LeftStart.x)
			{
				// walk to left start and jump
				Vector2D Launch = Inlink->LeftStart;
				Launch.y = InE->Point1.y;
				Launch.x = std::min(Launch.x + JumpPadding,Inlink->RightStart.x);

				float ActualCost = InE->ActualCost + ((Launch.x - InE->EntryPoint.x) + abs(InE->Point1.y - OutE->Point1.y)) * CostModifier;

				AddSuccessor(OutE, InE, ActualCost,
					Vector2D(Launch.x, OutE->Point1.y),
					bCanJumpUp ? WayPoint::EPassBehavior_JumpUp : WayPoint::EPassBehavior_CrossDown,
					Launch);
			}
			else if (InE->EntryPoint.x > Inlink->RightStart.x)
			{
				// walk to right start and jump
				Vector2D Launch = Inlink->LeftStart;
				Launch.y = InE->Point1.y;
				Launch.x = std::max(Launch.x - JumpPadding,Inlink->LeftStart.x);

				float ActualCost = InE->ActualCost + ((InE->EntryPoint.x - Launch.x) + abs(InE->Point1.y - OutE->Point1.y)) * CostModifier;

				AddSuccessor(OutE, InE, ActualCost,
					Vector2D(Inlink->RightStart.x, OutE->Point1.y),
					bCanJumpUp ? WayPoint::EPassBehavior_JumpUp : WayPoint::EPassBehavior_JumpDown,
					Launch);
			}
		}
	}

	bool IsPointOnEdge(const Vector2D& Point, const Edge* InE)
	{
		if (Point.x >= InE->Point1.x && Point.x <= InE->Point2.x)
		{
			if (Point.y == InE->Point1.y)
			{
				return true;
			}
		}
		return false;
	}


	void PathFindingHelper::PassByWalkLink(Edge* InE, Edge* OutE, const WalkLink* Inlink)
	{
		if (IsPointOnEdge(Inlink->Left,InE))
		{
			assert(IsPointOnEdge(Inlink->Right,OutE));
			float ActualCost = InE->ActualCost + abs(InE->EntryPoint.x - Inlink->Right.x);

			AddSuccessor(OutE, InE, ActualCost,
				Inlink->Right,
				WayPoint::EPassBehavior_Walk,
				Inlink->Left);
		}
		else if (IsPointOnEdge(Inlink->Right,InE))
		{
			assert(IsPointOnEdge(Inlink->Left,OutE));
			float ActualCost = InE->ActualCost + abs(InE->EntryPoint.x - Inlink->Left.x);

			AddSuccessor(OutE, InE, ActualCost,
				Inlink->Left,
				WayPoint::EPassBehavior_Walk,
				Inlink->Right);
		}
	}
	bool PathFindingHelper::CanJumpUpTo(const Vector2D& InStart, const Vector2D& InEnd)
	{
		if (FindingConfig.bIsAirPath)
		{
			return true;
		}
		float ApexX, ApexY;
		return (*FindingConfig.CanJumpReached)(FindingConfig.Owner,ApexX,ApexY,InStart.x,InStart.y,InEnd.x,InEnd.y);
	}

	bool PathFindingHelper::CanJumpDownTo(const Vector2D& InStart, const Vector2D& InEnd)
	{
		if (abs(InStart.x - InEnd.x) < FindingConfig.MaxJumpLength)
		{
			return true;
		}
		return false;
	}

	void PathFindingHelper::PassByJumpLink(Edge* InE, Edge* OutE, const JumpLink* Inlink)
	{
		if (InE->Point1.y <= OutE->Point1.y)
		{
			// jump up
			assert(IsPointOnEdge(Inlink->Left,InE));
			assert(IsPointOnEdge(Inlink->Right,InE));
			assert(IsPointOnEdge(Inlink->End,OutE));

			Vector2D LaunchPoint,SecondLanchPoint;
			Vector2D LandPoint;

			if (Inlink->End.x <= Inlink->Left.x)
			{
				// jump up towards left.
				LaunchPoint = Inlink->Left;
				LaunchPoint.x = std::min(LaunchPoint.x + JumpPadding,Inlink->Right.x);

				SecondLanchPoint = LaunchPoint;
				SecondLanchPoint.x = std::min(LaunchPoint.x + JumpPadding, Inlink->Right.x);

				LandPoint = Inlink->End;
				LandPoint.x = std::max(LandPoint.x - JumpPadding,OutE->Point1.x);
			}
			else if (Inlink->End.x >= Inlink->Right.x)
			{
				// jump up towards right
				LaunchPoint = Inlink->Right;
				LaunchPoint.x = std::max(LaunchPoint.x - JumpPadding,Inlink->Left.x);

				SecondLanchPoint = LaunchPoint;
				SecondLanchPoint.x = std::max(LaunchPoint.x - JumpPadding, Inlink->Left.x);

				LandPoint = Inlink->End;
				LandPoint.x = std::min(LandPoint.x + JumpPadding,OutE->Point2.x);
			}
			else
			{
				assert(0);
			}
			bool bCanJump = false;
			// try a beautiful jump first.
			if (CanJumpUpTo(SecondLanchPoint, LandPoint))
			{
				LaunchPoint = SecondLanchPoint;
				bCanJump = true;
			}
			else if (CanJumpUpTo(LaunchPoint, LandPoint))
			{
				bCanJump = true;
			}
			if (bCanJump)
			{
				float ActualCost = InE->ActualCost + abs(InE->EntryPoint.x - LaunchPoint.x) + VSize(LaunchPoint - LandPoint);
				AddSuccessor(OutE, InE, ActualCost,
					LandPoint,
					WayPoint::EPassBehavior_JumpUp,
					LaunchPoint);
			}
		}
		else
		{
			// jump down
			assert(IsPointOnEdge(Inlink->Left,OutE));
			assert(IsPointOnEdge(Inlink->Right,OutE));
			assert(IsPointOnEdge(Inlink->End,InE));
			Vector2D LandPoint;
			Vector2D LaunchPoint;

			int BakJumpPadding = JumpPadding;
			JumpPadding = 0;
			if (Inlink->End.x <= Inlink->Left.x)
			{
				// jump down towards right
				LaunchPoint = Inlink->End;
				LaunchPoint.x = std::max(LaunchPoint.x - JumpPadding,InE->Point1.x);

				LandPoint = Inlink->Left;
				LandPoint.x = std::min(LandPoint.x + JumpPadding, Inlink->Right.x);
			}
			else if (Inlink->End.x >= Inlink->Right.x)
			{
				// jump down towards left 
				LaunchPoint = Inlink->End;
				LaunchPoint.x = std::min(LaunchPoint.x + JumpPadding,InE->Point2.x);

				LandPoint = Inlink->Right;
				LandPoint.x = std::max(LandPoint.x - JumpPadding,Inlink->Left.x);
			}
			else
			{
				assert(0);
			}
			JumpPadding = BakJumpPadding;
			if (CanJumpDownTo(LaunchPoint,LandPoint))
			{
				float ActualCost = InE->ActualCost + abs(InE->EntryPoint.x - LaunchPoint.x) + VSize(LaunchPoint - LandPoint);

				AddSuccessor(OutE, InE, ActualCost,
					LandPoint,
					WayPoint::EPassBehavior_JumpDown,
					LaunchPoint);
			}
		}
	}

	void PathFindingHelper::AddSuccessor(
		Edge* ToE,
		Edge* FromE, 
		float ActualCost,
		const Vector2D& Entry,
		WayPoint::EPassBehavior Behavior, const Vector2D& OutPoint)
	{
		if (ToE->PathSessionID != PathSessionID)
		{
			ToE->PathSessionID = PathSessionID;
			ToE->ActualCost = -1;
			// important flag
			ToE->bInOpenList = false;
		}

		if (ToE->ActualCost < 0 || ActualCost < ToE->ActualCost)
		{
			// update
			ToE->EntryPoint = Entry;
			ToE->EntryBehavior = Behavior;

			ToE->ActualCost = ActualCost;
			ToE->EstimatedCost = VSize(ToE->EntryPoint - End);

			ToE->LastPoint = OutPoint;
			ToE->LastEdge = FromE;

			if (ToE->bInOpenList)
			{
				OpenList->Modify(ToE);
			}
			else
			{
				OpenList->Push(ToE);
			}
		}
	}

};
