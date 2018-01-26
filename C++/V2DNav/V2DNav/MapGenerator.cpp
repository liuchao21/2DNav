#include "MapGenerator.h"
#include <List>
#include <algorithm>
#include <set>
#include "assert.h"


namespace V2DNav
{
#ifndef DEBUG_NAV
#define NAV_ASSERT(c) if (!(c)) {return false;}
#else
#define NAV_ASSERT(c) assert(c);
#endif // NAV_ASSERT


NavMap* BuildNavMap(const PathBuildConfig& BuildConfig)
{
    // 0,check data
    if (BuildConfig.Agent.AgentHeight < 0
        || BuildConfig.Agent.AgentWidth < 0
        || BuildConfig.Agent.StepHeight < 0
        || BuildConfig.FloorNum <= 0
        || BuildConfig.Floors == NULL
        )
    {
        return NULL;
    }
    //TODO:: Check bounds
    // 1, covert to internal Floor
    NavMap* NewNavMap = new NavMap;
    NewNavMap->Agent = BuildConfig.Agent;
    int HalfHeight = BuildConfig.Agent.AgentHeight >> 1;
    int Radius = BuildConfig.Agent.AgentWidth >> 1;
    for (int i = 0; i < BuildConfig.FloorNum; ++i)
    {
        const SFloor* InFloor = &(BuildConfig.Floors[i]);
        Floor* NewFloor = new Floor;
        NewFloor->Min.x = InFloor->Min[0] - Radius;
        NewFloor->Min.y = InFloor->Min[1] - HalfHeight;
        NewFloor->Max.x = InFloor->Max[0] + Radius;
        NewFloor->Max.y = InFloor->Max[1] + HalfHeight;
        NewFloor->bCanCross = InFloor->bCanCross;

        NewNavMap->Floors.push_back(NewFloor);
    }

    if (NewNavMap)
    {
        // 2,
        GetFloorEdges(NewNavMap);
        // 3,
        BuildLinks(NewNavMap);
    }
	CheckMapData(NewNavMap);
    return NewNavMap;
}

bool SegmentIntersectWithFloor(const Segment* InSeg,const Floor* InFloor)
{
	return AABBIntersectWithLine(*InSeg, Box(InFloor->Min, InFloor->Max));
}

void GetFloorEdges(NavMap* InMap)
{
    size_t FloorNum = InMap->Floors.size();
    std::vector<Floor*>& Floors = InMap->Floors;
    int HalfHeight = InMap->Agent.AgentHeight >> 1;
    int Radius = InMap->Agent.AgentWidth >> 1;
    for (size_t i = 0; i < FloorNum; ++i)
    {
        //Segment NewSegment(Vector2D(Floors[i]->Min.x + Radius, Floors[i]->Max.y - HalfHeight),
        //                   Vector2D(Floors[i]->Max.x - Radius ,Floors[i]->Max.y - HalfHeight));
        Segment NewSegment(Vector2D(Floors[i]->Min.x, Floors[i]->Max.y),
                            Vector2D(Floors[i]->Max.x,Floors[i]->Max.y));

        std::list<const Floor*> TouchedFloors;
        for (size_t j = 0; j < FloorNum; ++j)
        {
            if (i == j || Floors[j]->bCanCross)
                continue;
            if (SegmentIntersectWithFloor(&NewSegment,Floors[j]))
            {
                TouchedFloors.push_back(Floors[j]);
            }
        }

        std::list<Segment> SplitedSegments;
        SplitedSegments.push_back(NewSegment);
        int FloorY = Floors[i]->Max.y;
        std::list<Segment>::iterator SegItr = SplitedSegments.begin();
        while (SegItr != SplitedSegments.end())
        {
            bool bSplited = false;
			bool bCovered = false;
            NewSegment = *SegItr;

            for (std::list<const Floor *>::iterator it = TouchedFloors.begin();
                 it != TouchedFloors.end(); ++it)
            {
                const Floor *TouchedFloor = *it;
                if (FloorY >= TouchedFloor->Min.y && FloorY < TouchedFloor->Max.y)
                {
					if (NewSegment.Point1.x >= TouchedFloor->Min.x
						&& NewSegment.Point2.x <= TouchedFloor->Max.x)
					{
						bCovered = true;
						break;
					}

					// left
                    if (TouchedFloor->Min.x < NewSegment.Point2.x)
                    {
                        if (NewSegment.Point1.x < TouchedFloor->Min.x)
                        {
                            Segment LeftSegment = NewSegment;
                            LeftSegment.Point2.x = TouchedFloor->Min.x;
                            SplitedSegments.push_back(LeftSegment);
                            bSplited = true;
                        }
                    }
					// right

                    if (TouchedFloor->Max.x < NewSegment.Point2.x)
                    {
                        if (NewSegment.Point1.x < TouchedFloor->Max.x)
                        {
                            Segment RightSegment = NewSegment;
                            RightSegment.Point1.x = TouchedFloor->Max.x;
                            SplitedSegments.push_back(RightSegment);
                            bSplited = true;
                        }
                    }
                    if (bCovered || bSplited)
                    {
                        break;
                    }
                }
            }
            if (bCovered || bSplited)
            {
                SegItr = SplitedSegments.erase(SplitedSegments.begin());
            }
            else
            {
                ++SegItr;
            }
        }

        if (!SplitedSegments.empty())
        {
            for (SegItr = SplitedSegments.begin();
                    SegItr != SplitedSegments.end(); ++SegItr)
            {
                Edge* NewEdge = new Edge(Floors[i],(*SegItr).Point1,(*SegItr).Point2);
                Floors[i]->ValidEdges.push_back(NewEdge);
            }
        }
    }
	MergeEdges(InMap);
}


void MergeEdges(NavMap* InMap)
{
    std::vector<Floor*>& Floors = InMap->Floors;

    size_t FloorNum = Floors.size();
	for (size_t i = 0; i < FloorNum; ++i)
	{
		for (size_t j = 0; j < i; ++j)
		{
			std::vector<Edge*>& EdgesI = Floors[i]->ValidEdges;
			std::vector<Edge*>& EdgesJ = Floors[j]->ValidEdges;

			while (true)
			{
				bool bMerged = false;
				for (std::vector<Edge*>::iterator I_Iter = EdgesI.begin();
					I_Iter != EdgesI.end(); ++I_Iter)
				{
					for (std::vector<Edge*>::iterator J_Iter = EdgesJ.begin();
						J_Iter != EdgesJ.end(); ++J_Iter)
					{
						if (CanMerge(*I_Iter, *J_Iter))
						{
							Edge* NewEdge = new Edge;
							NewEdge->Point1.y = (*I_Iter)->Point1.y;
							NewEdge->Point2.y = NewEdge->Point1.y;
							NewEdge->Point1.x = std::min((*I_Iter)->Point1.x, (*J_Iter)->Point1.x);
							NewEdge->Point2.x = std::max((*I_Iter)->Point2.x, (*J_Iter)->Point2.x);
							NewEdge->Owner = (*I_Iter)->Owner;
							EdgesI.erase(I_Iter);
							EdgesJ.erase(J_Iter);
							EdgesI.push_back(NewEdge);
							//EdgesJ.push_back(NewEdge);
							bMerged = true;
							break;
						}
					}
					if (bMerged)
					{
						break;
					}
				}
				if (!bMerged)
				{
					break;
				}
			}
		}
	}
}

bool CanMerge(Edge* E1, Edge* E2)
{
	if (E1 == E2)
	{
		return false;
	}
	if (E1->Point1.y != E2->Point1.y)
	{
		return false;
	}

	if (E1->Point2.x < E2->Point1.x
		|| E2->Point2.x < E1->Point1.x)
	{
		return false;
	}

	if (E1->Owner->bCanCross != E2->Owner->bCanCross)
	{
		return false;
	}

	return true;
}

void BuildLinks(NavMap* InMap)
{
    std::vector<Floor*>& Floors = InMap->Floors;

    size_t FloorNum = Floors.size();
    for (size_t i = 0; i < FloorNum; ++i)
    {
        for (size_t j = 0; j < i; ++j)
        {
            std::vector<Edge*>& EdgesI = Floors[i]->ValidEdges;
            std::vector<Edge*>& EdgesJ = Floors[j]->ValidEdges;

            for (std::vector<Edge*>::iterator I_Iter = EdgesI.begin();
                 I_Iter != EdgesI.end(); ++I_Iter)
            {
                for (std::vector<Edge*>::iterator J_Iter = EdgesJ.begin();
                    J_Iter != EdgesJ.end(); ++J_Iter)
                {
					if (*I_Iter != *J_Iter)
					{
						BuildLinksWithEdge(InMap,*I_Iter,*J_Iter);
					}
                }
            }
        }
    }
}

void BuildLinksWithEdge(NavMap* InMap,Edge* InE1,Edge* InE2)
{
    Edge* EH,*EL;
    if (InE1->Point1.y < InE2->Point2.y)
    {
        EH = InE2;
        EL = InE1;
    }
    else
    {
        EH = InE1;
        EL = InE2;
    }

    // build cross link
    BuildCrossLinks(InMap,EH,EL);

    // build walk link
    if (!BuildWalkLinks(InMap,EH,EL))
    {
        // build jump link
        BuildJumpLinks(InMap,EH,EL);
    }
}

bool BuildWalkLinks(NavMap* InMap,Edge* EH,Edge* EL)
{
    bool bHasLinks = false;
    if (EH->Point1.y - EL->Point1.y <= InMap->Agent.StepHeight)
    {
        // left
        if (EL->Point1.x < EH->Point1.x)
        {
            if (EL->Point2.x + InMap->Agent.AgentWidth > EH->Point1.x)
            {
                WalkLink* NewWalkLink = new WalkLink;
                NewWalkLink->Right = EH->Point1;
                NewWalkLink->Left = EL->Point2;
                if (NewWalkLink->Left.x > EH->Point1.x)
                {
                    NewWalkLink->Left.x = EH->Point1.x;
                }
                bHasLinks = true;
				AddLink(EH, EL, NewWalkLink);
            }
        }

        // right
        if (EH->Point2.x < EL->Point2.x)
        {
            if (EL->Point1.x - InMap->Agent.AgentWidth < EH->Point2.x)
            {
                WalkLink* NewWalkLink = new WalkLink;
                NewWalkLink->Left = EH->Point2;
                NewWalkLink->Right = EL->Point1;
                if (EL->Point1.x < EH->Point2.x)
                {
                    NewWalkLink->Right.x = EH->Point2.x;
                }
                bHasLinks = true;
				AddLink(EH, EL, NewWalkLink);
            }
        }
    }

    return bHasLinks;
}

bool BuildCrossLinks(NavMap* InMap,Edge* EH,Edge* EL)
{
    bool bHasLinks = false;
    if (EH->Owner->bCanCross)
    {
        if (EL->Point2.x <= EH->Point1.x
            || EH->Point2.x <= EL->Point1.x)
        {
            return false;
        }
		// lower edge should not be in cross floor.
		if (EL->Point1.y >= EH->Owner->Min.y)
		{
			return false;
		}

        Segment LeftSegment,RightSegment;
        LeftSegment.Point1.y = EL->Point1.y;
        LeftSegment.Point2.y = EH->Point1.y;
        RightSegment.Point1.y = EL->Point1.y;
        RightSegment.Point2.y = EH->Point1.y;

        if (EL->Point1.x < EH->Point1.x)
        {
            LeftSegment.Point1.x = EH->Point1.x;
            LeftSegment.Point2.x = EH->Point1.x;
        }
        else 
        {
            LeftSegment.Point1.x = EL->Point1.x;
            LeftSegment.Point2.x = EL->Point1.x;
        }

        if (EH->Point2.x < EL->Point2.x)
        {
            RightSegment.Point1.x = EH->Point2.x;
            RightSegment.Point2.x = EH->Point2.x;
        }
        else
        {
            RightSegment.Point1.x = EL->Point2.x;
            RightSegment.Point2.x = EL->Point2.x;
        }

        Vector2D Min(LeftSegment.Point1);
        Vector2D Max(RightSegment.Point2);
        std::vector<Floor*> Floors;
        GetFloorsIntersectWithAABB(InMap,Floors,Min,Max,false);
        std::list<Box> BoxList;
        BoxList.push_back(Box(Min,Max));
        std::list<Box>::iterator Itr = BoxList.begin();

        while (Itr != BoxList.end())
        {
            Box NewBox(*Itr);

            bool bSplited = false;
            bool bCover = false;
            size_t FloorNum = Floors.size();
            for (size_t i = 0; i < FloorNum; ++i)
            {
				if (Floors[i] == EH->Owner || Floors[i] == EL->Owner)
				{
					continue;
				}
                if (Floors[i]->Max.x <= NewBox.Min.x || NewBox.Max.x <= Floors[i]->Min.x || Floors[i]->Max.y <= NewBox.Min.y || NewBox.Max.y <= Floors[i]->Min.y)
                {
                    continue;
                }
                if (NewBox.Min.x >= Floors[i]->Min.x && NewBox.Max.x <= Floors[i]->Max.x)
                {
                    bCover = true;
                    break;
                }
				// left
                if (NewBox.Min.x < Floors[i]->Min.x)
                {
                    Box LeftBox(NewBox);
                    LeftBox.Max.x = Floors[i]->Min.x;
                    BoxList.push_back(LeftBox);
                    bSplited = true;
                    break;
                }
				// right
                if (Floors[i]->Max.x < NewBox.Max.x)
                {
                    Box RightBox(NewBox);
                    RightBox.Min.x = Floors[i]->Max.x;
                    BoxList.push_back(RightBox);
                    bSplited = true;
                    break;
                }
            }
            if (bSplited || bCover)
            {
                Itr = BoxList.erase(Itr);
            }
            else
            {
                ++Itr;
            }
        }

		if (!BoxList.empty())
		{
			for (Itr = BoxList.begin(); Itr != BoxList.end(); ++Itr)
			{
				CrossLink* NewCrossLink = new CrossLink();
				NewCrossLink->LeftStart = (*Itr).Min;
				NewCrossLink->LeftEnd = (*Itr).Min;
				NewCrossLink->LeftEnd.y = (*Itr).Max.y;

				NewCrossLink->RightStart = (*Itr).Min;
				NewCrossLink->RightStart.x = (*Itr).Max.x;
				NewCrossLink->RightEnd = (*Itr).Max;
				AddLink(EH, EL, NewCrossLink);
			}
			return true;
		}
    }
	return false;
}


bool BuildSameLineJumpLinks(NavMap* InMap, Edge* EH, Edge* EL)
{
	Edge* ELeft = NULL;
	Edge *ERight = NULL;
	// left
	if (EL->Point2.x <= EH->Point1.x)
	{
		ELeft = EL;
		ERight = EH;
	}
	if (EH->Point2.x <= EL->Point1.x)
	{
		ELeft = EH;
		ERight = EL;
	}

	if (ELeft && ERight)
	{
		bool bLinked = true;
		Segment Seg(ELeft->Point2, ERight->Point1);
		std::vector<Floor*> & Floors = InMap->Floors;
		for (size_t i = 0; i < Floors.size(); ++i)
		{
			//if (Floors[i]->bCanCross)
			//{
			//	continue;
			//}
			if (SegmentIntersectWithFloor(&Seg, Floors[i]))
			{
				//if (Seg.Point2.x <= Floors[i]->Min.x
				//	|| Seg.Point1.x >= Floors[i]->Max.x
				//	|| Seg.Point1.y >= Floors[i]->Max.y
				//	)
				//{
				//	continue;
				//}

				bLinked = false;

				break;
			}
		}
		if (bLinked)
		{
			JumpLink* NewJumpLink = new JumpLink;
			NewJumpLink->End = ELeft->Point2;
			NewJumpLink->Left = ERight->Point1;
			NewJumpLink->Right = ERight->Point2;
			AddLink(EH,EL,NewJumpLink);

			NewJumpLink = new JumpLink;
			NewJumpLink->End = ERight->Point1;
			NewJumpLink->Left = ELeft->Point1;
			NewJumpLink->Right = ELeft->Point2;
			AddLink(EH,EL,NewJumpLink);
			return true;
		}
		return false;
	}

	bool bLinked = false;
	if (EL->Point1.x >= EH->Point1.x && EL->Point1.x <= EH->Point2.x)
	{
		WalkLink* NewWalkLink = new WalkLink;
		NewWalkLink->Left = EL->Point1;
		NewWalkLink->Right = EL->Point1;

		AddLink(EH, EL, NewWalkLink);
		bLinked = true;
	}
	if (EL->Point2.x >= EH->Point1.x && EL->Point2.x <= EH->Point2.x)
	{
		WalkLink* NewWalkLink = new WalkLink;
		NewWalkLink->Left = EL->Point2;
		NewWalkLink->Right = EL->Point2;
		AddLink(EH, EL, NewWalkLink);
		bLinked = true;
	}

	if (EH->Point1.x >= EL->Point1.x && EH->Point1.x <= EL->Point2.x)
	{
		WalkLink* NewWalkLink = new WalkLink;
		NewWalkLink->Left = EH->Point1;
		NewWalkLink->Right = EH->Point1;
		AddLink(EH, EL, NewWalkLink);
		bLinked = true;
	}
	if (EH->Point2.x >= EL->Point1.x && EH->Point2.x <= EL->Point2.x)
	{
		WalkLink* NewWalkLink = new WalkLink;
		NewWalkLink->Left = EH->Point2;
		NewWalkLink->Right = EH->Point2;
		AddLink(EH, EL, NewWalkLink);
		bLinked = true;
	}

	return bLinked;
}

void AddLink(Edge* E1, Edge* E2,Link* NewLink)
{
	std::vector<Link*>* Links = NULL;
	Edge::AdjacentEdgeMap::iterator MapItr = E1->AdjacentEdges.find(E2);
	if (MapItr == E1->AdjacentEdges.end())
	{
		Links = new std::vector<Link*>;
		E1->AdjacentEdges.insert(Edge::AdjacentEdgeMap::value_type(E2, Links));
		E2->AdjacentEdges.insert(Edge::AdjacentEdgeMap::value_type(E1, Links));
	}
	else
	{
		Links = MapItr->second;
	}
	Links->push_back(NewLink);
}

bool BuildJumpLinks(NavMap* InMap, Edge* EH, Edge* EL)
{
	static int LinkLengthLimit = 2000;
    if (EH->Point1.x <= EL->Point1.x 
        && EH->Point2.x >= EL->Point2.x)
    {
        return false;
    }
	// same line
	if (EH->Point1.y == EL->Point1.y)
	{
		return BuildSameLineJumpLinks(InMap, EH, EL);
	}

    std::list<Triangle> TriangleList;
    std::vector<Floor*> Floors;
    //left 
    if (EL->Point1.x < EH->Point1.x)  
    {
        if (EL->Point2.x < EH->Point1.x)
        {
			Vector2D Left, Right;
			Left = EL->Point1;
			Right = EL->Point2;
			if (EH->Point1.x - Right.x < LinkLengthLimit)
			{
				if (EH->Point1.x - Left.x > LinkLengthLimit)
				{
					Left.x = EH->Point1.x - LinkLengthLimit;
				}
				TriangleList.push_back(Triangle(EH->Point1,Left,Right));
			}
        }
        else
        {
			Vector2D Left, Right;
			Left = EL->Point1;
			Right = Vector2D(EH->Point1.x, EL->Point1.y);
			if (EH->Point1.x - Left.x > LinkLengthLimit)
			{
				Left.x = EH->Point1.x - LinkLengthLimit;
			}
			TriangleList.push_back(Triangle(EH->Point1, Left, Right));
        }
        GetFloorsIntersectWithTriangle(InMap,Floors,*(TriangleList.rbegin()));
    }
    // right
    if (EH->Point2.x < EL->Point2.x)
    {
        if (EH->Point2.x < EL->Point1.x)
        {
			Vector2D Left, Right;
			Left = EL->Point1;
			Right = EL->Point2;
			if (Left.x - EH->Point2.x < LinkLengthLimit)
			{
				if (Right.x - EH->Point2.x > LinkLengthLimit)
				{
					Right.x = EH->Point2.x + LinkLengthLimit;
				}
				TriangleList.push_back(Triangle(EH->Point2,Left,Right));
			}
        }
        else
        {
			Vector2D Left, Right;
			Left = Vector2D(EH->Point2.x, EL->Point1.y);
			Right = EL->Point2;
			if (Right.x - EH->Point2.x > LinkLengthLimit)
			{
				Right.x = EH->Point2.x + LinkLengthLimit;
			}
			TriangleList.push_back(Triangle(EH->Point2, Left, Right));
        }
        GetFloorsIntersectWithTriangle(InMap,Floors,*(TriangleList.rbegin()));
    }

    std::list<Triangle>::iterator Itr = TriangleList.begin();
    while (Itr != TriangleList.end())
    {
		bool bErased = false;
        Triangle Tri(*Itr);

        Vector2D LeftDir(Tri.Left - Tri.Acme);
        Vector2D RightDir(Tri.Right - Tri.Acme);

        size_t FloorNum = Floors.size();
		for (size_t i = 0; i < FloorNum; ++i)
		{
			bool bSplited = false;
			bool bCoverLeft = false;
			bool bCoverRight = false;
			if (!TriangleIntersectWithFloor(Tri,Floors[i]))
			{
				continue;
			}

			if (Floors[i]->Max.y <= Tri.Left.y
				|| Tri.Acme.y <= Floors[i]->Min.y)
			{
				continue;
			}

			// left
			Vector2D Dir;
			if (Floors[i]->Min.x < Tri.Acme.x)
			{
				// avoid zero vector
				int MagixY = 0;
				if (Tri.Acme.y == Floors[i]->Max.y)
				{
					Tri.Acme.y += 1;
					MagixY = -1;
				}
				Dir = Vector2D(Vector2D(Floors[i]->Min.x, Floors[i]->Max.y) - Tri.Acme);
				if (Cross(Dir, LeftDir) >= 0)
				{
					bCoverLeft = true;
				}
				else if (Cross(Dir, LeftDir) * Cross(Dir, RightDir) < 0)
				{
					Triangle NewTri(Tri);
					int DeltX = (Tri.Acme.x - Floors[i]->Min.x) * (Tri.Acme.y - Tri.Right.y) / (Tri.Acme.y - Floors[i]->Max.y);
					NewTri.Right.x = Tri.Acme.x - DeltX;
					NewTri.Acme.y += MagixY;

					if (NewTri != Tri)
					{
						if (NewTri.Left.x < NewTri.Right.x)
						{
							TriangleList.push_back(NewTri);
						}
						bSplited = true;
					}
				}
				// recover
				Tri.Acme.y += MagixY;
			}
			else
			{
				Dir = Vector2D(Floors[i]->Min - Tri.Acme);
				if (Cross(Dir, LeftDir) >= 0)
				{
					bCoverLeft = true;
				}
				else if (Cross(Dir, LeftDir) * Cross(Dir, RightDir) < 0)
				{
					Triangle NewTri(Tri);
					NewTri.Right.x = Floors[i]->Min.x;
					if (NewTri != Tri)
					{
						if (NewTri.Left.x < NewTri.Right.x)
						{
							TriangleList.push_back(NewTri);
						}
						bSplited = true;
					}
				}
			}
			// right
			if (Tri.Acme.x < Floors[i]->Max.x)
			{
				// avoid zero vector
				int MagixY = 0;
				if (Tri.Acme.y == Floors[i]->Max.y)
				{
					Tri.Acme.y += 1;
					MagixY = -1;
				}
				Dir = Vector2D(Floors[i]->Max - Tri.Acme);
				if (Cross(RightDir, Dir) >= 0)
				{
					bCoverRight = true;
				}
				else if (Cross(Dir, LeftDir) * Cross(Dir, RightDir) < 0)
				{
					Triangle NewTri(Tri);
					int DeltX = (Floors[i]->Max.x - Tri.Acme.x) * (Tri.Acme.y - Tri.Right.y) / (Tri.Acme.y - Floors[i]->Max.y);
					NewTri.Left.x = Tri.Acme.x + DeltX;
					NewTri.Acme.y += MagixY;

					if (NewTri != Tri)
					{
						if (NewTri.Left.x < NewTri.Right.x)
						{
							TriangleList.push_back(NewTri);
						}
						bSplited = true;
					}
				}
			}
			else
			{
				Dir = Vector2D(Vector2D(Floors[i]->Max.x, Floors[i]->Min.y) - Tri.Acme);
				if (Cross(RightDir, Dir) >= 0)
				{
					bCoverRight = true;
				}
				else if (Cross(Dir, LeftDir) * Cross(Dir, RightDir) < 0)
				{
					Triangle NewTri(Tri);
					NewTri.Left.x = Floors[i]->Max.x;

					if (NewTri != Tri)
					{
						if (NewTri.Left.x < NewTri.Right.x)
						{
							TriangleList.push_back(NewTri);
						}
						bSplited = true;
					}
				}
			}
			if ((bCoverLeft && bCoverRight) || bSplited)
			{
				Itr = TriangleList.erase(Itr);
				bErased = true;
				break;
			}
		}

		if (!bErased)
		{
			++Itr;
		}
    }

	if (!TriangleList.empty())
	{
		for (Itr = TriangleList.begin(); Itr != TriangleList.end(); ++Itr)
		{
			JumpLink* NewJumpLink = new JumpLink;
			NewJumpLink->End = (*Itr).Acme;
			NewJumpLink->Left = (*Itr).Left;
			NewJumpLink->Right = (*Itr).Right;
			AddLink(EH, EL, NewJumpLink);
		}
		return true;
	}
	return false;
}

void GetFloorsIntersectWithAABB(NavMap* InMap,std::vector<Floor*>& OutFloors,Vector2D Min,Vector2D Max,bool bIgnoreCanCross)
{
    std::vector<Floor *> &Floors = InMap->Floors;

    size_t FloorNum = Floors.size();
    for (size_t i = 0; i < FloorNum; ++i)
    {
		if (bIgnoreCanCross && Floors[i]->bCanCross)
		{
			continue;
		}
		if (Floors[i]->Max.x <= Min.x
			|| Floors[i]->Min.x >= Max.x
			|| Floors[i]->Max.y <= Min.y
			|| Floors[i]->Min.y >= Max.y
			 )
		{
			continue;
		}
		OutFloors.push_back(Floors[i]);
    }
}

bool TriangleIntersectWithFloor(const Triangle& InTri, const Floor* InFloor)
{
	return (AABBIntersectWithLine(Segment(InTri.Acme, InTri.Left), Box(InFloor->Min, InFloor->Max))
		|| AABBIntersectWithLine(Segment(InTri.Acme, InTri.Right), Box(InFloor->Min, InFloor->Max))
		|| AABBIntersectWithLine(Segment(InTri.Left, InTri.Right), Box(InFloor->Min, InFloor->Max)));
}

void GetFloorsIntersectWithTriangle(NavMap* InMap,std::vector<Floor*>& OutFloors,const Triangle& InTri)
{
    std::vector<Floor *> &Floors = InMap->Floors;

    size_t FloorNum = Floors.size();
	for (size_t i = 0; i < FloorNum; ++i)
	{
		//if (Floors[i]->bCanCross)
		//{
		//	continue;
		//}
		if (TriangleIntersectWithFloor(InTri, Floors[i]))
		{
			OutFloors.push_back(Floors[i]);
		}
	}
}


int Dot(const Vector2D& Vec1,const Vector2D& Vec2)
{
    return Vec1.x * Vec2.x + Vec1.y * Vec2.y;
}

int64_t Cross(const Vector2D& Vec1,const Vector2D& Vec2)
{
	int64_t Ret = int64_t( Vec1.x * Vec2.y) - int64_t( Vec1.y * Vec2.x);
	return Ret;
}


bool AABBIntersectWithLine(const Segment& Seg, const Box& InBox)
{
	Vector2D Dir(Seg.Point2 - Seg.Point1);

	float TimeX = 0.0f, TimeY = 0.0f;
	// X
	if (Seg.Point1.x < InBox.Min.x)
	{
		if (Dir.x <= 0)
		{
			return false;
		}
		TimeX = (InBox.Min.x - Seg.Point1.x) * 1.0f / Dir.x;
	}
	else if (Seg.Point1.x > InBox.Max.x)
	{
		if (Dir.x >= 0)
		{
			return false;
		}
		TimeX = (InBox.Max.x - Seg.Point1.x) * 1.0f / Dir.x;
	}
	else
	{
		TimeX = 0;
	}

	// Y
	if (Seg.Point1.y < InBox.Min.y)
	{
		if (Dir.y <= 0)
		{
			return false;
		}
		TimeY = (InBox.Min.y - Seg.Point1.y) * 1.0f / Dir.y;
	}
	else if (Seg.Point1.y > InBox.Max.y)
	{
		if (Dir.y >= 0)
		{
			return false;
		}
		TimeY = (InBox.Max.y - Seg.Point1.y) * 1.0f / Dir.y;
	}
	else
	{
		TimeY = 0;
	}

	float HitTime = TimeX;
	if (TimeY > HitTime)
	{
		HitTime = TimeY;
	}

	if (HitTime >= 0 && HitTime <= 1.0)
	{
		float HitX = Seg.Point1.x + Dir.x * HitTime;
		float HitY = Seg.Point1.y + Dir.y * HitTime;

		if (HitX >= InBox.Min.x && HitX <= InBox.Max.x
			&& HitY >= InBox.Min.y && HitY <= InBox.Max.y)
		{
			return true;
		}
	}
	return false;
}

bool CheckMapData(NavMap* InMap)
{
	class ScopeFileHandle
	{
	public:
		ScopeFileHandle(std::string fileName, std::string fileMode)
		{
			fp = NULL;
			if (fopen_s(&fp, fileName.c_str(), fileMode.c_str()) != 0)
			{
				fp = NULL;
			}
		}
		~ScopeFileHandle()
		{
			if (fp)
				fclose(fp);
			fp = NULL;
		}
		FILE* fp;
	};
	ScopeFileHandle FileHandler("MapDebugData.txt", "w");

	bool bPrintLinks = true;
	bool bPrintEdges = true;
	bool bPrintFloors = true;

	std::set<Link*> LinkSet;
	size_t FloorNum = InMap->Floors.size();
	for (size_t i = 0; i < FloorNum; ++i)
	{
		if (FileHandler.fp && bPrintFloors)
		{
			// print floor
			fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Polygon[(%d,%d),(%d,%d),(%d,%d),(%d,%d)]\")\n",
				InMap->Floors[i]->Min.x, InMap->Floors[i]->Min.y,
				InMap->Floors[i]->Max.x, InMap->Floors[i]->Min.y,
				InMap->Floors[i]->Max.x, InMap->Floors[i]->Max.y,
				InMap->Floors[i]->Min.x, InMap->Floors[i]->Max.y);
		}

		size_t ENum = InMap->Floors[i]->ValidEdges.size();
		std::vector<V2DNav::Edge*>& Edges = InMap->Floors[i]->ValidEdges;
		for (size_t j = 0; j < ENum; ++j)
		{
			V2DNav::Edge* e = Edges[j];
			NAV_ASSERT(e->Point1.y == e->Point2.y);
			NAV_ASSERT(e->Point1.x < e->Point2.x);

			if (FileHandler.fp && bPrintEdges)
			{
				// print e
				fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
					e->Point1.x, e->Point1.y,
					e->Point2.x, e->Point2.y
				);
			}
			for (V2DNav::Edge::AdjacentEdgeMap::iterator eItr = e->AdjacentEdges.begin();
				eItr != e->AdjacentEdges.end(); ++eItr)
			{
				std::vector<V2DNav::Link*>* Links = eItr->second;
				for (size_t k = 0; k < Links->size(); ++k)
				{
					V2DNav::Link* link = (*Links)[k];
					V2DNav::WalkLink* WalkLink = link->LinkType == ELinkType::ELinkType_Walk ? static_cast<V2DNav::WalkLink*>(link): NULL;
					V2DNav::CrossLink* CrossLink = link->LinkType == ELinkType::ELinkType_Cross ? static_cast<V2DNav::CrossLink*>(link):NULL;
					V2DNav::JumpLink* JumpLink = link->LinkType == ELinkType::ELinkType_Jump ? static_cast<V2DNav::JumpLink*>(link):NULL;

					if (LinkSet.find(link) != LinkSet.end())
					{
						continue;
					}
					LinkSet.insert(link);

					// print link
					if (WalkLink)
					{
						NAV_ASSERT(WalkLink->Left.x <= WalkLink->Right.x);

						if (FileHandler.fp && bPrintLinks)
						{
							fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
								WalkLink->Left.x, WalkLink->Left.y,
								WalkLink->Right.x, WalkLink->Right.y
							);
						}
					}
					else if (CrossLink)
					{
						NAV_ASSERT(CrossLink->LeftStart.x == CrossLink->LeftEnd.x);
						NAV_ASSERT(CrossLink->LeftStart.y < CrossLink->LeftEnd.y);

						NAV_ASSERT(CrossLink->LeftStart.x <= CrossLink->RightStart.x);

						NAV_ASSERT(CrossLink->RightStart.x == CrossLink->RightEnd.x);
						NAV_ASSERT(CrossLink->RightStart.y < CrossLink->RightEnd.y);

						if (FileHandler.fp && bPrintLinks)
						{
							fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
								CrossLink->LeftStart.x, CrossLink->LeftStart.y,
								CrossLink->LeftEnd.x, CrossLink->LeftEnd.y
							);
							fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
								CrossLink->RightStart.x, CrossLink->RightStart.y,
								CrossLink->RightEnd.x, CrossLink->RightEnd.y
							);
						}
					}
					else if (JumpLink)
					{
						NAV_ASSERT(JumpLink->Left.x < JumpLink->Right.x);
						NAV_ASSERT(JumpLink->Left.y == JumpLink->Right.y);
						NAV_ASSERT(JumpLink->End.x <= JumpLink->Left.x || JumpLink->End.x >= JumpLink->Right.x);
						NAV_ASSERT(JumpLink->End.y >= JumpLink->Left.y);

						if (FileHandler.fp && bPrintLinks)
						{
							fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
								JumpLink->End.x, JumpLink->End.y,
								JumpLink->Left.x, JumpLink->Left.y
							);

							fprintf(FileHandler.fp, "ggbApplet.evalCommand(\"Segment[Point[{%d,%d}],Point[{%d,%d}]]\")\n",
								JumpLink->End.x, JumpLink->End.y,
								JumpLink->Right.x, JumpLink->Right.y
							);
						}
					}
				}
			}
		}
	}
	return true;
}

NavMap::~NavMap()
{
	std::set<std::vector<V2DNav::Link*>*> LinksSet;
	std::set<Edge*> EdgeSet;
	size_t FloorNum = Floors.size();
	for (size_t i = 0; i < FloorNum; ++i)
	{
		size_t ENum = Floors[i]->ValidEdges.size();
		std::vector<V2DNav::Edge*>& Edges = Floors[i]->ValidEdges;
		for (size_t j = 0; j < ENum; ++j)
		{
			V2DNav::Edge* e = Edges[j];
			if (EdgeSet.find(e) != EdgeSet.end())
			{
				continue;
			}
			EdgeSet.insert(e);
			
			for (V2DNav::Edge::AdjacentEdgeMap::iterator eItr = e->AdjacentEdges.begin();
				eItr != e->AdjacentEdges.end(); ++eItr)
			{
				std::vector<V2DNav::Link*>* Links = eItr->second;

				if (LinksSet.find(Links) != LinksSet.end())
				{
					continue;
				}
			}
		}
	}

	for (std::set<std::vector<V2DNav::Link*>*>::iterator LinksItr = LinksSet.begin();LinksItr != LinksSet.end(); ++LinksItr)
	{
		std::vector<V2DNav::Link*>* Links = *LinksItr;
		for (size_t i = 0; i < Links->size(); ++i)
		{
			delete (*Links)[i];
		}
		delete Links;
	}

	LinksSet.clear();

	for (std::set<Edge*>::iterator EdgeItr = EdgeSet.begin(); EdgeItr != EdgeSet.end(); ++EdgeItr)
	{
		delete *EdgeItr;
	}
	EdgeSet.clear();

	for (size_t i = 0; i < FloorNum; ++i)
	{
		delete Floors[i];
	}
}

};
