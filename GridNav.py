#coding=utf-8

from NavType import *
from QuadTree import *   


class MapBuilder:
    
    def __init__(self):
        self.gridList = [] #type:List[Grid]
        self.sceneTree = None #type:QuadTree
        self.bounds = None #type:Box2D
        self.obstacles = None #type:List[Obstacle]

    def BuildMap(self,bounds:Box2D,obstacles:List[Obstacle]):
        self.sceneTree = QuadTree(bounds)
        self.bounds = bounds
        for ob in obstacles:
            self.sceneTree.AddElement(ob)

        xValues = [] #type:List[Tuple[int,bool,Obstacle]]

        xValues.append((bounds.GetMinX(),False,None))
        for ob in obstacles:
            xValues.append((ob.GetBounds().GetMinX(),True,ob))
            xValues.append((ob.GetBounds().GetMaxX(),False,ob))
        xValues.append((bounds.GetMaxX(),True,None))

        xValues.sort(key = lambda e: e[0])

        #leftEdges = [Edge(Point2D(bounds.GetMinX(),bounds.GetMinY()),Point2D(bounds.GetMinX(),bounds.GetMaxY()))]
        leftEdges = [] #type:List[Edge]

        rightEdges = [] #type:List[Tuple[Edge,bool]]
        for i in range(len(xValues) + 1):
            if i == len(xValues) or (i > 0 and xValues[i - 1][0] != xValues[i][0]):
                if len(rightEdges) > 0: 
                    x = xValues[i - 1][0] 
                    # add non-existent edge to finish finding upper edge of new grid 
                    rightEdges.append((Edge(Point2D(x,bounds.GetMaxY() + 1),Point2D(x,bounds.GetMaxY() + 2)),True))
                    #sort 
                    rightEdges.sort(key = lambda e: e[0].GetMinY())
                    # remove repeated edge
                    edgeNum = len(rightEdges)
                    idx = 1
                    while idx < edgeNum:
                        if rightEdges[idx][0] == rightEdges[idx - 1][0]:
                            del rightEdges[idx]
                            edgeNum -= 1
                            continue
                        idx += 1

                    #use right edges to close left edges and get grids
                    newGrid = Grid()
                    closedEdge = []

                    rIdx = 0
                    lIdx = 0
                    startY = rightEdges[rIdx][0].GetMinY()
                    endY = rightEdges[rIdx][0].GetMaxY()
                    bottomY = -1
                    upY = -1

                    #walk through edges from bottom to up
                    while rIdx < edgeNum - 1 and lIdx < len(leftEdges):
                        if leftEdges[lIdx].GetMaxY() <= startY:
                            # current right edge can not close this left edge,try next.
                            lIdx += 1
                            continue
                        if  endY <= leftEdges[lIdx].GetMinY():
                            #no left edge can be closed by current right edge.
                            rIdx += 1
                            #update Y
                            startY = rightEdges[rIdx][0].GetMinY()
                            endY = rightEdges[rIdx][0].GetMaxY()
                            continue

                        if leftEdges[lIdx].GetMinY() <= startY:
                            if leftEdges[lIdx].GetMaxY() == endY:

                                newGrid.AddLeftEdge(leftEdges[lIdx])
                                newGrid.AddRightEdge(Edge(Point2D(x,rightEdges[rIdx][0].GetMinY()),Point2D(x,rightEdges[rIdx][0].GetMaxY())))
                                closedEdge.append(lIdx)

                                if bottomY == -1:
                                    bottomY = startY

                                if (rightEdges[rIdx][0].GetMaxY() < rightEdges[rIdx + 1][0].GetMinY() or 
                                    lIdx == len(leftEdges) or leftEdges[lIdx].GetMaxY() < leftEdges[lIdx + 1].GetMinY()):
                                    #get new grid
                                    if upY == -1:
                                        upY = endY
                                        leftX = leftEdges[lIdx].GetMinX()
                                        rightX = x
                                        newGrid.SetBottomEdge(Edge(Point2D(leftX,bottomY),Point2D(rightX,bottomY)))
                                        newGrid.SetUpperEdge(Edge(Point2D(leftX,upY),Point2D(rightX,upY)))
                                        self.gridList.append(newGrid)
                                        # reset grid info to get next new grid
                                        newGrid = Grid()
                                        bottomY = -1
                                        upY = -1

                                lIdx += 1
                                rIdx += 1
                                #update Y
                                startY = rightEdges[rIdx][0].GetMinY()
                                endY = rightEdges[rIdx][0].GetMaxY()

                            elif leftEdges[lIdx].GetMaxY() < endY:

                                newGrid.AddLeftEdge(leftEdges[lIdx])
                                closedEdge.append(lIdx)

                                if bottomY == -1:
                                    bottomY = startY
                                #update Y
                                startY = leftEdges[lIdx].GetMaxY()
                                lIdx += 1

                            else: #leftEdges[lIdx].GetMaxY() > endY:

                                newGrid.AddRightEdge(Edge(Point2D(x,rightEdges[rIdx][0].GetMinY()),Point2D(x,rightEdges[rIdx][0].GetMaxY())))

                                if bottomY == -1:
                                    bottomY = startY

                                if rightEdges[rIdx][0].GetMaxY() < rightEdges[rIdx + 1][0].GetMinY():
                                    #get new grid
                                    if upY == -1:
                                        upY = endY
                                        leftX = leftEdges[lIdx].GetMinX()
                                        rightX = x
                                        newGrid.SetBottomEdge(Edge(Point2D(leftX,bottomY),Point2D(rightX,bottomY)))
                                        newGrid.SetUpperEdge(Edge(Point2D(leftX,upY),Point2D(rightX,upY)))
                                        self.gridList.append(newGrid)
                                        # reset grid info to get next new grid
                                        newGrid = Grid()
                                        bottomY = -1
                                        upY = -1

                                rIdx += 1
                                #update Y
                                startY = rightEdges[rIdx][0].GetMinY()
                                endY = rightEdges[rIdx][0].GetMaxY()
                    closedEdge.sort(reverse=True)
                    for lIdx in closedEdge:
                        del leftEdges[lIdx]
                    #update leftEdge
                    for e in rightEdges:
                        if not e[1]:
                            leftEdges.append(e[0])
                    leftEdges.sort(key = lambda e:e.GetMinY())
                    rightEdges.clear()
                    yield leftEdges

            if i == len(xValues):
                continue

            x = xValues[i][0]
            bOpen = xValues[i][1]
            source = xValues[i][2]  #type:Obstacle

            if source != None:
                # middle edge
                start = Point2D(x,source.GetBounds().GetMinY())
                end = Point2D(x,source.GetBounds().GetMaxY())
                self._GetRightEdges(rightEdges,end,start,source,True,bOpen)
                # trace towards down
                start = Point2D(x,source.GetBounds().GetMinY())
                end = Point2D(x,bounds.GetMinY() - 1)
                self._GetRightEdges(rightEdges,end,start,source,False,False)
                # trace towards up
                start = Point2D(x,source.GetBounds().GetMaxY())
                end = Point2D(x,bounds.GetMaxY() + 1)
                self._GetRightEdges(rightEdges,end,start,source,False,False)
            else:
                start = Point2D(x,self.bounds.GetMinY())
                end = Point2D(x,self.bounds.GetMaxY())
                self._GetRightEdges(rightEdges,end,start,None,True,bOpen)

                
    def _SpliteVEdge(self,bounds:Box2D,end:Point2D,start:Point2D,outSplitedEdge:List) -> bool:
        assert start.x == end.x and start.y != end.y
        bSplited = False
        bReverse = False

        if end.y < start.y:
            #swap start and end
            start.y,end.y = end.y,start.y
            bReverse = True

        if start.x >= bounds.GetMinX() and start.x <= bounds.GetMaxX():
            if end.y >= bounds.GetMinY() and start.y <= bounds.GetMaxY():
                if start.y < bounds.GetMinY():
                    newStart = Point2D(start.x,start.y)
                    newEnd = Point2D(start.x,bounds.GetMinY())
                    outSplitedEdge.append((newStart,newEnd))

                if end.y > bounds.GetMaxY():
                    newStart = Point2D(start.x,bounds.GetMaxY())
                    newEnd = Point2D(start.x,end.y)
                    outSplitedEdge.append((newStart,newEnd))
                bSplited = True

        #recover 
        if bReverse:
            start.y,end.y = end.y,start.y
            bReverse = False
        return bSplited

    def _GetRightEdges(self,rightEdges:List,end:Point2D,start:Point2D,source:Interface_QuadTreeDataSupptor, bFromObstacle:bool,bIsOpen:bool):
        if bFromObstacle:
            lineTrace = CheckResult()
            self.sceneTree.LineCheck(lineTrace,end,start,source)
            splitedEdges = [(start,end)]
            if lineTrace.bHit:
                while lineTrace != None:
                    assert lineTrace.bHit and lineTrace.hitElement != None
                    currentEdgeNum = len(splitedEdges)
                    idx = 0
                    while idx < currentEdgeNum:
                        e = splitedEdges[idx]
                        if self._SpliteVEdge(lineTrace.hitElement.GetBounds(),e[1],e[0],splitedEdges):
                            del splitedEdges[idx]
                            currentEdgeNum -= 1
                            continue
                        idx += 1
                    lineTrace = lineTrace.next
            if len(splitedEdges) > 0:
                for e in splitedEdges:
                    rightEdges.append((Edge(e[0],e[1]),bIsOpen))
        else:
            checkResult = CheckResult()
            self.sceneTree.LineCheck(checkResult,end,start,source,True)
            if checkResult.bHit:
                if checkResult.hitTimer > 0.0:
                    #trace up
                    if end.y > start.y:
                        end.y = checkResult.hitElement.GetBounds().GetMinY()
                        if end.y != start.y:
                            rightEdges.append((Edge(start,end),bIsOpen))
                    #trace down
                    elif end.y < start.y:
                        end.y = checkResult.hitElement.GetBounds().GetMaxY()
                        if end.y != start.y:
                            rightEdges.append((Edge(end,start),bIsOpen))
            else:
                #trace up
                if end.y > start.y:
                    end.y = self.bounds.GetMaxY()
                    if end.y > start.y:
                        rightEdges.append((Edge(start,end),bIsOpen))
                #trace down
                elif end.y < start.y:
                    end.y = self.bounds.GetMinY()
                    if end.y != start.y:
                        rightEdges.append((Edge(end,start),bIsOpen))



