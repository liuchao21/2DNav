#coding=utf-8
from typing import Tuple,Optional,Union
import math

class Interface_QuadTreeDataSupptor:
    def GetBounds(self) -> 'Box2D':
        pass

class Point2DF:
    def __init__(self,x:float,y:float) -> None:
        self.x = x
        self.y = y
    def __sub__(self, inPoint:Union['Point2D','Point2DF']) -> 'Vector2DF':
        return Vector2DF(self.x - inPoint.x,self.y - inPoint.y)

class Point2D:
    def __init__(self,x:int,y:int) -> None:
        self.x = x
        self.y = y
    def __sub__(self, inPoint:'Point2D') -> 'Vector2D':
        return Vector2D(self.x - inPoint.x,self.y - inPoint.y)
    def __eq__(self,inPoint):
        return self.x == inPoint.x and self.y == inPoint.y
    
class Vector2D:
    def __init__(self,x:int,y:int) -> None:
        self.x = x
        self.y = y

    def __sub__(self, inVec:'Vector2D') -> 'Vector2D':
        return Vector2D(self.x - inVec.x,self.y - inVec.y)
    def __add__(self,inVec:'Vector2D') -> 'Vector2D':
        return Vector2D(self.x + inVec.x,self.y + inVec.y)
    def __mul__(self,fScalar:float) -> 'Vector2DF':
        return Vector2DF(self.x * fScalar,self.y * fScalar)
    def __rmul__(self,fScalar:float) -> 'Vector2DF':
        return self * fScalar
    def __matmul__(self,inVec:'Vector2D') -> float:
        return self.x * inVec.x + self.y * inVec.y
    def __rmatmul__(self,inVec:'Vector2D') -> float:
        return self @ inVec

    def sizeSq(self) -> float:
        return self.x*self.x + self.y*self.y

class Vector2DF:
    def __init__(self,x:float,y:float) -> None:
        self.x = x
        self.y = y
    def __sub__(self, inVec:Union[Vector2D, 'Vector2DF']) -> 'Vector2DF':
        return Vector2DF(self.x - inVec.x,self.y - inVec.y)
    def __add__(self,inVec:Union[Vector2D, 'Vector2DF']) -> 'Vector2DF':
        return Vector2DF(self.x + inVec.x,self.y + inVec.y)
    def __mul__(self,fScalar:float) -> 'Vector2DF':
        return Vector2DF(self.x * fScalar,self.y * fScalar)
    def __rmul__(self,fScalar:float) -> 'Vector2DF':
        return self * fScalar
    def __matmul__(self,inVec:Union[Vector2D, 'Vector2DF']) -> float:
        return self.x * inVec.x + self.y * inVec.y
    def __rmatmul__(self,inVec:Union[Vector2D, 'Vector2DF']) -> float:
        return self @ inVec
    
    def size(self) -> float:
        return math.sqrt(self.x*self.x + self.y*self.y)


class Edge:
    def __init__(self,point1:Point2D,point2:Point2D) -> None:
        self.point1 = point1
        self.point2 = point2
        self.neighbor1 = None
        self.neighbor2 = None

    def __eq__(self,inEdge) -> bool:
        return ((self.point1 == inEdge.point1 and self.point2 == inEdge.point2)
                or (self.point1 == inEdge.point2 and self.point2 == inEdge.point1))
    def GetMinX(self):
        return min(self.point1.x,self.point2.x)

    def GetMinY(self):
        return min(self.point1.y,self.point2.y)

    def GetMaxX(self):
        return max(self.point1.x,self.point2.x)

    def GetMaxY(self):
        return max(self.point1.y,self.point2.y)

class Box2D:
    def __init__(self,leftBottomPoint:Point2D,rightUpPoint:Point2D) -> None:
        self.leftBottomPoint = leftBottomPoint
        self.rightUpPoint = rightUpPoint
    def GetCenter(self):
        centerX = (self.leftBottomPoint.x + self.rightUpPoint.x) // 2
        centerY = (self.leftBottomPoint.y + self.rightUpPoint.y) // 2
        return Point2D(centerX,centerY)
    def GetMinX(self):
        return self.leftBottomPoint.x
    def GetMinY(self):
        return self.leftBottomPoint.y
    def GetMaxX(self):
        return self.rightUpPoint.x
    def GetMaxY(self):
        return self.rightUpPoint.y
    def ContainBox(self,inBox:'Box2D') -> bool:
        return self.ContainPoint(inBox.leftBottomPoint) and self.ContainPoint(inBox.rightUpPoint)
    def ContainPoint(self,inPoint:Point2D) -> bool:
        return (inPoint.x >= self.leftBottomPoint.x and inPoint.x <= self.rightUpPoint.x and
                inPoint.y >= self.leftBottomPoint.y and inPoint.y <= self.rightUpPoint.y)
    
    def LineCheck(self,end:Point2D,start:Point2D) -> Tuple[bool,float,Point2DF]:
        bHit = False
        hitTimer = 0.0
        hitLocation = Point2DF(0.0,0.0)
        timerX = 0.0
        timerY = 0.0

        if start.x < self.GetMinX():
            if end.x <= start.x:
                return bHit,hitTimer,hitLocation 
            timerX = (self.GetMinX() - start.x) / (end.x - start.x)
        elif start.x > self.GetMaxX():
            if end.x >= start.x:
                return bHit,hitTimer,hitLocation 
            timerX = (start.x - self.GetMaxX()) / (start.x - end.x)
        else:
            timerX = 0.0

        if start.y < self.GetMinY():
            if end.y <= start.y:
                return bHit,hitTimer,hitLocation 
            timerY = (self.GetMinY() - start.y) / (end.y - start.y)
        elif start.y > self.GetMaxY():
            if end.y >= start.y:
                return bHit,hitTimer,hitLocation 
            timerY = (start.y - self.GetMaxY()) / (start.y - end.y)
        else:
            timerY = 0.0
        
        hitTimer = max(timerX,timerY)
        dir = Vector2DF(end.x - start.x,end.y - start.y)
        if hitTimer >= 0.0 and hitTimer <= 1.0:
            size = dir * hitTimer
            hitLocation = Point2DF(start.x + size.x,start.y + size.y)
            
            if (hitLocation.x > self.GetMinX() - 0.1 and hitLocation.x < self.GetMaxX() + 0.1
                and hitLocation.y > self.GetMinY() - 0.1 and hitLocation.y < self.GetMaxY() + 0.1):
                return True,hitTimer,hitLocation
        
        return False,0,Point2DF(0,0)


class Grid(Interface_QuadTreeDataSupptor):
    def __init__(self):
        self.leftEdges = [] #type:List[Edge]
        self.rightEdges = [] #type:List[Edge]
        self.upperEdge = None #type:Edge
        self.bottomEdge = None #type:Edge

    def AddLeftEdge(self,e:Edge):
        self.leftEdges.append(e)
    def AddRightEdge(self,e:Edge):
        self.rightEdges.append(e)
    def SetUpperEdge(self,e:Edge):
        self.upperEdge = e
    def SetBottomEdge(self,e:Edge):
        self.bottomEdge = e
    def Reset(self):
        self.leftEdges = [] #type:List[Edge]
        self.rightEdges = [] #type:List[Edge]
        self.upperEdge = None #type:Edge
        self.bottomEdge = None #type:Edge

    def GetBounds(self):
        minX = self.leftEdges[0].GetMinX()
        minY = self.bottomEdge.GetMinY()
        maxX = self.rightEdges[0].GetMaxX()
        maxY = self.upperEdge.GetMaxY()
        return Box2D(Point2D(minX,minY),Point2D(maxX,maxY))

class Obstacle(Interface_QuadTreeDataSupptor):
    def __init__(self):
        self.bounds = Box2D(Point2D(0,0),Point2D(0,0))
        self.bCanCrossUp = False
        self.bCanCrossDown = False
    def copy(self,ob:'Obstacle'):
        self.bounds = ob.bounds
        self.bCanCrossUp = ob.bCanCrossUp
        self.bCanCrossDown = ob.bCanCrossDown
    def GetBounds(self):
        return self.bounds
    def UpdateBounds(self,bounds:Box2D):
        self.bounds = bounds

 