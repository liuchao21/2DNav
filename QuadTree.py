#coding=utf-8
from typing import Tuple,List,Optional
from NavType import *


MAX_ELEMENT=10

class CheckResult:
    def __init__(self):
        self.bHit = False
        self.hitTimer = -1 
        self.hitLocation = Point2DF(0.0,0.0)
        self.hitElement = None  #type:Interface_QuadTreeDataSupptor
        self.next = None #type:CheckResult
    def update(self,bHit:bool,hitTimer:float,hitLocation:Point2DF,hitElement:Interface_QuadTreeDataSupptor):
        self.bHit = bHit 
        self.hitTimer = hitTimer
        self.hitLocation = hitLocation
        self.hitElement = hitElement 

    def copy(self,inCheckResult:'CheckResult'):
        self.update(inCheckResult.bHit,inCheckResult.hitTimer,inCheckResult.hitLocation,inCheckResult.hitElement)
        self.next = inCheckResult.next
        

class QuadTreeNode:
    def __init__(self,bounds:Box2D) -> None:
        self.bounds = bounds        #type:Box2D
        self.container = ([],[])    #type:Tuple[list,list] 
        self.children = [None,None,None,None] #type:List[QuadTreeNode]

    def AddElement(self,newElement:Interface_QuadTreeDataSupptor):
        if not self._ShouldHaveChildren() or not self._HaveChildHold(newElement):
            self.container[0].append(newElement)
            return
        self.container[1].append(newElement)
        if len(self.container[0]) + len(self.container[1]) > MAX_ELEMENT and len(self.container[1]) > 0:
            self._PushElements()

    def _HaveChildHold(self,newElement):
        return self._GetChildIdx(newElement) >= 0

    def _PushElements(self):
        for e in self.container[1]:
            idx = self._GetChildIdx(e)
            if self.children[idx] == None:
                self.children[idx] = QuadTreeNode(self._GetChildBounds(idx))
            self.children[idx].AddElement(e)
        self.container[1].clear()
    
    def _GetChildBounds(self,idx:int) -> Optional[Box2D]:
        if idx == 0:
            return Box2D(self.bounds.leftBottomPoint,self.bounds.GetCenter())
        if idx == 1:
            center = self.bounds.GetCenter()
            leftBottomPoint = Point2D(center.x,self.bounds.GetMinY())
            rightUpPoint = Point2D(self.bounds.GetMaxX(),center.y)
            return Box2D(leftBottomPoint,rightUpPoint)
        if idx == 2:
            return Box2D(self.bounds.GetCenter(),self.bounds.rightUpPoint)
        if idx == 3:
            center = self.bounds.GetCenter()
            leftBottomPoint = Point2D(self.bounds.GetMinX(),center.y)
            rightUpPoint = Point2D(center.x,self.bounds.GetMaxY())
            return Box2D(leftBottomPoint,rightUpPoint)
        return None
            
    def _GetChildIdx(self,newElement:Interface_QuadTreeDataSupptor):
        for i in range(4):
            if self._GetChildBounds(i).ContainBox(newElement.GetBounds()):
                return i
        return -1

    def _ShouldHaveChildren(self) -> bool:
        if self.bounds.GetMaxX() - self.bounds.GetMinX() <= 1:
            return False

        if self.bounds.GetMaxY() - self.bounds.GetMinY() <= 1:
            return False
        return True

    def LineCheck(self,outCheckResult:CheckResult,end:Point2D,start:Point2D,source:Interface_QuadTreeDataSupptor):

        tailResult = outCheckResult
        allElements = self.container[0] + self.container[1] #type:List[Interface_QuadTreeDataSupptor]
        for e in allElements:
            if e != source:
                bHit,hitTimer,hitLocation = e.GetBounds().LineCheck(end,start)
                if bHit:
                    #first init header
                    if not tailResult.bHit:
                        tailResult.update(bHit,hitTimer,hitLocation,e)
                    else:
                        newCheckResult = CheckResult()
                        newCheckResult.update(bHit,hitTimer,hitLocation,e)
                        tailResult.next = newCheckResult
                        tailResult = newCheckResult
        
        for i in range(4):
            if self.children[i] != None:
                bHit,hitTimer,hitLocation = self._GetChildBounds(i).LineCheck(end,start)
                if bHit:
                    self.children[i].LineCheck(tailResult,end,start,source)

class QuadTree:
    def __init__(self,bounds:Box2D) -> None:
        self.root = QuadTreeNode(bounds)

    # 1 + 4 + 4^2 + 4^3 + ... + 4^(n - 1) = (4^n - 1)/3 > 4^(n-1) 
    # if n = 9,4^(9-1) = 2^16 = 65536,2^9=512,
    def AddElement(self,newElement:Interface_QuadTreeDataSupptor):
        self.root.AddElement(newElement)

    def LineCheck(self,outCheckResult:CheckResult,end:Point2D,start:Point2D,source:Interface_QuadTreeDataSupptor,bSingleResult = False):
        internalCheckResult = CheckResult()
        self.root.LineCheck(internalCheckResult,end,start,source)

        if bSingleResult:
            if internalCheckResult.bHit:
                #init
                outCheckResult.update(internalCheckResult.bHit,internalCheckResult.hitTimer,internalCheckResult.hitLocation,internalCheckResult.hitElement)
                #walk throuth result list
                while internalCheckResult != None:
                    if internalCheckResult.hitTimer < outCheckResult.hitTimer:
                        outCheckResult.update(internalCheckResult.bHit,internalCheckResult.hitTimer,internalCheckResult.hitLocation,internalCheckResult.hitElement)
                    internalCheckResult = internalCheckResult.next
        else:
            if internalCheckResult.bHit:
                outCheckResult.copy(internalCheckResult)


        



    