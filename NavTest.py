#coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


import NavType
from QuadTree import *   
from GridNav import *
from Utility import *


class GridMapFigure:
    def __init__(self):
        self.fig,self.ax = plt.subplots( figsize=(16, 9)) 
        plt.axis([-100,100,-100,100])
        plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)

        self.zp = ZoomPan()
        self.figZoom = self.zp.zoom_factory(self.ax,base_scale=1.05)
        self.figPan = self.zp.pan_factory(self.ax)

    def SetAxisToOrigin(self):
        self.ax.spines['right'].set_color('none')
        self.ax.spines['top'].set_color('none')
        self.ax.xaxis.set_ticks_position('bottom')
        self.ax.spines['bottom'].set_position(('data',0))
        self.ax.yaxis.set_ticks_position('left')
        self.ax.spines['left'].set_position(('data',0))

    def DrawRect(self,minX,minY,maxX,maxY,color='g',alpha=0.1):
        rect = plt.Rectangle((minX,minY),maxX - minX,maxY - minY,color=color,alpha=alpha)
        self.ax.add_patch(rect)

    def DrawRectByBox2D(self,bounds:NavType.Box2D,color='g',alpha=0.1):
        w = bounds.GetMaxX() - bounds.GetMinX()
        h = bounds.GetMaxY() - bounds.GetMinY()
        minX = bounds.GetMinX()
        minY = bounds.GetMinY()
        rect = plt.Rectangle((minX,minY),w,h,color=color,alpha=alpha)
        self.ax.add_patch(rect)

    def Show(self):
        plt.show()

mapFigure = GridMapFigure()
mapFigure.SetAxisToOrigin()
bounds = NavType.Box2D(NavType.Point2D(-100,-100),NavType.Point2D(100,100))

mapFigure.DrawRectByBox2D(bounds,color='b')
obstacle = [
    NavType.Box2D(NavType.Point2D(-100,-100),NavType.Point2D(-50,-70)),
    NavType.Box2D(NavType.Point2D(-30,-30),NavType.Point2D(10,-5)),
    NavType.Box2D(NavType.Point2D(-70,0),NavType.Point2D(-10,20)),
    NavType.Box2D(NavType.Point2D(50,-50),NavType.Point2D(75,50)),
    NavType.Box2D(NavType.Point2D(45,15),NavType.Point2D(55,30)),
    NavType.Box2D(NavType.Point2D(45,-30),NavType.Point2D(55,-15)),
    NavType.Box2D(NavType.Point2D(-75,30),NavType.Point2D(-50,80)),
    NavType.Box2D(NavType.Point2D(-65,75),NavType.Point2D(-45,80)),
    NavType.Box2D(NavType.Point2D(-65,30),NavType.Point2D(-45,50))
    ]

for rect in obstacle:
    mapFigure.DrawRectByBox2D(rect,color='gray',alpha=0.5)

grounds = []  #type:List[NavType.Obstacle]

for rect in obstacle:
    ground = NavType.Obstacle()
    ground.UpdateBounds(rect)
    grounds.append(ground)

mapBuilder = MapBuilder()
mapBuilder.BuildMap(bounds,grounds)

for grid in mapBuilder.gridList:
    mapFigure.DrawRectByBox2D(grid.GetBounds(),color='r')


mapFigure.Show()


