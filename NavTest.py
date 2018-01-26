#coding=utf-8

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


from NavType import *
from QuadTree import *   
from GridNav import *
from Utility import *

initialVelocity = 30
jumpHeight = 50 
jumpLength = 50 
g = None
def onKeyPress(event):
    global g
    if event.key != ' ':
        return
    if g != None:
        e = next(g)
        if len(e) > 0:
            for l in e:
                mapFigure.DrawSegment(l.point1,l.point2)
        #mapFigure.DrawRectByBox2D(grid.GetBounds(),color='r')
 

def onPress(event):
    global lineA,lineB
    if event.button == 1:
        #clear lines
        mapFigure.ax.lines.clear()

        startPos = Vector2D(0,0)
        endPos = Vector2D(float(event.xdata),float(event.ydata))
        deltaX = endPos.x - startPos.x
        deltaY = endPos.y - startPos.y

        mapFigure.DrawParabola(initialVelocity,startPos,endPos)
        #times = 1 + max( (abs(int(deltaX)) - 1) // jumpHeight,(abs(int(deltaY)) - 1) // jumpLength )
        #for i in range(times):
        #    endPos.x = startPos.x +  1 / times * deltaX
        #    endPos.y = startPos.y +  1 / times * deltaY
        #    mapFigure.DrawParabola(initialVelocity,startPos,endPos)
        #    startPos.x = endPos.x
        #    startPos.y = endPos.y

class GridMapFigure:
    def __init__(self):
        self.fig,self.ax = plt.subplots( figsize=(16, 9)) 
        plt.axis([-1000,1000,-1000,1000])
        plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)

        self.zp = ZoomPan()
        self.figZoom = self.zp.zoom_factory(self.ax,base_scale=1.05)
        self.figPan = self.zp.pan_factory(self.ax)

        #self.fig.canvas.mpl_connect('button_press_event',onPress)
        self.fig.canvas.mpl_connect('key_press_event',onKeyPress)


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

    def DrawSegment(self,startPos:Point2D,endPos:Point2D):
        plt.plot([startPos.x,endPos.x],[startPos.y,endPos.y])

    def DrawRectByBox2D(self,bounds:Box2D,color='g',alpha=0.1):
        w = bounds.GetMaxX() - bounds.GetMinX()
        h = bounds.GetMaxY() - bounds.GetMinY()
        minX = bounds.GetMinX()
        minY = bounds.GetMinY()
        if w <= 0 or h <= 0:
            rect = plt.Rectangle((minX,minY),w,h,color=color,alpha=alpha)
        rect = plt.Rectangle((minX,minY),w,h,color=color,alpha=alpha)
        self.ax.add_patch(rect)

    def DrawParabola(self,initialVelocity,startPos:Vector2D,endPos:Vector2D):
        g = 980
        sampleX = np.linspace(0,endPos.x - startPos.x)
        deltaX = endPos.x - startPos.x
        deltaY = endPos.y - startPos.y

        #insideSqrt = initialVelocity**4 - g * (g * deltaX**2 + 2 * deltaY * initialVelocity**2)
        #if insideSqrt <= 0:
        #    return
        MaxYSpeed = 304
        initialVelocity = MaxYSpeed / 2
        TanAngleA = (deltaY + (deltaX**2 * g) / (2 * initialVelocity**2)) / deltaX;
	    #float ZSpeed = MaxYSpeed * TanAngle;

     #   if (FMath::Abs(ZSpeed) > MaxZSpeed)
     #   {
     #       return false;
     #   } 
        #TanAngleA = (initialVelocity**2 + math.sqrt(insideSqrt)) / (g * deltaX)
        #TanAngleB = (initialVelocity**2 - math.sqrt(insideSqrt)) / (g * deltaX)
        #if TanAngleA > TanAngleB:
        #plt.plot(startPos.x + sampleX,startPos.y + TanAngleA * sampleX - sampleX**2 * g * (1 + TanAngleA**2) / (2 * initialVelocity**2))
        #else:
        #    plt.plot(startPos.x + sampleX,startPos.y + TanAngleB * sampleX - sampleX**2 * g * (1 + TanAngleB**2) / (2 * initialVelocity**2))
        plt.plot(startPos.x + sampleX,startPos.y + TanAngleA * sampleX - sampleX**2 * g / (2 * initialVelocity**2))

    def Show(self):
        plt.show()

mapFigure = GridMapFigure()
mapFigure.SetAxisToOrigin()

#bounds = Box2D(Point2D(-100,-100),Point2D(100,100))
#
obstacles = [] #type:List[Box2D]
foundBounds = False
bounds = Box2D(Point2D(0,0),Point2D(0,0))

with open("MapData.pathfind",mode='r') as mapFile:
    while True:
        data = mapFile.readline() #type:str
        if data == "":
            break
        obstacle = data.split(' ')
        obstacle = [i.strip('\n') for i in obstacle]
        if obstacle[0] == "Map":
            continue
        if not foundBounds:
            bounds = Box2D(Point2D(int(obstacle[0]),int(obstacle[1])),Point2D(int(obstacle[2]),int(obstacle[3])))
            foundBounds = True
        else:
            try:
                obstacles.append(Box2D(Point2D(int(obstacle[0]),int(obstacle[1])),Point2D(int(obstacle[2]),int(obstacle[3]))))
            except ValueError as e:
                print(obstacle)
                

if foundBounds:
    mapFigure.DrawRectByBox2D(bounds,color='b')

grounds = []  #type:List[Obstacle]

for rect in obstacles:
    # right
    if rect.GetMinX() >= bounds.GetMaxX():
        continue
    # left
    if rect.GetMaxX() <= bounds.GetMinX():
        continue
    # down
    if rect.GetMaxY() <= bounds.GetMinY():
        continue
    # up
    if rect.GetMinY() >= bounds.GetMaxY():
        continue
    ground = Obstacle()
    if rect.GetMinX() < bounds.GetMinX():
        rect.leftBottomPoint.x = bounds.leftBottomPoint.x
    if rect.GetMinY() < bounds.GetMinY():
        rect.leftBottomPoint.y = bounds.leftBottomPoint.y
    if rect.GetMaxX() > bounds.GetMaxX():
        rect.rightUpPoint.x = bounds.rightUpPoint.x
    if rect.GetMaxY() > bounds.GetMaxY():
        rect.rightUpPoint.y = bounds.rightUpPoint.y

    if rect.GetMinX() != rect.GetMaxX() and rect.GetMinY() != rect.GetMaxY():
        mapFigure.DrawRectByBox2D(rect,color='gray',alpha=0.5)
        ground.UpdateBounds(rect)
        grounds.append(ground)

mapBuilder = MapBuilder()
g = mapBuilder.BuildMap(bounds,grounds)
print (type(g))

#for grid in mapBuilder.gridList:
#    mapFigure.DrawRectByBox2D(grid.GetBounds(),color='r')

#mapFigure.DrawParabola(initialVelocity,Vector2D(0,0),Vector2D(35,50))

       


mapFigure.Show()


