import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def key(self):
        return str(self.x) + "," + str(self.y)

    def equals(self,compPoint):
        return self.x == compPoint.x and self.y == compPoint.y




class Node:
    point = None
    orientation = None
    prevNode = None
    g_cost = -1
    cost = -1
    endPoint = None

    #constructor
    def __init__(self,nodePoint,initOri,endPoint,previousNode):
       self.point = nodePoint
       self.prevNode = previousNode
       self.orientation = initOri
       self.endPoint = endPoint
       self.calcCost(endPoint)



    def calcCost(self, endPoint):
        if(self.prevNode != None):
            rotCost = abs(self.orientation - self.prevNode.orientation)
            if(rotCost >2):
                rotCost = 1
            self.g_cost = self.prevNode.g_cost+1+rotCost

        else:
            self.g_cost = 0
        #h_cost = abs(self.point.x-endPoint.x) + abs(self.point.y-endPoint.y)
        h_cost = math.sqrt((self.point.x - endPoint.x)**2 + (self.point.y - endPoint.y)**2)
        self.cost = self.g_cost/2 + h_cost/2

    def key(self):
        return self.point.key()

    def createNewNodes(self, curNodes, world, blockedThresh):
        dx = [1,0,-1,0]
        dy = [0,1,0,-1]
        theta = [1,2,3,4]
        newNodes = []
        for i in range(0,4):
            tempX = self.point.x+dx[i]
            tempY = self.point.y+dy[i]
            key = str(tempX)+","+str(tempY)
            #print key
            if (not ((key in curNodes) or tempX < 0 or tempX >= world.width or tempY < 0 or tempY >= world.height or (world.getVal(tempX,tempY)) >= blockedThresh)):
                #print "added node"
                newNodes.append(Node(Point(tempX,tempY),theta[i], self.endPoint, self))
        return newNodes




class Grid:


    def __init__(self,width,height,data):
        self.width = width
        self.height = height
        self.data = data

    def getVal(self,x,y):
        return self.data[x+y*self.width]

    def setVal(self,x,y,val):
        self.data[x+y*self.width] = val
