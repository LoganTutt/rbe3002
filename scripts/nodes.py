class Point:
    def __init__(self, x, y)
        self.x = x
        self.y = y









class Node:
    point = None
    prevNode = None
    g_cost = -1
    cost = -1

    #constructor
    def __init__(self,nodePoint,endPoint,previousNode):
       self.point = nodePoint
       self.prevNode = previousNode
       self.calcCost(endPoint)

    def calcCost(self, endPoint):
        self.g_cost = self.prevNode.g_cost+1
        h_cost = abs(self.point.x-endPoint.x) + abs(self.point.y-endPoint.y)
        self.cost = self.g_cost + self.h_cost


    def createNewNodes(self, curNodes):
        dx = [0,1,0,-1]
        dy = [1,0,-1,0]
        newNodes = []
        for i in range(0,4):
            key = str(self.point.x+dx[i]) + "," + str(self.point.y+dy[i])
            if not (key in curNodes): 
                newNodes.append(Node(Point(point.x+dx[i],point.y+dy[i]), self.endPoint, self))
        return newNodes
