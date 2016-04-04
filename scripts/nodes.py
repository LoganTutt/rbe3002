class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def key(self):
        return str(self.x) + "," + str(self.y)

    def equals(self,compPoint):
        return self.x == compPoint.y and self.y == compPoint.y








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
        if(self.prevNode != None):
            self.g_cost = self.prevNode.g_cost+1
        else:
            self.g_cost = 0
        h_cost = abs(self.point.x-endPoint.x) + abs(self.point.y-endPoint.y)
        self.cost = self.g_cost + self.h_cost


    def createNewNodes(self, curNodes, world, blockedThresh):
        dx = [0,1,0,-1]
        dy = [1,0,-1,0]
        newNodes = []
        for i in range(0,4):
            tempX = self.point.x+dx[i]
            tempY = self.point.y+dy[i]
            key = self.point.key()
            if (not ((key in curNodes) or tempX < 0 or tempX >= len(world) or tempY < 0 or tempY >= len(world[tempX]) or (world[tempX][tempY]) >= blockedThresh)):
                newNodes.append(Node(Point(tempX,tempY), self.endPoint, self))
        return newNodes
