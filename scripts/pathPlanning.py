import numpy as np
import cv2

class Pos():
    def __init__(self, v, h, parent = None):
        self.v = v
        self.h = h
        self.fValue = 0
        self.gValue = 0
        self.hValue = 0
        self.parent = parent
        self.visited = 0

    def __add__(self, o):
        return Pos(self.v + o.v, self.h + o.h)

    def __sub__(self, o):
        return Pos(self.v - o.v, self.h - o.h)

    def __mul__(self, o):
        return Pos(self.v*o.v, self.h*o.h)

    def __str__(self):
        return str(self.v) + ', ' + str(self.h)

    def __eq__(self, o):
        if self.v == o.v and self.h == o.h:
            return True
        else:
            return False

    def dist(self, o):
        return ((self.v-o.v)**2 + (self.h-o.h)**2)**0.5

    def cross(self, o):
        return (self.v*o.h - self.h*o.v)

class PathFinder():
    def __init__(self, startPos, destPos, mapImg, canvas, j, pixelSpan = 800):
        self.currPos = startPos
        self.startPos = startPos
        self.mapImg = mapImg
        self.destPos = destPos
        self.canvas = canvas
        self.trajectory = []
        self.visited = np.zeros((800,800))
        self.pathCost = 0
        self.refNeighbours = []
        for m in [0, -j, j]:
            for n in [0, -j, j]:
                if m != 0 or n != 0:
                    self.refNeighbours.append(Pos(m,n))

    # def nodeState(self, pos):
    #     return 255 - self.mapImg[pos.v][pos.h]
    #     if self.mapImg[pos.v][pos.h] > 170:
    #         return 'white'
    #     elif self.mapImg[pos.v][pos.h] < 100:
    #         return 'black'
    #     else:
    #         return 'gray'

    def hValue(self, pos):
        normFactor = max(abs(self.startPos.v-self.destPos.v), abs(self.startPos.h-self.destPos.h))
        return max(abs(pos.v-self.destPos.v), abs(pos.h-self.destPos.h))/normFactor

    def gValue(self, pos):
        return (255 - self.mapImg[pos.v][pos.h])/255 # if self.mapImg[pos.v][pos.h] > 0 else 100000

    def getLeastFPos(self, kg = 1, kh = 1):
        neighbours = [self.currPos + x for x in self.refNeighbours]
        minFValue = np.inf
        nextNode = neighbours[0]
        for node in neighbours:
            if self.visited[node.v][node.h] == 0:
                node.gValue = self.gValue(node)
                node.hValue = self.hValue(node)
                node.fValue = kg*node.gValue + kh*node.hValue
                if node.fValue < minFValue:
                    minFValue = node.fValue
                    nextNode = node

        self.pathCost += minFValue
        return nextNode

    def aStar(self, record = 0, kg = 1, kh = 1):
        while self.currPos != self.destPos: #and self.nodeState(self.currPos) != 'gray':
            self.visited[self.currPos.v][self.currPos.h] = 1
            if record == 1:
                # cv2.circle(self.canvas, (self.currPos.h, self.currPos.v), 1, (0, 0, 255), 1)
                self.trajectory.append(self.currPos)
            leastFPos = self.getLeastFPos(kg, kh)
            self.currPos = leastFPos
        self.trajectory.append(self.currPos)

    def shortestPath(self):
        self.aStar(0, 1.1, 0.9)

        temp = self.startPos
        self.startPos = self.destPos
        self.destPos = temp
        self.visited = 1-self.visited

        self.pathCost = 0
        self.aStar(1, 0, 1)
        self.trajectory = self.trajectory[::-1]

        return self.trajectory, self.pathCost

def prepareMap(img):
    canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    newImg = np.array(img)
    newImg[newImg < 127] = 0
    newImg[newImg >= 127] = 255
    newImg = cv2.erode(newImg, np.ones((50,50)))
    newImg = cv2.dilate(newImg, np.ones((30,30)))

    final = np.array(newImg)

    kernel = np.ones((50,50),np.float32)/2500
    final = cv2.filter2D(newImg,-1,kernel)

    final[img == 127] = 127
    final[newImg == 0] = 0

    # cv2.imshow('win', final)
    # cv2.waitKey(1)

    return final, canvas

if __name__ == '__main__':
    img = cv2.imread('../../generatedMap.png', 0)
    newImg, canvas = prepareMap(img)

    startPos = Pos(400, 400)
    destPos = Pos(650, 400)
    finder = PathFinder(startPos, destPos, newImg, canvas, 1)
    finder.shortestPath()
    cv2.imshow('win', finder.canvas)
    cv2.waitKey(0)
