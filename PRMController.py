import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors
import shapely.geometry

from Dijkstra import Graph, dijkstra, to_array
import pylab as pl
import environment_2d



class PRMController:
    def __init__(self, options):
        self.opt = options
        self.env1 = self.opt.env1
        self.env2 = self.opt.env2
        self.env3 = self.opt.env3
        self.env = environment_2d.Environment(self.env1, self.env2, self.env3)
        pl.clf()
        q = self.env.random_query()
        if q is not None:
            self.x_start, self.y_start, self.x_goal, self.y_goal = q
        self.numOfCoords = self.opt.nbrs
        self.current = (np.array((self.x_start,self.y_start))).reshape(1,2)
        self.destination = (np.array((self.x_goal, self.y_goal))).reshape(1,2)
        self.graph = Graph()
        self.k = self.opt.k
        self.solutionFound = False
        self.obs_coor = self.env.obs_coor
        self.shortcut = self.opt.shortcut
        self.edgePlot = self.opt.edgePlot
        self.pointPlot = self.opt.pointPlot
        self.solutionPlot = self.opt.solutionPlot

    def runPRM(self, initialRandomSeed):
        seed = initialRandomSeed
        # Keep resampling if no solution found
        while(not self.solutionFound):
            print("Trying with random seed {}".format(seed))
            # np.random.seed(seed)
            self.rng = np.random.default_rng(seed)
            # Generate n random samples called milestones
            self.genCoords(self.env1, self.env2, self.numOfCoords,self.pointPlot)
            self.env.plot()

            # Link each vertex to k nearest neighbours.
            self.findNearestNeighbour(self.k, self.edgePlot)

            # Search for shortest path from start to end node - Using Dijkstra's shortest path alg
            self.shortestPath(self.solutionPlot,self.shortcut)

            seed = np.random.randint(1, 100000)
            self.collisionFreePoints = np.array([])
            self.graph = Graph()
        plt.show(block = True)

    def genCoords(self,x_range,y_range,nbrs,pointPlot):
        self.collisionFreePoints = np.array([])
        x_list = x_range*self.rng.random([nbrs])
        y_list = y_range*self.rng.random([nbrs])
        for i in range(len(x_list)):
            if not(self.env.check_collision(x_list[i],y_list[i])):
                if(self.collisionFreePoints.size == 0):
                    self.collisionFreePoints = np.array((x_list[i],y_list[i]))
                else:
                    self.collisionFreePoints = np.vstack(
                        [self.collisionFreePoints, np.array((x_list[i],y_list[i]))])
        self.collisionFreePoints = np.concatenate(\
            (self.collisionFreePoints, self.current, self.destination), axis=0)
        if pointPlot == True:
            self.plotPoints(self.collisionFreePoints)

    def findNearestNeighbour(self, k, edgePlot):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=k)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)

        for i, p in enumerate(X):
            # Ignoring nearest neighbour - nearest neighbour is the point itself
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = p
                end_line = neighbour
                if(not self.checkLineCollision(start_line, end_line)):
                    self.collisionFreePaths = np.concatenate(
                        (self.collisionFreePaths, p.reshape(1, 2), neighbour.reshape(1, 2)), axis=0)

                    a = str(self.findNodeIndex(p))
                    b = str(self.findNodeIndex(neighbour))
                    self.graph.add_node(a)
                    self.graph.add_edge(a, b, distances[i, j+1])
                    x = [p[0], neighbour[0]]
                    y = [p[1], neighbour[1]]
                    if edgePlot == True:
                        plt.plot(x, y)

    def shortestPath(self, solutionPlot,shortcut):
        self.startNode = str(self.findNodeIndex(self.current))
        self.endNode = str(self.findNodeIndex(self.destination))

        dist, prev = dijkstra(self.graph, self.startNode)

        pathToEnd = to_array(prev, self.endNode)

        if(len(pathToEnd) > 1):
            self.solutionFound = True
        else:
            pl.clf()
            return

        # Plotting shorest path
        pointsToDisplay = [(self.findPointsFromNode(path))
                           for path in pathToEnd]

        x = [item[0] for item in pointsToDisplay]
        y = [item[1] for item in pointsToDisplay]
        self.env.plot_query(self.x_start, self.y_start, self.x_goal, self.y_goal)
        if shortcut == True:
            x,y = self.pathshortcut(x,y)
        if solutionPlot == True:
            plt.plot(x, y, c="blue", linewidth=3.5)

        print("--done--")

    def checkLineCollision(self, start_line, end_line):
        line = shapely.geometry.LineString([start_line,end_line])
        for coor in self.obs_coor:
            obs = shapely.geometry.Polygon(coor)
            if line.intersects(obs):
                return True
        return False

    def findNodeIndex(self, p):
        return np.where((self.collisionFreePoints == p).all(axis=1))[0][0]

    def findPointsFromNode(self, n):
        return self.collisionFreePoints[int(n)]

    def plotPoints(self, points):
        x = [item[0] for item in points]
        y = [item[1] for item in points]
        plt.scatter(x, y, c="black", s=1)

    def pathshortcut(self,x,y):
        x_new = x.copy()
        y_new = y.copy()
        count = 0
        iter = 0
        while iter<5:
            for i in range(0,len(x)-2,2):
                #xy
                x1 = x[i]
                x2 = x[i+1]
                x3 = x[i+2]
                y1 = y[i]
                y2 = y[i+1]
                y3 = y[i+2]
                #line1
                xl1 = [x1,x2]
                yl1 = [y1,y2]
                c1 = np.polyfit(xl1,yl1,1)
                l1 = np.poly1d(c1)
                #line2
                xl2 = [x2,x3]
                yl2 = [y2,y3]
                c2 = np.polyfit(xl2,yl2,1)
                l2 = np.poly1d(c2)
                
                #generate random x
                randomx1 = self.generate_random_x(x1,x2)
                randomx2 = self.generate_random_x(x2,x3)
                randomy1 = l1(randomx1)
                randomy2 = l2(randomx2)
                if not(self.checkLineCollision((randomx1,randomy1),(randomx2,randomy2))):
                    x_new[i+count+1] = randomx1
                    y_new[i+count+1] = randomy1
                    x_new.insert(i+count+2,randomx2)
                    y_new.insert(i+count+2,randomy2)
                    count += 1
            x = x_new.copy()
            y = y_new.copy()
            count = 0
            iter += 1
        return x_new,y_new

    def generate_random_x(self,n1,n2):
        if n1>n2:
            randomx = np.random.uniform(n1,n2)
            return randomx
        else:
            randomx = np.random.uniform(n2,n1)
            return randomx
