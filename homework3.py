import queue

class SquareGrid:
    def __init__(self, W, H, Z, startpoint):
        self.width = W
        self.height = H
        self.weights = {}
        self.movements = [(1, 0), (-1, 0), (0, 1),(0, -1), (-1, -1), (1, 1),(1, -1), (-1, 1)]
        self.Z=Z
        self.startpoint=startpoint

    # CHECK IF THE ROVER ISWITHIN THE GRID
    def WithinGrid(self, point):
        return 0 <= point[0] < self.width and 0 <= point[1] < self.height

    # CHECK THE VALID HOP POSITION WITHIN Z ELEVATION
    def WithinRange(self, current, neighbor_points):
        # logic for Z elevation distance
        neighs = []
        for point in neighbor_points:
            if (abs(self.weights[(current[0],current[1])] - self.weights[(point[0],point[1])]) <= self.Z):
                neighs.append(point)
        return neighs

    def find_neighbors(self, current):
        # FINDING ALL NEIGHBORS
        #neighbors = [current + move for move in self.movements]
        neighbors=[]

        for move in self.movements:
            neighbors.append((current[0]+move[0],current[1]+move[1]))
        # CHECK IF DATA POINT IS VALID NEIGHBOR WITHIN GRID RANGE
        neighbors = filter(self.WithinGrid, neighbors)
        # CHECK IF NEIGHBOR IS WITHIN Z- DISTANCE
        neighbors = self.WithinRange(current, neighbors)  # all within Z-distance neighbors
        return neighbors

    def HopCost(self, from_point, to_point):

        x_diff=abs(to_point[0]-from_point[0])
        y_diff=abs(to_point[1]-from_point[1])
        if x_diff == 0 or y_diff == 0:
            return 10
        else:
            return 14

    def costWeight(self, from_point, to_point):
        Weight_Diff = abs(self.weights[(to_point[0],to_point[1])] - self.weights[(from_point[0],from_point[1])])
        return Weight_Diff




    # vector is not hashable so cant be a key value

    def heuristic(self,from_point, to_point):
        # manhattan distance
        return (abs(from_point[0] - to_point[0]) + abs(from_point[1] - to_point[1])) * 10

    def AStar(self,endpoint):
        # print("INSIDE a STAR")
        q = PriorityHeapQueue()
        q.put(self.startpoint, 0)
        path = {}
        cost = {}
        path[self.startpoint] = None
        cost[self.startpoint] = 0
        COUNT = 0
        while not q.empty():
            current = q.get()
            # print("##########################################################################################")
            # print("poped node",current," poped weight: ",graph.weights[(current[0],current[1])],"popped total cost:",cost[current])
            COUNT += 1
            if current == endpoint:
                break

            for next in self.find_neighbors(current):
                # print("inside A star current  :",current)
                # print("inside A star neighbor :",next)
                # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                # print("next :",next)
                next_cost = cost[current] + self.HopCost(current, next) + self.costWeight(current, next)
                # print("cost[current] :",cost[current])
                # print("graph.cost(current, vector_to_int(next)) :",graph.HopCost(current, next))
                # print("graph.costWeight(current,vector_to_int(next)) :",graph.costWeight(current,next))
                if ((next not in cost) or (next_cost < cost[next])):
                    cost[next] = next_cost
                    heuristic_cost = self.heuristic(next, endpoint)
                    # print("heuristic(next, endnode)",heuristic_cost)
                    priority = next_cost + heuristic_cost
                    if (next not in path.keys() or next == endpoint):
                        q.put(next, priority)
                        # path[next] = next - current
                    path[next] = current

        return path

    def UCS(self,endpoint):
        # QUEUE DECLARATION
        q = PriorityHeapQueue()
        # APPEND START NODE TO EMPTY QUEUE
        q.put(self.startpoint, 0)
        path = {}
        cost = {}
        path[self.startpoint] = None
        cost[self.startpoint] = 0
        visitednodes = []
        visitednodes.append(self.startpoint)

        while not q.empty():
            current = q.get()
            if current == endpoint:
                break
            for next in self.find_neighbors(current):
                HopCost = self.HopCost(current, next)
                next_cost = cost[current] + HopCost
                if next not in path or next_cost < cost[next]:
                    cost[next] = next_cost
                    priority = next_cost
                    visitednodes.append(next)
                    path[next] = current
                    q.put(next, priority)
        return path

    def BFS(self,endpoint):
        # QUEUE DECLARATION
        q = queue.Queue()
        # APPEND START NODE TO EMPTY QUEUE
        q.put(self.startpoint)
        visitednodes = []
        path = {}
        cost = {}
        path[self.startpoint] = None
        visitednodes.append(self.startpoint)
        cost[self.startpoint] = 0
        while (q.empty() == False):
            current = q.get()
            if current == endpoint:
                break
            for next in self.find_neighbors(current):
                # if(nextnode not in visitednodes):
                #next_cost = cost[current] + self.HopCost(current, next)
                if (next not in path):
                    #cost[next] = next_cost
                    q.put(next)
                    visitednodes.append(next)
                    path[next] = current
        return path

class PriorityHeapQueue:
    def __init__(self):
        self.points =queue.PriorityQueue()

    def put(self, point, cost):
        self.points.put((cost, point))
        #heapq.heappush(self.points, (cost, point))

    def get(self):
        return self.points.get()[1]
        #return heapq.heappop(self.points)[1]

    def empty(self):
        return self.points.empty()
        #return len(self.points) == 0

filename = "input.txt"
with open(filename) as f:
    content = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
content = [x.strip() for x in content]
algo = content[0]
matsize = content[1]
landsite = content[2]
Z = content[3]
targetcount = content[4]
target = []
for i in content[5:5 + int(targetcount)]:
    target.append(i)
Grid = content[5 + int(targetcount):]
GrdNew = []
for i in Grid:
    l = []
    i = i.split()
    # print(i)
    for k in i:
        if (k != " "):
            l.append(int(k))
    GrdNew.append(l)
    l = []
matsize = matsize.split()
TILE_WIDTH = int(matsize[0])
TILE_HEIGHT = int(matsize[1])

Output = open("output.txt", "w")
landsite=landsite.split()
start = int(landsite[0]), int(landsite[1])

graph = SquareGrid(TILE_WIDTH, TILE_HEIGHT,int(Z),start)
GridPoints = []

for i in range(len(GrdNew[0])):
    for j in range(len(GrdNew)):
        GridPoints.append((i, j))

for point in GridPoints:
    graph.weights[point] = GrdNew[point[1]][point[0]]
OutputList=""
for i in target:

    try:
        i = i.split()
        end = int(i[0]), int(i[1])
    except ValueError:
        OutputList+="FAIL\n"
        #Output.write("FAIL\n")
        continue

    if (algo == 'BFS'):
        path =graph.BFS(end)
    elif (algo == 'A*'):
        path = graph.AStar(end)
    elif (algo == 'UCS'):
        path = graph.UCS(end)
    else:
        OutputList+="FAIL\n"
        #Output.write("FAIL\n")

    # FINDING Path
    if (len(path) == 0 or end not in path):
        OutputList+="FAIL\n"
        #Output.write("FAIL\n")

    else:
        current = end
        s = queue.LifoQueue()
        s.put(current)
        while current != start:
            x = current[0]
            y = current[1]
            # find next in path
            current = path[current]
            s.put(current)
        while (not s.empty()):
            cnode = s.get()
            OutputList+=str(int(cnode[0])) + "," + str(int(cnode[1])) + " "
            #Output.write(str(int(cnode[0])) + "," + str(int(cnode[1])) + " ")
        OutputList+="\n"
        #Output.write("\n")

Output.write(OutputList)