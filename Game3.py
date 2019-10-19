# Pathfinding - Part 1
# Graphs
# KidsCanCode 2017
import pygame as pyg
from collections import deque 
from queue import LifoQueue
import heapq
from math import pow 
from os import path
import matplotlib.pyplot as plt

vector = pyg.math.Vector2

TILE_SIZE = 25
TILE_WIDTH = 20
TILE_HEIGHT = 20
GRID_WIDTH = TILE_SIZE * TILE_WIDTH
GRID_HEIGHT = TILE_SIZE * TILE_HEIGHT
FPS = 30
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE=(0,0,255)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 0)
DARKGRAY = (40, 40, 40)
LIGHTGRAY = (255, 255, 255)
MEDGRAY = (75, 75, 75)
EXPLORED=(255, 204, 128)

pyg.init()
screen = pyg.display.set_mode((GRID_WIDTH, GRID_HEIGHT))
clock = pyg.time.Clock()
class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.connections = [vector(1, 0), vector(-1, 0), vector(0, 1), vector(0, -1), vector(-1, -1), vector(1, 1), vector(1, -1), vector(-1, 1)]

    def in_bounds(self, node):
        return 0 <= node.x < self.width and 0 <= node.y < self.height

    def passable(self, node):
        return node not in self.walls

    def find_neighbors(self, node):
        neighbors = [node + connection for connection in self.connections]
        neighbors = filter(self.in_bounds, neighbors)
        neighbors = filter(self.passable, neighbors)
        return neighbors

    def draw(self):
        for wall in self.walls:
            rect = pyg.Rect(wall * TILE_SIZE, (TILE_SIZE, TILE_SIZE))
            pyg.draw.rect(screen, BLACK, rect)
                  
class CostGrid(SquareGrid):
    def __init__(self,width,height):
        super().__init__(width,height)
        self.weights={}


    def cost(self,from_node,to_node):
        if((vector(to_node)-vector(from_node)).length_squared()==1):
            return self.weights.get(to_node,0)+10
        else:
            return self.weights.get(to_node,0)+14    

class PriorityHeapQueue:

    def __init__(self):
         self.nodes=[]

    def put(self,node,cost):
        heapq.heappush(self.nodes,(cost,node))

    def get(self):
        return heapq.heappop(self.nodes)[1]

    def empty(self):
        return len(self.nodes)==0    


def draw_icons():
    startnode_center = (endnode.x * TILE_SIZE + TILE_SIZE / 2, endnode.y * TILE_SIZE + TILE_SIZE / 2)
    screen.blit(home_img, home_img.get_rect(center=startnode_center))
    endnode_center = (startnode.x * TILE_SIZE + TILE_SIZE / 2, startnode.y * TILE_SIZE + TILE_SIZE / 2)
    screen.blit(cross_img, cross_img.get_rect(center=endnode_center))

def draw_grid():
    for x in range(0, GRID_WIDTH, TILE_SIZE):
        pyg.draw.line(screen, BLACK, (x, 0), (x, GRID_HEIGHT))
    for y in range(0, GRID_HEIGHT, TILE_SIZE):
        pyg.draw.line(screen, BLACK, (0, y), (GRID_WIDTH, y))

# vector is not hashable so cant be a key value
def vector_to_int(vectorA):
    return (int(vectorA.x),int(vectorA.y))


def heuristic(from_node,to_node):
    #manhattan distance
    return (abs(from_node.x-to_node.x)+abs(from_node.y-to_node.y))*10

def BFS(graph, startnode,endnode):
    q=deque()
    q.append(startnode)
    visitednodes=[]
    path={}
    path[vector_to_int(startnode)]=None

    visitednodes.append(startnode)

    while(len(q)>0):
        currentnode=q.popleft()
        if currentnode == endnode:
            break
        for nextnode in graph.find_neighbors(currentnode):
            #if(nextnode not in visitednodes):
            if(vector_to_int(nextnode) not in path):
                q.append(nextnode)
                visitednodes.append(nextnode)
                path[vector_to_int(nextnode)] = nextnode - currentnode
    print("********************VISITED***********************")
    print(visitednodes)
    print("************************PATH***********************")
    for key, value in path.items() :
        print (key)
    return path            

def Dijkstras(graph,startnode,endnode):
    q=PriorityHeapQueue()
    q.put(vector_to_int(startnode),0)
    path={}
    cost={}
    path[vector_to_int(startnode)]=None
    cost[vector_to_int(startnode)]=0

    while not q.empty():
        current=q.get()
        if vector(current)==endnode:
            break

        for next in graph.find_neighbors(vector(current)):
            next_cost=cost[current]+graph.cost(current,vector_to_int( next))
            if ((vector_to_int(next) not in cost) or (next_cost< cost[vector_to_int(next)])):
                cost[vector_to_int(next)]=next_cost
                priority=next_cost
                q.put(vector_to_int(next),priority)
                path[vector_to_int(next)]=vector(next)-vector(current)
    return path            

def AStar(graph,startnode,endnode):
    q=PriorityHeapQueue()
    q.put(vector_to_int(startnode),0)
    path={}
    cost={}
    path[vector_to_int(startnode)]=None
    cost[vector_to_int(startnode)]=0

    while not q.empty():
        current=q.get()
        if vector(current)==endnode:
            break

        for next in graph.find_neighbors(vector(current)):
            next_cost=cost[current]+graph.cost(current,vector_to_int( next))
            if ((vector_to_int(next) not in cost) or (next_cost< cost[vector_to_int(next)])):
                cost[vector_to_int(next)]=next_cost
                priority=heuristic(next,endnode)
                q.put(vector_to_int(next),priority)
                path[vector_to_int(next)]=vector(next)-vector(current)
    return path 

def DFS(graph, startnode,endnode):
    s=LifoQueue()
    s.put(startnode)
    visitednodes=[]
    path={}
    path[vector_to_int(startnode)]=None
    

    while(not s.empty()):
        currentnode=s.get()
        if currentnode == endnode:
            break
        visitednodes.append(startnode)    
        for nextnode in graph.find_neighbors(currentnode):
            #if(nextnode not in visitednodes):
            if(vector_to_int(nextnode) not in path):
                s.put(nextnode)
                visitednodes.append(nextnode)
                path[vector_to_int(nextnode)] = nextnode - currentnode

    print(visitednodes)
    print("*******************************************")
    print(path)
    return path

icon_dir = path.join(path.dirname(__file__), '')
home_img = pyg.image.load(path.join(icon_dir, 'd.png')).convert_alpha()
home_img = pyg.transform.scale(home_img, (30, 30))
home_img.fill((255, 255, 255, 255), special_flags=pyg.BLEND_RGBA_MULT)
cross_img = pyg.image.load(path.join(icon_dir, 's.png')).convert_alpha()
cross_img = pyg.transform.scale(cross_img, (30, 30))
cross_img.fill((255, 255, 255, 255), special_flags=pyg.BLEND_RGBA_MULT)
arrows = {}

arrow_img = pyg.image.load(path.join(icon_dir, 'arrow3.png')).convert_alpha()
arrow_img = pyg.transform.scale(arrow_img, (15, 15))
for dir in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
    arrows[dir]=pyg.transform.rotate(arrow_img,vector(dir).angle_to(vector(1,0)))

graph=CostGrid(TILE_WIDTH, TILE_HEIGHT)
walls = [(2, 1), (3, 1), (4, 1), (1, 1), (4, 2), (4, 3), (4, 4), (4, 5), (3, 5), (2, 5), (2, 3), (1, 3), (0, 3), (0, 4),(0, 6), (0, 5), 
        (0, 7), (1, 7), (2, 7), (2, 8), (2, 9), (2, 10), (2, 11), (0, 11), (0, 12), (0, 13), (1, 13), (2, 13), (3, 13), (4, 13), (5, 13), 
        (3, 8), (4, 8), (5, 8), (7, 8), (6, 8), (5, 15), (5, 16), (5, 17), (5, 18), (6, 16), (7, 16), (8, 16), (9, 16),(18, 13),
        (10, 16), (11, 16), (7, 10), (8, 10), (9, 10), (10, 10), (11, 10), (11, 9), (11, 8), (11, 7), (11, 6), (11, 5), (12, 5), (13, 5),
        (12, 10), (13, 10), (14, 10), (17, 10), (16, 10), (17, 9), (17, 8), (17, 7), (16, 7), (15, 7), (16, 6), (16, 5), (16, 4), (17, 4),
        (18, 4), (18, 2), (17, 2), (16, 2), (15, 2), (14, 2), (14, 1), (13, 1), (12, 1), (11, 1), (10, 1), (9, 1), (8, 1), (8, 2), (8, 3),
        (8, 4), (8, 5), (7, 4), (13, 11), (13, 13), (13, 14), (12, 13), (14, 13), (15, 13), (16, 13), (16, 14), (16, 15), (16, 16), 
        (18, 12), (19, 12), (8, 13), (9, 13), (9, 14), (9, 19), (11, 19), (12, 19), (10, 19), (13, 19), (0, 16), (1, 16), (2, 16), (2, 17), 
        (2, 18), (14, 17), (15, 17), (16, 17), (17, 17), (18, 18)]
for wall in walls:
    graph.walls.append(vector(wall))
   

startnode=vector(5,2)
endnode=vector(14,9)


path=AStar(graph,startnode,endnode)  
running = True
while running:
    clock.tick(FPS)
    for event in pyg.event.get():
        if event.type == pyg.QUIT:
            running = False
        if event.type == pyg.KEYDOWN:
            if event.key == pyg.K_ESCAPE:
                running = False
            if event.key == pyg.K_m:
                # dump the wall list for saving
                print([(int(loc.x), int(loc.y)) for loc in g.walls])
        if event.type == pyg.MOUSEBUTTONDOWN:
            mpos = vector(pyg.mouse.get_pos()) // TILE_SIZE
            if event.button == 1:
                if mpos in graph.walls:
                    graph.walls.remove(mpos)
                else:
                    graph.walls.append(mpos)

                startnode=mpos
                endnode=(4,4)
                path=AStar(graph,startnode,endnode)    
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                print(startnode)
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")

    pyg.display.set_caption("{:.2f}".format(clock.get_fps()))
    screen.fill(WHITE)
    # fill explored area
    for node in path:
        x, y = node
        rect = pyg.Rect(x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pyg.draw.rect(screen, EXPLORED, rect)
    draw_grid()
    graph.draw()
#    for node,dir in path.items():
#        if dir:
#            x,y=node
#            x=x*TILE_SIZE+TILE_SIZE/2
#             y=y*TILE_SIZE+TILE_SIZE/2
#            img=arrows[vector_to_int(dir)]
#            r=img.get_rect(center=(x,y))
#            screen.blit(img,r)

#draw path from start to goal
    currentnode = endnode
    while currentnode != startnode:
        x = currentnode.x * TILE_SIZE + TILE_SIZE / 2
        y = currentnode.y * TILE_SIZE + TILE_SIZE / 2
        img = arrows[vector_to_int(path[(currentnode.x, currentnode.y)])]
        r = img.get_rect(center=(x, y))
        screen.blit(img, r)
        # find next in path
        currentnode = currentnode - path[vector_to_int(currentnode)]
    draw_icons()    
    pyg.display.flip()

    fig = plt.figure(figsize=(4, 5))       # size in inches
# use plot(), etc. to create your plot.

# Pick one of the following lines to uncomment
# save_file = None
# save_file = os.path.join(your_directory, your_file_name)  

    #plt.savefig(pyg.display.flip())
    #plt.close(fig)    