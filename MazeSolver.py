from matplotlib import pyplot as plt
import cv2
import math
import heapq

 
def fixPixelValues(px):
    # convert the RGB values into floating point to avoid an overflow
    return [ float(px[0]), float(px[1]), float(px[2]) ]

## Given (x,y) coordinates of two neighboring pixels, calculate the edge weight.
## Uses squared euclidean distance between the pixel values and adds 0.1
def getEdgeWeight(img, u, v):
    # get edge weight for edge between u, v
    # First make sure that the edge is legit
    i0,j0 = u[0], u[1]
    i1,j1 = v[0], v[1]
    height, width, _ = img.shape
    assert i0 >= 0 and j0 >= 0 and i0 < width and j0 < height # pixel position valid?
    assert i1 >= 0 and j1 >= 0 and i1 < width and j1 < height # pixel position valid?
    assert -1 <= i0 - i1 <= 1 # edge between node and neighbor?
    assert -1 <= j0 - j1 <= 1
    px1 = fixPixelValues(img[j0,i0])
    px2 = fixPixelValues(img[j1,i1])
    return 0.1 + (px1[0] - px2[0])**2 + (px1[1] - px2[1])**2 + (px1[2]- px2[2])**2
    
# Function that takes in img, the maze, and path, an array of coordinates to follow 
# Draws a series of red lines between each coordinate and next to 
# show the path in the image. pThick is line thickness.
def drawPath(img, path, pThick=2):
    v = path[0]
    x0, y0 = v[0], v[1]
    for v in path:
        x, y = v[0], v[1]
        cv2.line(img,(x,y), (x0,y0), (255,0,0),pThick)
        x0, y0 = x,y

# Vertex data structure used later in computeShortestPath
class Vertex: 
    def __init__ (self,  i, j):
        self.x = i # The x coordinate
        self.y = j  # The y coordinate
        self.d = float('inf') # the shortest path estimate
        self.processed = False # Has this vertex's final shortest path distance been computed
        self.idx_in_priority_queue = -1 # The index of this vertex in the queue
        self.pi = None # the parent vertex.
    
    def __lt__(self, other):
        return self.x < other.x


## Priority Queue used later in computeShortestPath
class PriorityQueue:
    def __init__(self):
        self.heap = []

    def insert(self, priority, v):
        heapq.heappush(self.heap, (priority, v))

    def get(self):
        return heapq.heappop(self.heap)[-1]


"""
Function to find shortest path through an img maze

Input: img, read in through cv2 library
       source, starting x, y coordinates
       dest, ending x, y coordinates

Output: Array of coordinates to traverse to travel the shortest path from 
        source to dest
"""
def computeShortestPath(img, source, dest): 
    q = PriorityQueue()
    start = Vertex(source[0], source[1])
    start.d = 0
    nodes = {(0, 0):start}
    q.insert(start.d, start) 
    currentvert = start
    size = img.shape
    
    ## loop until we reach the destination
    while currentvert.x != dest[0] or currentvert.y != dest[1]:
        
        currentvert = q.get()
        # go to next thing in queue if this vertex has already been seen
        if (currentvert.x, currentvert.y) in nodes:
            continue
            
        ## check all 4 possible adjacent coordinates and create a new vertex for any that dont exist already
        
        # make sure x-1 is valid
        if currentvert.x - 1 >= 0 and not (currentvert.x-1, currentvert.y) in nodes: 
                newvert = Vertex(currentvert.x-1, currentvert.y)
                
                #relax edge
                if currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y)) < newvert.d:
                    newvert.d = currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y))
                    newvert.pi = currentvert
                q.insert(newvert.d, newvert)

        # make sure y-1 is valid
        if currentvert.y - 1 >= 0 and not (currentvert.x, currentvert.y-1) in nodes: 
                newvert = Vertex(currentvert.x, currentvert.y-1)
                
                #relax edge
                if currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y)) < newvert.d:
                    newvert.d = currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y))
                    newvert.pi = currentvert
                q.insert(newvert.d, newvert)
                
        # make sure x+1 is valid
        if currentvert.x + 1 < size[1] and not (currentvert.x+1, currentvert.y) in nodes: 
                newvert = Vertex(currentvert.x+1, currentvert.y)
                
                #relax edge
                if currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y)) < newvert.d:
                    newvert.d = currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y))  
                    newvert.pi = currentvert 
                q.insert(newvert.d, newvert)
        
        # make sure y+1 is valid
        if currentvert.y + 1 < size[0] and not (currentvert.x, currentvert.y+1) in nodes: 
                newvert = Vertex(currentvert.x, currentvert.y+1)
                
                #relax edge
                if currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y)) < newvert.d:
                    newvert.d = currentvert.d + getEdgeWeight(img, (currentvert.x,currentvert.y), (newvert.x, newvert.y))
                    newvert.pi = currentvert
                q.insert(newvert.d, newvert)
            
        currentvert.processed = True
        nodes[(currentvert.x, currentvert.y)] = currentvert
        if currentvert.x == dest[0] and currentvert.y == dest[1]:
            destvert = Vertex(currentvert.x, currentvert.y)
            destvert.pi = currentvert.pi
            destvert.d = currentvert.d
            break
            
    # once we have reached the destination, follow all vertices and add to array
    # to create the path to follow        
    path = [(destvert.x, destvert.y)]
    while destvert.pi != None:
        destvert = destvert.pi
        path.append((destvert.x,destvert.y))
    del nodes
    return path