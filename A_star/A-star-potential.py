from heapq import heappush, heappop
import numpy as np
import scipy.ndimage
import traceback
import gui
import common
import math

world_extents = (340,170)
world_obstacles = np.zeros(world_extents, dtype=np.uint8)
visited_nodes = None
show_visited_nodes = True
use_potential_function = True
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()
def toggle_visited_nodes():
    global show_visited_nodes
    show_visited_nodes = not show_visited_nodes
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def toggle_potential_function():
    global use_potential_function
    use_potential_function = not use_potential_function
    update_callback()
def apply_distance_transform():
    global world_obstacles
    if use_potential_function and np.max(world_obstacles) == 255:
        # Compute distance transform.
        dist_transform = 255-np.minimum(
            16*scipy.ndimage.morphology.distance_transform_edt(
            255-world_obstacles), 255)
        m = max(np.max(dist_transform), 1)  # Prevent m==0.
        world_obstacles = np.uint8((dist_transform * 255) / m)
    else:
        # Keep 255 values only (set all other to 0).
        world_obstacles = (world_obstacles == 255) * np.uint8(255)
def update_callback(pos = None):
    # First apply distance transform to world_obstacles.
    apply_distance_transform()
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = astar(start, goal, world_obstacles)
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)

# Each tuple is: (movement_x, movement_y, cost).
s2 = np.sqrt(2)
movements = [ # Direct neighbors (4N).
              (1,0, 1.), (0,1, 1.), (-1,0, 1.), (0,-1, 1.),
              # Diagonal neighbors.
              # Comment this out to play with 4N only (faster).
              (1,1, s2), (-1,1, s2), (-1,-1, s2), (1,-1, s2),
            ]

def distance(p, q):
    return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
def angle(p,q):
    if q[0]-p[0]==0:
        if q[1]-p[1]>0:
            return 90.0
        else:
            return 270.0        
    else:
        if q[1]-p[1]>=0 and q[0]-p[0]>=0:
            return math.degrees(math.atan((q[1]-p[1])/(q[0]-p[0])))
        elif q[1]-p[1]>0 and q[0]-p[0]<0:
            return 180+math.degrees(math.atan((q[1]-p[1])/(q[0]-p[0])))
        elif q[1]-p[1]<0 and q[0]-p[0]<0:
            return 180+math.degrees(math.atan((q[1]-p[1])/(q[0]-p[0])))
        elif q[1]-p[1]<0 and q[0]-p[0]>0:
            return 360+math.degrees(math.atan((q[1]-p[1])/(q[0]-p[0])))

def astar(start, goal, obstacles):
    """A* algorithm."""
    front = [ (distance(start,goal),0.001, start, None) ]
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)
    came_from = {}
    dis = {}
    ang= {}

    while front:
        element=heappop(front)
        total_cost, cost, pos, previous = element
        if visited[pos]>0:
            continue
        visited[pos]=cost
        came_from[pos]=previous
        if pos == goal:
            break
        for dx, dy, deltacost in movements:
            x,y=pos
            new_x= x+dx
            new_y= y+dy
            # Determine new position and check bounds.
            if not (new_x>=0 and new_x<extents[0]):
                continue

            if not (new_y>=0 and new_y<extents[1]):
                continue

            # Add to front if: not visited before and no obstacle.
            new_pos = (new_x, new_y)
            if (obstacles[new_x][new_y]==255):
                continue
            obstaclecost = obstacles[new_pos] / 64
            new_cost=cost+deltacost+obstaclecost   
            new_total_cost= new_cost+obstaclecost+distance(new_pos,goal)
            new_element = (new_total_cost,new_cost, new_pos,pos)
            heappush(front,new_element)
            
    path = []
    pathnew = []
    if pos == goal:  
        while pos:
            path.append(pos)
            pos = came_from[pos]
        path.reverse()
    for i in range((len(path)/2)+1):
        pathnew.append(path[i])
    #print came_from
    #print total_cost
    #print len(path)
    #print len(pathnew)
    #print pathnew
    outfile = file("trail1.txt", "w")
    for c in path:
        print >> outfile, "(%d,%d),\n" %c,
    print >> outfile
    outfile.close()
    '''[if visited == None :
        print 'no path'
    else:
        print 'distance = ',total_cost'''


    #FINDING THE TOTAL DISTANCE
    
    for x in  range(1,len(path)):
        dis[x]=distance(path[x-1],path[x])
        ang[x]= angle(path[x-1],path[x])
    D=dis.values()
    A=ang.values()
    
    e=A[0]
    for i in range(len(A)-1):

        if A[i+1]==e:
            A[i+1]=0

        else:
            if A[i+1]==0 and e!=0:
                A[i+1]=360-e
                e=0
            elif A[i+1]>e and e!=0:
                c=A[i+1]
                A[i+1]= A[i+1]-e
                e=c
            elif A[i+1]<e and e!=0:
                c=A[i+1]
                A[i+1] = 360-(e - A[i+1])
                e=c
            elif A[i+1]>e and e==0:
                e=A[i+1]

            #print e

    
    # CONSTANT MULTIPLICATION TO MAKE MOTOR ANGLES TO ROBOT ANGLES
    for i in range(len(D)):
        D[i]= int(math.floor(9.3*D[i]))
        A[i]= int(math.floor(3.6*A[i]))
    
    #print A
    #print D
    Af=[]
    W=A
    for i in range(len(W)):
        num = W[i]
        if len(str(num)) == 2:
            num = "100"+str(num);
            Af.append(num)
        elif len(str(num)) == 4:
            num = "1"+str(num)
            Af.append(num)
        elif len(str(num))==3:
            num = "10"+str(num);
            Af.append(num)
        #print num;
    #print 'newAf',Af;
    T=[]
    for x in range(len(A)):
        if A[x]!=0:
        #print 'x',x
            sum=D[x]
            for i in range(x,len(A)-1):
            #print 'i',i    
                        
                if(A[i+1]==0):
                    sum=sum+D[i+1];         
                elif A[i+1]!=0:
                    break
        #print sum
            T.append(sum)
        else:
            x=x+1
#print X    
    #print Tr
    Df=[]
    for i in range(len(T)):
        num = T[i]
        if len(str(num)) == 2:
            num = "300"+str(num);
            Df.append(num)
        elif len(str(num)) == 4:
            num = "3"+str(num)
            Df.append(num)
        elif len(str(num))==3:
            num = "30"+str(num);
            Df.append(num)
        #print num;
    #print 'newDf',Df;
    Arduinodata=[]
    for i in range(len(Df)):
        x=Af[i]
        y=Df[i]
        Arduinodata.append(x)
        Arduinodata.append(y)
    #print Arduinodata
    outfile = file("Arduinodata.txt", "w")
    for c in Arduinodata:
        print >> outfile, "%d,"%int(c),
    print >> outfile
    outfile.close()


    return (path, visited)
    H=came_from.items()
    


# Main program.
if __name__ == '__main__':
    # Link functions to buttons.
    callbacks = {"update": update_callback,
                 "button_1_press": add_obstacle,
                 "button_1_drag": add_obstacle,
                 "button_1_release": update_callback,
                 "button_2_press": remove_obstacle,
                 "button_2_drag": remove_obstacle,
                 "button_2_release": update_callback,
                 "button_3_press": remove_obstacle,
                 "button_3_drag": remove_obstacle,
                 "button_3_release": update_callback,
                 }
    # Extra buttons.
    buttons = [("Clear", clear_obstacles),
               ("Use Potential Function", toggle_potential_function),
               ("Show Visited", toggle_visited_nodes)]

    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "on", "A* Algorithm using potential function.")

    # Start GUI main loop.
    gui.run()
