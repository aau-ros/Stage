import sys
import math
import random as rd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi

'''
settings
'''
# minimum distance in pixels between center docking station and next closest one
min_dist = 10

# for debugging
debug = False

'''
random helper function
'''
def random_n(low, high, n, onlyints=True):
    if onlyints: randfunc = rd.randrange
    else: randfunc = rd.uniform
    return list(( randfunc(low,high) for _ in xrange(n) ))

'''
check command line parameters
print usage
'''
if len(sys.argv) <= 5:
    print "Not enough arguments:"
    print sys.argv[0], "<seed> <map size> <num ds> <num robots> <connectivity>"
    exit()

# read map size
size = int(sys.argv[2])

'''
place docking stations
'''
# distance between center docking station and next closest one
dist = 0

# seed for docking station distribution
rd.seed(sys.argv[1])

# find distribution where center docking station has enough room
while(dist < min_dist):
    # generate distribution
    points = np.array([random_n(0,size,2) for _ in xrange(int(sys.argv[3]))])

    # find center most docking station
    d = size
    center = (size/2,size/2)
    for (x,y) in points:
        if(math.hypot(x-float(size)/2, y-float(size)/2) < d):
            d = math.hypot(x-float(size)/2, y-float(size)/2)
            center = (x,y)

    # find docking station closest to center docking station
    d = size
    closest = (0,0)
    for (x,y) in points:
        if (x,y) == center:
            continue
        if(math.hypot(x-center[0], y-center[1]) < d):
            d = math.hypot(x-center[0], y-center[1])
            closest = (x,y)

    # check distance of center docking station to next closest one
    dist = math.hypot(center[0]-closest[0], center[1]-closest[1])

# store coordinates in stage world file
if not debug:
    f = open("worlds/eae_dss.inc", "w")
    for i,(x,y) in enumerate(points):
        f.write("docking_station( pose [ " + str(x-float(size)/2) + " " + str(y-float(size)/2) + " 0 0 ] fiducial_return " + str(i+1) + " name \"ds" + str(i+1) + "\" )\n")
    f.close()

'''
place robots
all robots are placed on a unit circle around center most docking station
'''
if not debug:
    # read number of robots from command line parameter
    nr = int(sys.argv[4])

    # angle between robots
    angle = 2*math.pi / nr

    # place every robot
    f = open("worlds/eae_robots.inc", "w")
    for r in xrange(nr):
        # define position
        a = r * angle
        x = center[0] + math.cos(a) - float(size)/2
        y = center[1] + math.sin(a) - float(size)/2

        # store coordinates in stage world file
        f.write("robot( pose [ " + str(x) + " " + str(y) + " 0 " + str(a*180.0/math.pi) + " ] )\n")
    f.close()

'''
generate voronoi graph
'''
vor = Voronoi(points)

'''
output map
'''
# read connectivity from command line parameter
con = float(sys.argv[5])

# output docking stations
if debug:
    plt.plot(points[:, 0], points[:, 1], 'o')

# output edges of finite walls
for simplex in vor.ridge_vertices:
    simplex = np.asarray(simplex)
    if np.all(simplex >= 0):
        if(rd.random() >= con):
            plt.plot(vor.vertices[simplex, 0], vor.vertices[simplex, 1], 'k-')

# output edges of infinite walls
center = points.mean(axis=0)
for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
    simplex = np.asarray(simplex)
    if np.any(simplex < 0):
        if(rd.random() >= con):
            i = simplex[simplex >= 0][0] # finite end Voronoi vertex
            t = points[pointidx[1]] - points[pointidx[0]]  # tangent
            t = t / np.linalg.norm(t)
            n = np.array([-t[1], t[0]]) # normal
            midpoint = points[pointidx].mean(axis=0)
            far_point = vor.vertices[i] + np.sign(np.dot(midpoint - center, n)) * n * 200
            plt.plot([vor.vertices[i,0], far_point[0]], [vor.vertices[i,1], far_point[1]], 'k-')

# define layout of graph
plt.xlim(0, size)
plt.ylim(0, size)
fig = plt.gcf()
ax = plt.gca()
ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)
fig.set_size_inches(float(size)/100, float(size)/100)
plt.tight_layout(pad=0, w_pad=0, h_pad=0)

# display graph
if debug:
    plt.show()

# save graph on hd
else:
    plt.savefig("worlds/bitmaps/voronoi.png", dpi=100)