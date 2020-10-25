import numpy as np
import matplotlib.pyplot as plt
import random
import time
import threading
import sys
from matplotlib.patches import Circle, PathPatch, Rectangle
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from networktables import NetworkTables
import mpl_toolkits.mplot3d.art3d as art3d

sys.setrecursionlimit(5000)

cond = threading.Condition()
notified = [False]
def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

    
NetworkTables.initialize(server='127.0.0.1')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

shuffleboard = NetworkTables.getTable('Shuffleboard')
tab = shuffleboard.getSubTable('Test3DGrapher')

plt.ion()
fig = plt.figure()
ax = Axes3D(fig)

def connect():
    with cond:
        print("Looking for Shuffleboard on Network")
        if not notified[0]:
            cond.wait()
    time.sleep(4)

def gatherData():
    global x,y,z,t,xlim,ylim,zlim
    while(True):
        while(NetworkTables.isConnected()):
            vx = tab.getValue('vx', 0)
            vy = tab.getValue('vy', 0)
            vz = tab.getValue('vz', 0)
            T = tab.getValue('T', 0)

            t = np.linspace(0, T, 100)
            x = vx * t
            y = vy * t
            z = vz * t - 9.8 * t * t

            xlim = vx * T
            ylim = vy * T
            zlim = vz * T - 9.8 * T * T
            plotstuff()
        connect()

def plotstuff():
    global xlim,ylim
    ax.clear()
    ax.plot(x, y, z,'g', label='Ball Path')
    
    c = Circle((ylim,zlim),0.5)
    s = Rectangle((ylim,zlim - 0.16),0.25,0.25,45)
    s.set_color('r')
    s.set_alpha(0.5)
    c.set_color('b')
    c.set_alpha(0.4)
    ax.add_patch(c)
    ax.add_patch(s)
    art3d.pathpatch_2d_to_3d(c, z=xlim, zdir = "x")
    art3d.pathpatch_2d_to_3d(s, z=xlim, zdir = "x")
    ax.set_aspect('auto')
    if(ylim == 0 or xlim == 0):
        xlim = 5
        ax.set_zlim3d(0,5)

    if xlim > ylim:
        ylim = xlim
    else:
        xlim = ylim

    

    ax.set_xlim3d(0, xlim)
    ax.set_ylim3d(-ylim, ylim)
    plt.legend()
    plt.show()
    plt.pause(.02)

gatherData()


