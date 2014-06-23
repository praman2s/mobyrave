#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/mobyrave')
try:
    env=Environment()
    env.Load('scenes/youbot1.env.xml')
    env.SetViewer("qtcoin")
    moby = RaveCreatePhysicsEngine(env,'moby')
    moby.SetGravity([0,0,-9.8])
    print moby.GetGravity()
    raw_input("Check for visualization")
    
finally:
    RaveDestroy()
