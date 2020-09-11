# -*- coding: utf-8 -*-
"""
Created on Wed Apr  8 21:28:23 2020

@author: boein
"""

from RobotRaconteur.Client import *
import time

#RRN is imported from RobotRaconteur.Client
#Connect to the service.
obj = RRN.ConnectService('rr+tcp://localhost:9999/?service=Create')
# obj = RRN.ConnectService('rr+tcp://localhost:8338/RRParallel/parallelTestServ')

obj.StartStreaming()
#The "Create" object reference is now available for use
#Drive for a bit
# obj.run()
# time.sleep(1)

while True:
    print ('Result: ',obj.read)
    if obj.read==0:
        break