# -*- coding: utf-8 -*-
"""
Created on Tue May 22 10:16:51 2018

@author: YOGHES WARAN
"""

import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting

#Pre-Allocation

PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print 'Connected to remote API server'
    
else:
    print 'Connection not successful'
    sys.exit('Could not connect')


#retrieve motor  handles



sensor_h1=[] #empty list for handles
sensor_h2=[]
sensor_h3=[]
sensor_h4=[]
sensor_h5=[]

sensor_val1=np.array([]) #empty array for sensor measurements
sensor_val2=np.array([])
sensor_val3=np.array([])
sensor_val4=np.array([])
sensor_val5=np.array([])

#orientation of all the sensors: 
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

desti1= np.array([0,0])
desti2= np.array([3,-3])
desti3= np.array([3,3])
desti4= np.array([-3,3])
desti5= np.array([-3,-3])


errorCode,p3dx1=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
errorCode,p3dx2=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#0',vrep.simx_opmode_oneshot_wait)
errorCode,p3dx3=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#1',vrep.simx_opmode_oneshot_wait)
errorCode,p3dx4=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#2',vrep.simx_opmode_oneshot_wait)
errorCode,p3dx5=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#3',vrep.simx_opmode_oneshot_wait)


errorCode,left_motor_handle1=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle1=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)

errorCode,left_motor_handle2=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#0',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle2=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#0',vrep.simx_opmode_oneshot_wait)

errorCode,left_motor_handle3=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#1',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle3=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#1',vrep.simx_opmode_oneshot_wait)

errorCode,left_motor_handle4=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#2',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle4=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#2',vrep.simx_opmode_oneshot_wait)

errorCode,left_motor_handle5=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#3',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle5=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#3',vrep.simx_opmode_oneshot_wait)

for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h1.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val1=np.append(sensor_val1,np.linalg.norm(detectedPoint)) #get list of values
        
for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x)+'#0',vrep.simx_opmode_oneshot_wait)
        sensor_h2.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val2=np.append(sensor_val2,np.linalg.norm(detectedPoint)) #get list of values

for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x)+'#1',vrep.simx_opmode_oneshot_wait)
        sensor_h3.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val3=np.append(sensor_val3,np.linalg.norm(detectedPoint)) #get list of values

for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x)+'#2',vrep.simx_opmode_oneshot_wait)
        sensor_h4.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val4=np.append(sensor_val4,np.linalg.norm(detectedPoint)) #get list of values

for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x)+'#3',vrep.simx_opmode_oneshot_wait)
        sensor_h5.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val5=np.append(sensor_val5,np.linalg.norm(detectedPoint)) #get list of values

errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

t = time.time()


while (time.time()-t)<180:
   
   sensor_val1=np.array([]) #empty array for sensor measurements
   sensor_val2=np.array([])
   sensor_val3=np.array([])
   sensor_val4=np.array([])
   sensor_val5=np.array([])    
   
   for x in range(1,16+1):
       errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h1[x-1],vrep.simx_opmode_buffer)                
       sensor_val1=np.append(sensor_val1,np.linalg.norm(detectedPoint))
   
   for x in range(1,16+1):
       errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h2[x-1],vrep.simx_opmode_buffer)                
       sensor_val2=np.append(sensor_val2,np.linalg.norm(detectedPoint))
   
   for x in range(1,16+1):
       errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h3[x-1],vrep.simx_opmode_buffer)                
       sensor_val3=np.append(sensor_val3,np.linalg.norm(detectedPoint))
   
   for x in range(1,16+1):
       errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h4[x-1],vrep.simx_opmode_buffer)                
       sensor_val4=np.append(sensor_val4,np.linalg.norm(detectedPoint))
   
   for x in range(1,16+1):
       errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h5[x-1],vrep.simx_opmode_buffer)                
       sensor_val5=np.append(sensor_val5,np.linalg.norm(detectedPoint))

   errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
   errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
   errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
   errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
   errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
   
   pos1=pos1[0:2]   
   pos2=pos2[0:2]
   pos3=pos3[0:2]   
   pos4=pos4[0:2]   
   pos5=pos5[0:2]   
   
   pos1=np.array(pos1)   
   pos2=np.array(pos2)
   pos3=np.array(pos3)
   pos4=np.array(pos4)
   pos5=np.array(pos5)
   
   d12=np.linalg.norm(pos1-pos2)   
   d13=np.linalg.norm(pos1-pos3)
   d14=np.linalg.norm(pos1-pos4)
   d15=np.linalg.norm(pos1-pos5)
   
   d23=np.linalg.norm(pos2-pos3)
   d24=np.linalg.norm(pos2-pos4)
   d25=np.linalg.norm(pos2-pos5)
   
   d34=np.linalg.norm(pos4-pos3)
   d35=np.linalg.norm(pos5-pos3)
   
   d45=np.linalg.norm(pos4-pos5)
    
   p1x=pos1[0]
   p1y=pos1[1]
  
   p2x=pos2[0]
   p2y=pos2[1]
   
   p3x=pos3[0]
   p3y=pos3[1]
  
   p4x=pos4[0]
   p4y=pos4[1]
   
   p5x=pos5[0]
   p5y=pos5[1] 
   
   w1=np.array([0.05,0.05,0.05,0.05])
   w2=np.array([0.05,0.05,0.05])
   w3=np.array([0.05,0.05])
   w4=np.array([0.05])
   
   ppos1=pos1+0.1*(w1[0]*d12*(pos2-pos1)+w1[1]*d13*(pos3-pos1)+w1[2]*d14*(pos4-pos1)+w1[3]*d15*(pos5-pos1))
   ppos2=pos2+0.1*(w1[0]*d12*(pos1-pos2)+w2[0]*d23*(pos3-pos2)+w2[1]*d24*(pos4-pos2)+w2[2]*d25*(pos5-pos2))
   ppos3=pos3+0.1*(w1[1]*d13*(pos1-pos3)+w2[0]*d23*(pos2-pos3)+w3[0]*d34*(pos4-pos3)+w3[1]*d35*(pos5-pos3))
   ppos4=pos4+0.1*(w1[2]*d14*(pos1-pos4)+w2[1]*d24*(pos2-pos4)+w3[0]*d34*(pos3-pos4)+w4[0]*d45*(pos5-pos4))
   ppos5=pos5+0.1*(w1[3]*d15*(pos1-pos5)+w2[2]*d25*(pos2-pos5)+w3[1]*d35*(pos3-pos5)+w4[0]*d45*(pos4-pos5))   
   
   pos1=ppos1
   pos2=ppos2
   pos3=ppos3
   pos4=ppos4
   pos5=ppos5
   
   theta1=np.arctan2(pos1[1]-p1y,pos1[0]-p1x)
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.8*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)

   theta1=np.arctan2(pos2[1]-p2y,pos2[0]-p2x)
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.8*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)
   
   theta1=np.arctan2(pos3[1]-p3y,pos3[0]-p3x)
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.8*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)
  
   theta1=np.arctan2(pos4[1]-p4y,pos4[0]-p4x)
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.8*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)

   theta1=np.arctan2(pos5[1]-p5y,pos5[0]-p5x)
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.8*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)

   









