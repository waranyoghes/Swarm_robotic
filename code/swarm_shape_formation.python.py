# -*- coding: utf-8 -*-
"""
Created on Sat May 26 11:05:45 2018

@author: YOGHES WARAN
"""

import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting
from munkres import Munkres, print_matrix
#Pre-Allocation

m = Munkres()
PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')



def insertionsort(arr,ind):
 
    # Traverse through 1 to len(arr)
    for i in range(1, len(arr)):
 
        key = arr[i]
        key1=ind[i]
        # Move elements of arr[0..i-1], that are
        # greater than key, to one position ahead
        # of their current position
        j = i-1
        while j >=0 and key < arr[j] :
                arr[j+1] = arr[j]
                ind[j+1]=ind[j]
                j -= 1
        arr[j+1] = key
        ind[j+1]=key1


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


desti=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[0,0]])

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


t = time.time()

a=1
b=1
c=1
d=1
e=1

while (a==1 or b==1 or c==1 or d==1 or e==1):

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

  pos=np.matrix([pos1,pos2,pos3,pos4,pos5])
  matrix=np.zeros(shape=(5,5))

  for i in range(0,5):
    for j in range(0,5):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


    
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

  pos=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  if (abs(desti[indexes[0][1]]-pos[0])>0.05).all():  
   
   theta1=np.arctan2(desti.item(indexes[0][1],1)-pos.item(0,1),desti.item(indexes[0][1],0)-pos.item(0,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)   
   a=1
   
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,0, vrep.simx_opmode_streaming)       
    a=0
  
  if (abs(desti[indexes[1][1]]-pos[1])>0.05).all():  
   
   theta1=np.arctan2(desti.item(indexes[1][1],1)-pos.item(1,1),desti.item(indexes[1][1],0)-pos.item(1,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)   
   b=1
   
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,0, vrep.simx_opmode_streaming)       
    b=0
  
  if (abs(desti[indexes[2][1]]-pos[2])>0.05).all():  
   
   theta1=np.arctan2(desti.item(indexes[2][1],1)-pos.item(2,1),desti.item(indexes[2][1],0)-pos.item(2,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)   
   c=1   
   
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,0, vrep.simx_opmode_streaming)       
    c=0
  
  if (abs(desti[indexes[3][1]]-pos[3])>0.05).all():  
   
   theta1=np.arctan2(desti.item(indexes[3][1],1)-pos.item(3,1),desti.item(indexes[3][1],0)-pos.item(3,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]
   theta=theta1-theta2        
   r=0.0925
   d=0.38
   v=0.1
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)   
   d=1   
   
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,0, vrep.simx_opmode_streaming)       
    d=0
  
  if (abs(desti[indexes[4][1]]-pos[4])>0.05).all():  
   
    theta1=np.arctan2(desti.item(indexes[4][1],1)-pos.item(4,1),desti.item(indexes[4][1],0)-pos.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.1
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
   
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
    e=1    
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    e=0
  
  time.sleep(0.005)

print ("finish")





