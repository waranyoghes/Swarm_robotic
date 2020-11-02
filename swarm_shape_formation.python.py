# -*- coding: utf-8 -*-
"""
Created on Thu May 31 14:02:33 2018

@author: YOGHES WARAN
"""

import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting
import threading
from munkres import Munkres, print_matrix
#Pre-Allocation

m = Munkres()
PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print 'Connected to remote API server'
    
else:
    print 'Connection not successful'
    sys.exit('Could not connect')


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

desti=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2]])


def task1(lock):
  lock.acquire()  
  if (abs(desti[indexes[0][1]]-pos[0])>a).all():  
   
   theta1=np.arctan2(desti.item(indexes[0][1],1)-pos.item(0,1),desti.item(indexes[0][1],0)-pos.item(0,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]+angle[0]
   theta=theta1+angle5-theta2        
   r=0.0925
   d=0.38
   v=0.16
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)   
   
   
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,0, vrep.simx_opmode_streaming)       
  lock.release()    

def task2(lock):
 lock.acquire()
 if (abs(desti[indexes[1][1]]-pos[1])>a).all():  
   
   theta1=np.arctan2(desti.item(indexes[1][1],1)-pos.item(1,1),desti.item(indexes[1][1],0)-pos.item(1,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]+angle[0]
   theta=theta1+angle5-theta2        
   r=0.0925
   d=0.38
   v=0.16
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)   
   
   
 else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,0, vrep.simx_opmode_streaming)       
 lock.release()  

def task3(lock):
 lock.acquire()
 if (abs(desti[indexes[2][1]]-pos[2])>a).all():  
   
   theta1=np.arctan2(desti.item(indexes[2][1],1)-pos.item(2,1),desti.item(indexes[2][1],0)-pos.item(2,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]+angle[0]
   theta=theta1+angle5-theta2        
   r=0.0925
   d=0.38
   v=0.16
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)   
     
   
 else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,0, vrep.simx_opmode_streaming)       
 lock.release() 

def task4(lock):
 lock.acquire()
 if (abs(desti[indexes[3][1]]-pos[3])>a).all():  
   
   theta1=np.arctan2(desti.item(indexes[3][1],1)-pos.item(3,1),desti.item(indexes[3][1],0)-pos.item(3,0))
   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
   theta2=angle[2]+angle[0]
   theta=theta1+angle5-theta2        
   r=0.0925
   d=0.38
   v=0.16
   wo=0.4*theta
   vr=v+d*wo
   vl=v-d*wo
   vr=vr/r
   vl=vl/r
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)   
     
   
 else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,0, vrep.simx_opmode_streaming)       
 lock.release() 
count=1
while (count):
  
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  
  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

  pos1=pos1[0:2]   
  pos2=pos2[0:2]
  pos3=pos3[0:2]   
  pos4=pos4[0:2]   
  pos5=pos5[0:2]      
  
  pos5=np.array(pos5)
  
  angle=angle[0]+angle[2]
  r=np.zeros((2,2))
  
  r[0][0]=math.cos(angle)
  r[0][1]=math.sin(angle)
  r[1][0]=-math.sin(angle)
  r[1][1]=math.cos(angle)

  pos1=pos1-pos5
  pos2=pos2-pos5
  pos3=pos3-pos5
  pos4=pos4-pos5
  
  pos1=np.dot(r,pos1)
  pos2=np.dot(r,pos2)
  pos3=np.dot(r,pos3)
  pos4=np.dot(r,pos4)
  
  pos5=pos5.tolist()
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  matrix=np.zeros(shape=(4,4))

  for i in range(0,4):
    for j in range(0,4):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[1,-4]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  
  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
   
    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]+angle[0]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.15
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
    a=0.00000001
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
        
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    a=0.05
    count=0
  pos=np.matrix([pos1,pos2,pos3,pos4])
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t1.join()
  t2.join()
  t3.join()
  t4.join()
 

count=1
while (count):
  
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  
  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

  pos1=pos1[0:2]   
  pos2=pos2[0:2]
  pos3=pos3[0:2]   
  pos4=pos4[0:2]   
  pos5=pos5[0:2]      
  
  pos5=np.array(pos5)
  
  angle=angle[0]+angle[2]
  r=np.zeros((2,2))
  
  r[0][0]=math.cos(angle)
  r[0][1]=math.sin(angle)
  r[1][0]=-math.sin(angle)
  r[1][1]=math.cos(angle)

  pos1=pos1-pos5
  pos2=pos2-pos5
  pos3=pos3-pos5
  pos4=pos4-pos5
  
  pos1=np.dot(r,pos1)
  pos2=np.dot(r,pos2)
  pos3=np.dot(r,pos3)
  pos4=np.dot(r,pos4)
  
  pos5=pos5.tolist()
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  matrix=np.zeros(shape=(4,4))

  for i in range(0,4):
    for j in range(0,4):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[4,-4]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  
  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
   
    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]+angle[0]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.15
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
    a=0.00000001
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
        
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    a=0.05
    count=0
  pos=np.matrix([pos1,pos2,pos3,pos4])
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t1.join()
  t2.join()
  t3.join()
  t4.join()
 
count=1
while (count):
  
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  
  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

  pos1=pos1[0:2]   
  pos2=pos2[0:2]
  pos3=pos3[0:2]   
  pos4=pos4[0:2]   
  pos5=pos5[0:2]      
  
  pos5=np.array(pos5)
  
  angle=angle[0]+angle[2]
  r=np.zeros((2,2))
  
  r[0][0]=math.cos(angle)
  r[0][1]=math.sin(angle)
  r[1][0]=-math.sin(angle)
  r[1][1]=math.cos(angle)

  pos1=pos1-pos5
  pos2=pos2-pos5
  pos3=pos3-pos5
  pos4=pos4-pos5
  
  pos1=np.dot(r,pos1)
  pos2=np.dot(r,pos2)
  pos3=np.dot(r,pos3)
  pos4=np.dot(r,pos4)
  
  pos5=pos5.tolist()
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  matrix=np.zeros(shape=(4,4))

  for i in range(0,4):
    for j in range(0,4):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[4,1]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  
  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
   
    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]+angle[0]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.15
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
    a=0.00000001
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
        
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    a=0.05
    count=0
  pos=np.matrix([pos1,pos2,pos3,pos4])
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t1.join()
  t2.join()
  t3.join()
  t4.join()
 
count=1
while (count):
  
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  
  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

  pos1=pos1[0:2]   
  pos2=pos2[0:2]
  pos3=pos3[0:2]   
  pos4=pos4[0:2]   
  pos5=pos5[0:2]      
  
  pos5=np.array(pos5)
  
  angle=angle[0]+angle[2]
  r=np.zeros((2,2))
  
  r[0][0]=math.cos(angle)
  r[0][1]=math.sin(angle)
  r[1][0]=-math.sin(angle)
  r[1][1]=math.cos(angle)

  pos1=pos1-pos5
  pos2=pos2-pos5
  pos3=pos3-pos5
  pos4=pos4-pos5
  
  pos1=np.dot(r,pos1)
  pos2=np.dot(r,pos2)
  pos3=np.dot(r,pos3)
  pos4=np.dot(r,pos4)
  
  pos5=pos5.tolist()
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  matrix=np.zeros(shape=(4,4))

  for i in range(0,4):
    for j in range(0,4):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[1,1]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  
  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
   
    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]+angle[0]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.15
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
    a=0.00000001
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
        
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    a=0.05
    count=0
  pos=np.matrix([pos1,pos2,pos3,pos4])
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t1.join()
  t2.join()
  t3.join()
  t4.join()
 
t = time.time()

while ((t-time.time())<60):

  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  
  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)

  pos1=pos1[0:2]   
  pos2=pos2[0:2]
  pos3=pos3[0:2]   
  pos4=pos4[0:2]   
  pos5=pos5[0:2]      
  
  pos5=np.array(pos5)
  
  angle=angle[0]+angle[2]
  r=np.zeros((2,2))
  
  r[0][0]=math.cos(angle)
  r[0][1]=math.sin(angle)
  r[1][0]=-math.sin(angle)
  r[1][1]=math.cos(angle)

  pos1=pos1-pos5
  pos2=pos2-pos5
  pos3=pos3-pos5
  pos4=pos4-pos5
  
  pos1=np.dot(r,pos1)
  pos2=np.dot(r,pos2)
  pos3=np.dot(r,pos3)
  pos4=np.dot(r,pos4)
  
  pos5=pos5.tolist()
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  matrix=np.zeros(shape=(4,4))

  for i in range(0,4):
    for j in range(0,4):  
     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])

  p=matrix.tolist()

  indexes = m.compute(p)


  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[1,1]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
  
  
  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
   
    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
    theta2=angle[2]+angle[0]
    theta=theta1-theta2        
    r=0.0925
    d=0.38
    v=0.15
    wo=0.4*theta
    vr=v+d*wo
    vl=v-d*wo
    vr=vr/r
    vl=vl/r
    a=0.00000001
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
        
  
  else:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
    a=0.05
    count=0
  pos=np.matrix([pos1,pos2,pos3,pos4])
  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t1.join()
  t2.join()
  t3.join()
  t4.join()  
  
  
 


print "finish"
