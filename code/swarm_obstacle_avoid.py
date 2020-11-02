# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 11:53:10 2020

@author: YOGHES WARAN
"""

# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:32:54 2018

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
global a
global pos0
global desti0 
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
#  global a  
#  lock.acquire()  
#      
#  if sensor_val0[min_ind][min_ind0]<0.3:
#        steer=-1/sensor_loc[min_ind0]
#        v=1	#forward velocity
#        kp=0.5	#steering gain
#        vl=v+kp*steer
#        vr=v-kp*steer
##        print "V_l =",vl
##        print "V_r =",vr
#        
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)    
#      
#  
#  else:
#   if (abs(desti[indexes[0][1]]-pos[0])>a).all():  
#   
#    theta1=np.arctan2(desti.item(indexes[0][1],1)-pos.item(0,1),desti.item(indexes[0][1],0)-pos.item(0,0))
#    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
#    theta2=angle[2]+angle[0]
#    theta=theta1+angle5-theta2        
#    r=0.0925
#    d=0.38
#    v=0.16
#    wo=0.4*theta
#    vr=v+d*wo
#    vl=v-d*wo
#    vr=vr/r
#    vl=vl/r
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)   
#   
#   
#   else:
 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle1,vl, vrep.simx_opmode_streaming)
 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle1,vr, vrep.simx_opmode_streaming)       
#  lock.release()    

def task2(lock):
# global a 
# lock.acquire()
# if sensor_val0[min_ind][min_ind0]<0.3:   
#        steer=-1/sensor_loc[min_ind0]
#        v=1	#forward velocity
#        kp=0.5	#steering gain
#        vl=v+kp*steer
#        vr=v-kp*steer
##        print "V_l =",vl
##        print "V_r =",vr
#        
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)    
#     
# 
# else:
#  if (abs(desti[indexes[1][1]]-pos[1])>a).all():  
#   
#   theta1=np.arctan2(desti.item(indexes[1][1],1)-pos.item(1,1),desti.item(indexes[1][1],0)-pos.item(1,0))
#   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
#   theta2=angle[2]+angle[0]
#   theta=theta1+angle5-theta2        
#   r=0.0925
#   d=0.38
#   v=0.16
#   wo=0.4*theta
#   vr=v+d*wo
#   vl=v-d*wo
#   vr=vr/r
#   vl=vl/r
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)   
#   
#   
#  else:
 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle2,vl, vrep.simx_opmode_streaming)
 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle2,vr, vrep.simx_opmode_streaming)       
# lock.release()  

def task3(lock):
# global a 
# lock.acquire()
#    
# if sensor_val0[min_ind][min_ind0]<0.3: 
#        steer=-1/sensor_loc[min_ind0]
#        v=1	#forward velocity
#        kp=0.5	#steering gain
#        vl=v+kp*steer
#        vr=v-kp*steer
##        print "V_l =",vl
##        print "V_r =",vr
#        
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)    
#     
# 
# 
# else:
#  if (abs(desti[indexes[2][1]]-pos[2])>a).all():  
#   
#   theta1=np.arctan2(desti.item(indexes[2][1],1)-pos.item(2,1),desti.item(indexes[2][1],0)-pos.item(2,0))
#   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
#   theta2=angle[2]+angle[0]
#   theta=theta1+angle5-theta2        
#   r=0.0925
#   d=0.38
#   v=0.16
#   wo=0.4*theta
#   vr=v+d*wo
#   vl=v-d*wo
#   vr=vr/r
#   vl=vl/r
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)   
#     
#   
#  else:
 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle3,vl, vrep.simx_opmode_streaming)
 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle3,vr, vrep.simx_opmode_streaming)       
# lock.release() 

def task4(lock):
# global a 
# lock.acquire()
## if sensor_val0[min_ind][min_ind0]<0.3:
##        steer=-1/sensor_loc[min_ind0]
##        v=1	#forward velocity
##        kp=0.5	#steering gain
##        vl=v+kp*steer
##        vr=v-kp*steer
###        print "V_l =",vl
###        print "V_r =",vr
#        
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
#        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)    
#     
# 
# else:
#  if (abs(desti[indexes[3][1]]-pos[3])>a).all():  
#   
#   theta1=np.arctan2(desti.item(indexes[3][1],1)-pos.item(3,1),desti.item(indexes[3][1],0)-pos.item(3,0))
#   errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
#   theta2=angle[2]+angle[0]
#   theta=theta1+angle5-theta2        
#   r=0.0925
#   d=0.38
#   v=0.16
#   wo=0.4*theta
#   vr=v+d*wo
#   vl=v-d*wo
#   vr=vr/r
#   vl=vl/r
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
#   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)   
#     
#   
#  else:
 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle4,vl, vrep.simx_opmode_streaming)
 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle4,vr, vrep.simx_opmode_streaming)       
# lock.release() 
    
   
   
    
#def sensor2(arr):
#    sensor_val=np.append([sensor_val,arr[15]])
#    sensor_val=np.append([sensor_val,arr[0]])
#
#def sensor3(arr):
#    sensor_val=np.append([sensor_val,arr[7]])
#    sensor_val=np.append([sensor_val,arr[8]])
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 
    
a=0.0001
count=1
errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[4,4]])
pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
   
while (count):
     
  desti=np.matrix([[0.77,0.77],[-0.77,0.77],[-0.77,-0.77],[0.77,-0.77]])
  
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
 
 
 # SENSOR VALUE FUNCTION CALIING
 
        
  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
  angle5=angle5[0]+angle5[2] 
  
  
  lock = threading.Lock()
  t1 = threading.Thread(target=task1, args=(lock,))
  t2 = threading.Thread(target=task2, args=(lock,))
  t3 = threading.Thread(target=task3, args=(lock,))
  t4 = threading.Thread(target=task4, args=(lock,))
  
  
  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[5,5]])
  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
 
   
  sensor_val1=np.array([])    
  sensor_val2=np.array([])    
  sensor_val3=np.array([])    
  sensor_val4=np.array([])    

  for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h1[x-1],vrep.simx_opmode_buffer)                
        sensor_val1=np.append(sensor_val1,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
  sensor_sq=sensor_val1[0:8]*sensor_val1[0:8] #square the values of front-facing sensors 1-8
        
  min_ind=np.where(sensor_sq==np.min(sensor_sq))
  min_ind1=min_ind[0][0]
  
  for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h2[x-1],vrep.simx_opmode_buffer)                
        sensor_val2=np.append(sensor_val2,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
  sensor_sq=sensor_val2[0:8]*sensor_val2[0:8] #square the values of front-facing sensors 1-8
        
  min_ind=np.where(sensor_sq==np.min(sensor_sq))
  min_ind2=min_ind[0][0]

  for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h3[x-1],vrep.simx_opmode_buffer)                
        sensor_val3=np.append(sensor_val3,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
  sensor_sq=sensor_val3[0:8]*sensor_val3[0:8] #square the values of front-facing sensors 1-8
        
  min_ind=np.where(sensor_sq==np.min(sensor_sq))
  min_ind3=min_ind[0][0]

  for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h4[x-1],vrep.simx_opmode_buffer)                
        sensor_val4=np.append(sensor_val4,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
  sensor_sq=sensor_val4[0:8]*sensor_val4[0:8] #square the values of front-facing sensors 1-8
        
  min_ind=np.where(sensor_sq==np.min(sensor_sq))
  min_ind4=min_ind[0][0]
  
  sensor_val=np.array([sensor_val1[min_ind1],sensor_val2[min_ind2],sensor_val3[min_ind3],sensor_val4[min_ind4]])  
  
  sensor_sq=sensor_val[0:4]*sensor_val[0:4] #square the values of front-facing sensors 1-8
  
  min_ind=np.where(sensor_sq==np.min(sensor_sq))
  min_ind=min_ind[0][0]
  
  if min_ind==0:
      min_ind0=min_ind1
  else:
      if min_ind==1:
          min_ind0=min_ind2
      else:
          if min_ind==2:
              min_ind0=min_ind3
          else:
              min_ind0=min_ind4
  
  sensor_val0=np.array([sensor_val1.tolist(),sensor_val2.tolist(),sensor_val3.tolist(),sensor_val4.tolist()])
  
  
  if sensor_val0[min_ind][min_ind0]<0.4:
        steer=-1/sensor_loc[min_ind0]
        v=1	#forward velocity
        kp=0.5	#steering gain
        vl=v+kp*steer
        vr=v-kp*steer
#        print "V_l =",vl
#        print "V_r =",vr
        print (min_ind)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)    
  else:  
   if (abs(desti0.item(4,1)-pos0.item(4,1))>0.5) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.5) :  
   
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
     vl=0
     vr=0     
     errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
     errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
     a=0.5
     count=0
     print (count)
  
  pos=np.matrix([pos1,pos2,pos3,pos4])
  
  t1.start()
  t2.start()
  t3.start()  
  t4.start()
  
  t4.join()
  t3.join()
  t2.join()
  t1.join()
  time.sleep(0.025)
print ("end")


#while ((t-time.time())<60):
#
#  desti=np.matrix([[1,1],[-1,1],[-1,-1],[1,-1]])
#  
#  errorCode,pos1=vrep.simxGetObjectPosition(clientID,p3dx1,-1,vrep.simx_opmode_blocking)
#  errorCode,pos2=vrep.simxGetObjectPosition(clientID,p3dx2,-1,vrep.simx_opmode_blocking)
#  errorCode,pos3=vrep.simxGetObjectPosition(clientID,p3dx3,-1,vrep.simx_opmode_blocking)
#  errorCode,pos4=vrep.simxGetObjectPosition(clientID,p3dx4,-1,vrep.simx_opmode_blocking)
#  errorCode,pos5=vrep.simxGetObjectPosition(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
#  
#  errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
#
#  pos1=pos1[0:2]   
#  pos2=pos2[0:2]
#  pos3=pos3[0:2]   
#  pos4=pos4[0:2]   
#  pos5=pos5[0:2]      
#  
#  pos5=np.array(pos5)
#  
#  angle=angle[0]+angle[2]
#  r=np.zeros((2,2))
#  
#  r[0][0]=math.cos(angle)
#  r[0][1]=math.sin(angle)
#  r[1][0]=-math.sin(angle)
#  r[1][1]=math.cos(angle)
#
#  pos1=pos1-pos5
#  pos2=pos2-pos5
#  pos3=pos3-pos5
#  pos4=pos4-pos5
#  
#  pos1=np.dot(r,pos1)
#  pos2=np.dot(r,pos2)
#  pos3=np.dot(r,pos3)
#  pos4=np.dot(r,pos4)
#  
#  pos5=pos5.tolist()
#  pos=np.matrix([pos1,pos2,pos3,pos4])
#  
#  matrix=np.zeros(shape=(4,4))
#
#  for i in range(0,4):
#    for j in range(0,4):  
#     matrix[i][j]=np.linalg.norm(desti[j]-pos[i])
#
#  p=matrix.tolist()
#
#  indexes = m.compute(p)
#
#
#  errorCode,angle5=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
#  angle5=angle5[0]+angle5[2] 
#  
#  
#  lock = threading.Lock()
#  t1 = threading.Thread(target=task1, args=(lock,))
#  t2 = threading.Thread(target=task2, args=(lock,))
#  t3 = threading.Thread(target=task3, args=(lock,))
#  t4 = threading.Thread(target=task4, args=(lock,))
#  
#  
#  desti0=np.matrix([[2,2],[-2,2],[-2,-2],[2,-2],[10,0]])
#  pos0=np.matrix([pos1,pos2,pos3,pos4,pos5])
#  
#  
#  if (abs(desti0.item(4,1)-pos0.item(4,1))>0.05) or (abs(desti0.item(4,0)-pos0.item(4,0))>0.05) :  
#   
#    theta1=np.arctan2(desti0.item(4,1)-pos0.item(4,1),desti0.item(4,0)-pos0.item(4,0))
#    errorCode,angle=vrep.simxGetObjectOrientation(clientID,p3dx5,-1,vrep.simx_opmode_blocking)
#    theta2=angle[2]+angle[0]
#    theta=theta1-theta2        
#    r=0.0925
#    d=0.38
#    v=0.15
#    wo=0.4*theta
#    vr=v+d*wo
#    vl=v-d*wo
#    vr=vr/r
#    vl=vl/r
#    a=0.00000001
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,vl, vrep.simx_opmode_streaming)
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,vr, vrep.simx_opmode_streaming)
#        
#  
#  else:
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle5,0, vrep.simx_opmode_streaming)
#    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle5,0, vrep.simx_opmode_streaming)       
#    a=0.05
#    count=0
#  pos=np.matrix([pos1,pos2,pos3,pos4])
#  
#  t1.start()
#  t2.start()
#  t3.start()  
#  t4.start()
#  
#  t1.join()
#  t2.join()
#  t3.join()
#  t4.join()  
#  
#  
# 
#
#
print ("finish")
