
simxGetObjectOrientation

#world  to gripper 

returnCode,ROBOTIQ_85_handle=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85',sim.simx_opmode_oneshot_wait) #definition gripper

returnCode,gripper_position=sim.simxGetObjectPosition(clientID,ROBOTIQ_85_handle,-1,sim.simx_opmode_streaming)  
returnCode,gripper_orientation=sim.simxGetObjectOrientation(clientID,ROBOTIQ_85_handle,-1,sim.simx_opmode_streaming) 

time.sleep(2)

returnCode,gripper_position=sim.simxGetObjectPosition(clientID,ROBOTIQ_85_handle,-1,sim.simx_opmode_buffer)
returnCode,gripper_orientation=sim.simxGetObjectOrientation(clientID,ROBOTIQ_85_handle,-1,sim.simx_opmode_buffer)
print('Gripper Information')
gripper_position[0]=gripper_position[0]*1000 #convert to mm
gripper_position[1]=gripper_position[1]*1000
gripper_position[2]=gripper_position[2]*1000
print('gripper position:',gripper_position)
print('gripper orientation:',gripper_orientation)  
print('--------------------------')



#world to vision sensor
returnCode,visionSensor_handle=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)


returnCode,visionSensor_position=sim.simxGetObjectPosition(clientID,visionSensor_handle,-1,sim.simx_opmode_streaming)  
returnCode,visionSensor_orientation=sim.simxGetObjectOrientation(clientID,visionSensor_handle,-1,sim.simx_opmode_streaming) 
time.sleep(2)

returnCode,visionSensor_position=sim.simxGetObjectPosition(clientID,visionSensor_handle,-1,sim.simx_opmode_buffer)
returnCode,visionSensor_orientation=sim.simxGetObjectOrientation(clientID,visionSensor_handle,-1,sim.simx_opmode_buffer)

visionSensor_position[0]=visionSensor_position[0]*1000 #convert to mm
visionSensor_position[1]=visionSensor_position[1]*1000
visionSensor_position[2]=visionSensor_position[2]*1000


returnCode,camera_gripper_position=sim.simxGetObjectPosition(clientID,ROBOTIQ_85_handle,visionSensor_handle,sim.simx_opmode_streaming)
returnCode,camera_gripper_orientation=sim.simxGetObjectOrientation(clientID,ROBOTIQ_85_handle,visionSensor_handle,sim.simx_opmode_streaming)
time.sleep(2)

returnCode,camera_gripper_position=sim.simxGetObjectPosition(clientID,ROBOTIQ_85_handle,visionSensor_handle,sim.simx_opmode_buffer)
returnCode,camera_gripper_orientation=sim.simxGetObjectOrientation(clientID,ROBOTIQ_85_handle,visionSensor_handle,sim.simx_opmode_buffer)

camera_gripper_position[0]=camera_gripper_position[0]*1000 #convert to mm
camera_gripper_position[1]=camera_gripper_position[1]*1000
camera_gripper_position[2]=camera_gripper_position[2]*1000
print('camera  gripper position:',camera_gripper_position)
print('camera gripper orientation:',camera_gripper_orientation)  


simSetGraphUserData(graphHandle,'myData',value)
sim.setGraphUserData(graphHandle, "CPG_output", data)