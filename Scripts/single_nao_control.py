# -*- coding: utf-8 -*-
"""
@author: Pierre Jacquot
"""
#For more informations please check : http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm
import vrep,sys
from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl
from threading import Thread # 多线程
import json
from vision_sensor import streamVisionSensor # 获取视频流

print '================ Program Sarted ================'

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.2',19997,True,True,5000,5)
if clientID!=-1:
	print 'Connected to remote API server'

else:
	print 'Connection non successful'
	sys.exit('Could not connect')
	# 考虑修改上面vrep.simxStart中的第二个参数
	# 参数从C:\Program Files\V-REP3\V-REP_PRO_EDU\remoteApiConnections.txt文件中获得

print "================ Choregraphe's Initialization ================"

# 文件读取方式
socket_config_file= "./socket_config.json"
with open(socket_config_file,'r') as f:
	config_dict = json.load(f)
	print "read socket config from ",socket_config_file,":",config_dict
naoIP=config_dict[u"nao_ip"].encode('utf-8')
naoPort=int(config_dict[u"nao_port"])

# 每次输入方式
# print 'Enter your NAO IP:'
# naoIP = raw_input()
# print 'Enter your NAO port:'
# naoPort = raw_input()

# print type(naoIP),type(naoPort)
#naoPort = map(int,naoPort.split())

motionProxy = ALProxy("ALMotion",naoIP, naoPort)
postureProxy = ALProxy("ALRobotPosture", naoIP, naoPort)

#Go to the posture StandInitZero
posture = 'StandZero'
print 'Posture Initialization : ' + posture
motionProxy.stiffnessInterpolation('Body', 1.0, 1.0)
postureProxy.goToPosture(posture, 1.0)  # 原为postureProxy.goToPosture(posture,1.0,1.0)

Head_Yaw=[];Head_Pitch=[];
L_Hip_Yaw_Pitch=[];L_Hip_Roll=[];L_Hip_Pitch=[];L_Knee_Pitch=[];L_Ankle_Pitch=[];L_Ankle_Roll=[];
R_Hip_Yaw_Pitch=[];R_Hip_Roll=[];R_Hip_Pitch=[];R_Knee_Pitch=[];R_Ankle_Pitch=[];R_Ankle_Roll=[];
L_Shoulder_Pitch=[];L_Shoulder_Roll=[];L_Elbow_Yaw=[];L_Elbow_Roll=[];L_Wrist_Yaw=[]
R_Shoulder_Pitch=[];R_Shoulder_Roll=[];R_Elbow_Yaw=[];R_Elbow_Roll=[];R_Wrist_Yaw=[]
R_H=[];L_H=[];R_Hand=[];L_Hand=[];
Body = [Head_Yaw,Head_Pitch,L_Hip_Yaw_Pitch,L_Hip_Roll,L_Hip_Pitch,L_Knee_Pitch,L_Ankle_Pitch,L_Ankle_Roll,R_Hip_Yaw_Pitch,R_Hip_Roll,R_Hip_Pitch,R_Knee_Pitch,R_Ankle_Pitch,R_Ankle_Roll,L_Shoulder_Pitch,L_Shoulder_Roll,L_Elbow_Yaw,L_Elbow_Roll,L_Wrist_Yaw,R_Shoulder_Pitch,R_Shoulder_Roll,R_Elbow_Yaw,R_Elbow_Roll,R_Wrist_Yaw,L_H,L_Hand,R_H,R_Hand]

get_first_handles(clientID,Body)
print "================ Handles Initialization ================"
commandAngles = motionProxy.getAngles('Body', False)
print '========== NAO is listening =========='

def handle_video():
	streamVisionSensor("NAO_vision1",clientID) # 获取naoqi上面摄像头的数据

# 主线程
main_thread=Thread(target=JointControl,args=(clientID,motionProxy,0,Body))
main_thread.start()
# 视频流处理线程
handle_video_thread=Thread(target=streamVisionSensor,args=("NAO_vision1",clientID))
handle_video_thread.start()

