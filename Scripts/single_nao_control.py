# -*- coding: utf-8 -*-
"""
@author: Pierre Jacquot
"""
#For more informations please check : http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm
import vrep,sys,time
from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl
from threading import Thread # 多线程
import json
from vision_sensor import streamVisionSensor, analysisVideo # 获取视频流


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

def video2_detect(coordlist, config):
    pass

handle_video2_thread = Thread(target=analysisVideo, args=("NAO_vision2", clientID, video2_detect))

def video1_detect(coordlist, config):
	# for obj in coordlist:
	#     print obj['class']
	# print "[%s]x1:%s,y1:%s,x2:%s,y2:%s" % (obj['label'], obj['coord'][0], obj['coord'][1], obj['coord'][2],
	#                                        obj['coord'][3])  # output object coordination.
	# print coordlist, config
	for obj in coordlist:
		if obj['class'] == config["targetClass"]:
			config["targetObjs"].append(obj)
	config["nowFrame"] += 1
	if config["nowFrame"] >= config["frameNum"]:
		ctrlRobot(config["targetObjs"],config)
		config["targetObjs"] = []
		config["nowFrame"] = 0

def ctrlRobot(targetClassFrames,config):
    if len(targetClassFrames) == 0:
        return
    bias_range = 50
    all_center_x = 0
    for frame in targetClassFrames:
        # print frame
        all_center_x += (frame['coord'][0]+frame['coord'][2])/2
    image_c_x = targetClassFrames[0]['w']/2
    obj_c_x = all_center_x/len(targetClassFrames)
    print "centerX of image", image_c_x
    print "centerX of target:", obj_c_x
    if obj_c_x < image_c_x - bias_range or obj_c_x > image_c_x + bias_range:
        # robot_ip = "127.0.0.1"
        # robot_port = 9527
        # try:
        #     motionProxy = ALProxy("ALMotion", robot_ip, robot_port)
        # except Exception, e:
        #     print "Error:", e
        x = 0
        y = 0
        if obj_c_x < image_c_x - bias_range:
            theta = 0.03
        else:
            theta = -0.03
        motionProxy.moveToward(x, y, theta)
    else:
        print "Theta is done!"
        x = 0.5
        y = 0
        theta = 0
        motionProxy.moveToward(x, y, theta)
        if not config["show_in_v2"]:
            print "have opened the second video~"
            handle_video2_thread.start()
            config["show_in_v2"] = True



# 主线程
main_thread=Thread(target=JointControl,args=(clientID,motionProxy,0,Body))
main_thread.start()
time.sleep(1)

# 视频流处理线程
handle_video1_thread = Thread(target=analysisVideo, args=("NAO_vision1", clientID, video1_detect))
handle_video1_thread.start()