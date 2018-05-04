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
import math


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

motionProxy = ALProxy("ALMotion", naoIP, naoPort)
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
commandAngles = motionProxy.getAngles('Body', True)
# print "command angles:", commandAngles
print '========== NAO is listening =========='

def ctrlRobot(target_classFrames,config):
    if len(target_classFrames) == 0:
        print "[INFO] 未检测到目标物体"
        return
    bias_range = 50 # 误差范围
    all_center_x = 0
    all_bottom_y = 0
    all_center_y = 0
    for frame in target_classFrames:
        # print frame
        all_center_x += (frame['coord'][0]+frame['coord'][2])/2
        all_center_y += (frame['coord'][1]+frame['coord'][3])/2
        all_bottom_y += frame['coord'][3]
    image_c_x = target_classFrames[0]['w']/2
    image_c_y = target_classFrames[0]['h']/2
    obj_c_x = all_center_x/len(target_classFrames)  # object x's center
    obj_c_y = all_center_y/len(target_classFrames)  # object y's center
    obj_b_y = all_bottom_y/len(target_classFrames)
    print "----------------------------------"
    print "(centerX,centerY) of image:(", image_c_x, image_c_y, ")"
    print "(centerX,centerY) of target:(", obj_c_x, obj_c_y, ")"
    print "bottomY of target:", obj_b_y
    if obj_c_x < image_c_x - bias_range or obj_c_x > image_c_x + bias_range:
        print "水平角度纠正中..."
        x = 0
        y = 0
        if obj_c_x < image_c_x - bias_range:
            theta = 0.03
        else:
            theta = -0.03
        motionProxy.moveToward(x, y, theta)
    else:
        focal_length = 372 # 像素
        nao_height = 48.3  # cm
        print "水平角度纠正完成!"
        obj_y_len = abs(obj_b_y - image_c_y)  # 物体y轴距图像中心带的距离
        gama = math.atan(obj_y_len*1.0 / focal_length)  # 372为焦距像素
        print "gama幅度", gama
        gama_angle = gama*360*0.1/(2*math.pi)  # 算角度
        pitch_angle = motionProxy.getAngles('HeadPitch', True)[0]  # 头上下角度
        beta = gama_angle+pitch_angle+1.2  # 1.2为上面摄像头与水平的偏移距离
        print "最终角度：", beta
        dist = nao_height*1.0 / math.tan(beta) / 100
        print "距离物体距离：", dist, "米"
        if dist<0.5:
            print "结束行走"
            return
        x = dist
        y = 0
        theta = 0
        motionProxy.moveTo(x, y, theta)
        # if not config["show_in_v2"]:
        #     print "[Info] Have opened the second video~"
        #     config["show_in_v2"] = True

def get_average_coord(coordlist, config, callback):
    for obj in coordlist:
        if obj['class'] == config["target_class"]:
            config["target_objs"].append(obj)
    config["now_frame"] += 1
    if config["now_frame"] >= config["frame_num"]:
        callback(config["target_objs"], config)
        config["target_objs"] = []
        config["now_frame"] = 0



def handle_video1_coordlist(coordlist, config):
    # print "video1:", coordlist
    get_average_coord(coordlist, config, ctrlRobot)


def handle_video2_coordlist(coordlist, config):
    print "video2:", coordlist
    get_average_coord(coordlist, config, ctrlRobot)

# 主线程
main_thread=Thread(target=JointControl,args=(clientID,motionProxy,0,Body))
main_thread.start()
# 视频处理线程
# 上摄像头视频处理
video_config = {
    "target_class": "pottedplant",
    "target_objs": [],
    "frame_num": 5,
    "now_frame": 0,
    "show_in_v2": False,
    "video_label":"outputTop",
    "play_video": True,
    "vision_sensor_handle": -1
}
handle_video_thread = Thread(target=analysisVideo, args=("NAO_vision1", clientID, video_config, handle_video1_coordlist))
handle_video_thread.start()
# 下面部分视频处理
# video2_config = {
#     "target_class": "pottedplant",
#     "target_objs": [],
#     "frame_num": 5,
#     "now_frame": 0,
#     "show_in_v1": False,
#     "video_label":"outputBottom",
#     "play_video": True,
#     "vision_sensor_handle": -1
# }
# handle_video2_thread = Thread(target=analysisVideo, args=("NAO_vision2", clientID, video2_config, handle_video2_coordlist))
# handle_video2_thread.start()