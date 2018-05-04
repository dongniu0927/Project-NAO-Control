# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 15:30:54 2015

@author: Pierre Jacquot
"""

import vrep,time,sys
from naoqi import ALProxy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as I
import array
from threading import Thread # 多线程
import cv2
sys.path.append("../../")
from objectDetectionMulticlasses.VideoObjectDetection import detect

def streamVisionSensor(vision_sensor_name,client_id,pause=0.0001):
    # 获得handle
    res1,visionSensorHandle=vrep.simxGetObjectHandle(client_id,vision_sensor_name,vrep.simx_opmode_oneshot_wait)
    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(client_id,visionSensorHandle,0,vrep.simx_opmode_streaming)
    #Allow the display to be refreshed
    plt.ion()
    #Initialiazation of the figure
    time.sleep(0.5)
    if vrep.simxGetConnectionId(client_id)!=-1:
        res,resolution,image=vrep.simxGetVisionSensorImage(client_id,visionSensorHandle,0,vrep.simx_opmode_buffer)
        print "size of video:",resolution,"return code",res
        im = I.new("RGB", (resolution[0], resolution[1]), "white")
        #Give a title to the figure
        fig = plt.figure(1)
        fig.canvas.set_window_title(vision_sensor_name)
        #inverse the picture
        plotimg = plt.imshow(im,origin='lower')
        #Let some time to Vrep in order to let him send the first image, otherwise the loop will start with an empty image and will crash
        time.sleep(1)
        # 播放原视频
        while (vrep.simxGetConnectionId(client_id) != -1):
            # Get the image of the vision sensor
            res, resolution, image = vrep.simxGetVisionSensorImage(client_id, visionSensorHandle, 0,vrep.simx_opmode_buffer)
            # Transform the image so it can be displayed using pyplot
            if len(resolution) == 0:
                continue
            image_byte_array = array.array('b', image)
            im = I.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
            # Update the image
            plotimg.set_data(im)
            # Refresh the display
            plt.draw()
            # The mandatory pause ! (or it'll not work)
            plt.pause(pause)
    print 'End of Simulation'


def analysisVideo(vision_sensor_name, client_id, config, handle_video_coordlist):
    res1, visionSensorHandle = vrep.simxGetObjectHandle(client_id, vision_sensor_name, vrep.simx_opmode_oneshot_wait)
    # config["vision_sensor_handle"] = visionSensorHandle
    vrep.simxGetVisionSensorImage(client_id, visionSensorHandle, 0, vrep.simx_opmode_streaming)
    time.sleep(1)
    while (vrep.simxGetConnectionId(client_id)!=-1):
        # if visionSensorHandle != config["vision_sensor_handle"]:
        #     continue
        # print "config_handle:", config["vision_sensor_handle"]
        # print "handle:", visionSensorHandle
        res, resolution, image = vrep.simxGetVisionSensorImage(client_id, visionSensorHandle, 0, vrep.simx_opmode_buffer)
        while (not image):
            pass
        if len(resolution) == 0:
            continue
        image_byte_array = array.array('b', image)
        _img = I.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        im = np.array(_img)[::-1, :, ::-1].copy()
        coordlist = detect(im)  # 检测目标物体坐标
        handle_video_coordlist(coordlist, config)
        # if config["play_video"]:
        cv2.imshow(config["video_label"], im)
        if cv2.waitKey(15) & 0xFF == 27:  # Press ESC to exit :)
            break
    print 'End of Analysis'

def getVisionSensor(vision_sensor_name,client_id):
    #Get the handle of the vision sensor
    res1,visionSensorHandle=vrep.simxGetObjectHandle(client_id,vision_sensor_name,vrep.simx_opmode_oneshot_wait)
    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(client_id,visionSensorHandle,0,vrep.simx_opmode_streaming)
    time.sleep(1)
    while (vrep.simxGetConnectionId(client_id)!=-1):
        #Get the image of the vision sensor
        res,resolution,image=vrep.simxGetVisionSensorImage(client_id,visionSensorHandle,0,vrep.simx_opmode_buffer)
        # print resolution
    print 'End of Simulation'
    
if __name__ == '__main__':
    vrep.simxFinish(-1)
    client_id=vrep.simxStart('127.0.0.2',19999,True,True,5000,5)
    if client_id!=-1:
        print 'Connected to remote API server'
        #Get and display the pictures from the camera
        streamVisionSensor('NAO_vision1',client_id,0.0001)
        #Only get the image
        #getVisionSensor('NAO_vision1',client_id)

    else:
        print 'Connection non successful'
        sys.exit('Could not connect')
