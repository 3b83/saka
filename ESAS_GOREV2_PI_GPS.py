#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
from pymavlink import mavutil
import cv2
import numpy as np
import RPi.GPIO as GPIO #simulasyonda GPIO yu sil
from datetime import datetime
"""
#SIMULASYON
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect #connection_string="127.0.0.1:14553"
sitl = None

print('Connecting to vehicle on: %s' % connection_string)

vehicle = connect(connection_string, wait_ready=True)
"""
#GERCEK
timeToWait_long=5
timeToWait_short=2
yukseklik=6

sualma_yukseklik=1
sualma_suresi=5

subirakma_yukseklik=2
subirakma_suresi=10

vehicle = connect('/dev/ttyS0', baud=57600, wait_ready=True)

#DEVRIMDEKI KOORDINATLAR
#direk2 = LocationGlobalRelative(39.8918839, 32.7852309,yukseklik)
#direk1 = LocationGlobalRelative(39.8912438, 32.7856815,yukseklik)
#direk2_sagi= LocationGlobalRelative(39.8919703, 32.7854039, 5) #(45,20)
#direk1_solu= LocationGlobalRelative(39.8912830, 32.7859216, 5) #(-45,20)

#####SAHA KOORDINATLARI
kirmizi_baslangic=LocationGlobalRelative(39.8816222,32.7753764,yukseklik)
kirmizi_bitis=LocationGlobalRelative(39.8818404,32.7755964,yukseklik)
sualma_havuz=LocationGlobalRelative(39.8816365, 32.7752638,yukseklik)
direk2_sagi= LocationGlobalRelative(39.8919703, 32.7854039, yukseklik) #(45,20)
direk1_solu= LocationGlobalRelative(39.8912830, 32.7859216, yukseklik) #(-45,20)
bitis=LocationGlobalRelative(39.8818404,32.7755964,yukseklik)

############ SİM KOORDİNATLAR
"""
sualma_havuz=LocationGlobalRelative(-35.3630378, 149.1650186,5)
kirmizi_baslangic=LocationGlobalRelative(-35.3631272, 149.165073, 5) #(15,15)
kirmizi_bitis=LocationGlobalRelative(-35.3636662, 149.1650723, 5) #(-45,15) 
"""

def suAl(su_alma_suresi):
    in3 = 16
    in4 = 20
    enable = 21
    # GPIO pin schematic setup
    GPIO.setmode(GPIO.BCM) 
    # For GPIO.BCM schmatic pin setup
    GPIO.setup(in3,GPIO.OUT)
    GPIO.setup(in4,GPIO.OUT)
    GPIO.setup(enable,GPIO.OUT)
    # GPIO pin initilazation, su almaya basla
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    p=GPIO.PWM(enable,1000)
    p.start(100)
    # bekle
    print("su aliyorum")
    time.sleep(su_alma_suresi)
    #GPIO pin reset
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    print("su almayi bitirdim")
    GPIO.cleanup()
    return;

def suBosalt(su_bosaltma_suresi):
    in1 = 23
    in2 = 24
    enable = 25
    # GPIO pin schematic setup
    GPIO.setmode(GPIO.BCM) 
    # For GPIO.BCM schmatic pin setup
    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(enable,GPIO.OUT)
    # GPIO pin initilazation, su bosaltmaya basla
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
#    GPIO.output(enable,GPIO.HIGH)
    p=GPIO.PWM(enable,1000)
    p.start(100)  # bekle
    print("su birakiyorum")
    time.sleep(su_bosaltma_suresi)
    #GPIO pin reset
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(enable,GPIO.LOW)
    print("su birakmayi bitirdim")
    GPIO.cleanup()
    return;

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_relativeFrame(dForward, dRight, dDown, yaw_rate, vehicle):
    #Changes position of the vehicle with respect to relative frame of the vehicle.
    #Positions are relative to the vehicle’s current position and heading.
    #I.e x=1,y=2,z=3 is 1m forward, 2m right and 3m Down from the current position.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #MAV_FRAME_BODY_FRD may be used too
        0b0000011111111000,
        dForward, dRight, dDown, #pozisyonlar(metre)
        0, 0, 0,#hizlar(metre/s)
        0, 0, 0,#akselarasyon(fonksiyonsuz)
        0, math.radians(yaw_rate))#yaw,yaw_rate(rad,rad/s)
    vehicle.send_mavlink(msg)

def goto_position_globalFrame(dNorth, dEast, dDown, yaw_rate, vehicle):
    #Changes position of the vehicle with respect to global NED frame.
    #Positions are relative to the vehicle’s EKF Origin in NED frame.
    #The EKF origin is the vehicle’s location when it first achieved a good position estimate (home location).
    #I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000011111111000,
        dNorth, dEast, dDown, #pozisyonlar(metre)
        0, 0, 0,#hizlar(metre/s)
        0, 0, 0,#akselarasyon(fonksiyonsuz)
        0, math.radians(yaw_rate))#yaw,yaw_rate(rad,rad/s)
    vehicle.send_mavlink(msg)

def hoverLOITER(lat, lon, alt):
    #Changes into LOITER and hovers.
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 
        0,
        0,0,0,0, #param1-4, not used
        lat,#param5, target latitude
        lon,#param6, target longidute
        alt) #param7, target altitude
    vehicle.send_mavlink(msg)

def velocity(velocity_x, velocity_y, velocity_z, yaw_rate, vehicle):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000011111000111,
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, math.radians(yaw_rate))
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def getBearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def setYaw(heading,vehicle,relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle

    if heading<0:
        heading = 360 + heading

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto(lat, lon, alt, gotoFunction=vehicle.simple_goto):
    #Difference between simple_goto and goto is simple_goto requires time.sleep() afterwards.
    #goto includes the appropriate timer itself.
    targetLocation=LocationGlobalRelative(lat,lon,alt)
    targetDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.07: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(0.4)

arm_and_takeoff(yukseklik)
time.sleep(timeToWait_long)
homeLocation = vehicle.location.global_relative_frame
vehicle.groundspeed=15

#Direk 2 yi gecme 
############################################
goto(direk2_sagi.lat,direk2_sagi.lon,yukseklik)
time.sleep(timeToWait_short)

#Su bırakma alanının belirlenmesi
##########################################
#Su alma baslangic noktasina gider
goto(kirmizi_baslangic.lat, kirmizi_baslangic.lon, yukseklik)
time.sleep(timeToWait_short)

#Su arama kodu baslar
vehicle.groundspeed=3
vehicle.simple_goto(kirmizi_bitis)

red_found=0
time_count_array=[0,0]
time_index=0
center_offset=100
arama_bitis_noktasi=kirmizi_bitis
arama_baslangic_noktasi=kirmizi_baslangic

kamera=cv2.VideoCapture(0)
width=int(kamera.get(3)) #640, center[0]
height=int(kamera.get(4)) #480, center[1]

out = cv2.VideoWriter("GOREV2_orig"+datetime.now().strftime("%m-%d-%Y_%H-%M-%S")+".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),20,(width,height))
out_final = cv2.VideoWriter("GOREV2_filt"+datetime.now().strftime("%m-%d-%Y_%H-%M-%S")+".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),20,(width,height))

while(not red_found):
    arama_bitisine_uzaklik = get_distance_metres(vehicle.location.global_relative_frame,arama_bitis_noktasi)
    
    ret,video=kamera.read()
    out.write(video)

    hsv=cv2.cvtColor(video,cv2.COLOR_BGR2HSV)
    mask1=cv2.inRange(hsv,(0,100,100),(10,255,255))
    mask2=cv2.inRange(hsv,(160,100,100),(185,255,255))
    mask=cv2.bitwise_or(mask1,mask2)    
    
    #NOISE CLEANING
    nb_comp,output,stats,centroids=cv2.connectedComponentsWithStats(mask,connectivity=8)
    sizes=stats[1:,-1];nb_comp=nb_comp-1
    min_size=20
    maskfinal=np.zeros((output.shape))
    for i in range(0,nb_comp):
        if sizes[i]>=min_size:
            maskfinal[output==i+1]=255
    maskfinal=maskfinal.astype(np.uint8)
    final=cv2.bitwise_and(video,video,mask=maskfinal)

    out_final.write(final)

    #CIRCLE DRAWING
    ret,thresh=cv2.threshold(maskfinal,127,255,0)
    contours,hierarchy=cv2.findContours(thresh,1,2)
    center=(0,0)
    if (len(contours)>0):
        big = max(contours, key=cv2.contourArea)
        biggestArea=cv2.contourArea(big)
        (x,y),radius=cv2.minEnclosingCircle(big)
        center=(int(x),int(y))
        print(center)
        radius=int(radius)
        cv2.circle(maskfinal,center,radius,(255,255,255),-1)
        maskcont,maskhier=cv2.findContours(maskfinal,1,2)
        percent=int((cv2.contourArea(max(maskcont, key=cv2.contourArea))*100)/(640*480))
        radius=int(radius)
        vecx=240-int(y)
        vecy=int(x)-320
        vector=(vecx,vecy)
        cv2.circle(final,center,radius,(0,255,0),7)
        cv2.circle(final,center,3,(255,255,0),7) 

    #Second degree polynomial speed function to avoid overshoot while getting closer to red pool.
    #If red is away from the center, set speed to 3m/s. As getting closer to center, decrease speed.
    speedFunction=(8/(height*height)) * (center[1]*center[1]) - 8/height * center[1] + 3
    vehicle.groundspeed=speedFunction
    #print("Speed= {}\n".format(vehicle.groundspeed))

    #PRINT LOCATION IF CENTERED FOR 3 SECS
    if  width/2-center_offset < center[0] < width/2+center_offset and height/2-center_offset < center[1] < height/2+center_offset:
        print("Centered!")
        hover_lat=vehicle.location.global_frame.lat
        hover_lon=vehicle.location.global_frame.lon
        hover_alt=5
        if vehicle.channels['3'] != 1500:
            vehicle.channels.overrides['3'] = 1500
        print(hover_lat,hover_lon,hover_alt)
        hoverLOITER(hover_lat, hover_lon, hover_alt)
        time_count_array[time_index]=time.time()
        time_index=1
        if(time_count_array[1]-time_count_array[0])>1:
            print("Coordinate saved:")
            red_found=1
            kirmizi_havuz=vehicle.location.global_relative_frame
            print(kirmizi_havuz)
            if vehicle.mode.name!="GUIDED":
                vehicle.mode = VehicleMode("GUIDED")
                vehicle.channels.overrides['3'] = None
            kamera.release()
            out.release()
            out_final.release()
            cv2.destroyAllWindows()
    else:
        #start over if focus is lost
        time_count_array=[0,0]
        time_index=0
        if vehicle.mode.name!="GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.channels.overrides['3'] = None
            #go to ending point if red is located at the bottom half
            if height/2<center[1]<(height) or center==(0,0):
                vehicle.simple_goto(arama_bitis_noktasi)
            else: #go to starting point if red is located at the top half
                #means that red is missed, turn back and find 
                vehicle.simple_goto(arama_baslangic_noktasi)
    
    if arama_bitisine_uzaklik < 1:
        #Geri don, kirmiziyi tekrar ara. Noktaları tersine cevir
        vehicle.simple_goto(kirmizi_baslangic)
        arama_bitis_noktasi=kirmizi_baslangic
        arama_baslangic_noktasi=kirmizi_bitis

    #cv2.imshow("final", final)
    if cv2.waitKey(25) & 0xFF== ord("q"):
        break

vehicle.groundspeed=15
goto(kirmizi_bitis.lat, kirmizi_bitis.lon, yukseklik)
time.sleep(timeToWait_short)

#Direk 1'i gecme   
################################
goto(direk1_solu.lat, direk1_solu.lon, yukseklik)
time.sleep(timeToWait_short)
##############################

vehicle.groundspeed=10

#Su alma havuzuna gitme
goto(sualma_havuz.lat,sualma_havuz.lon,yukseklik)
goto_position_relativeFrame(0, 0, yukseklik-sualma_yukseklik, 0, vehicle) #sualma_yukseklige kadar in
time.sleep(timeToWait_long+3)

suAl(sualma_suresi)
goto_position_relativeFrame(0, 0, -yukseklik, 0, vehicle) #yerde surunmemesi icin ilk yukseklige cik
time.sleep(timeToWait_short)

#########################
#Su birakilacak alana gitme
goto(kirmizi_havuz.lat,kirmizi_havuz.lon,yukseklik)
goto_position_relativeFrame(0, 0, yukseklik-subirakma_yukseklik, 0, vehicle) #sualma_yukseklige kadar in
time.sleep(timeToWait_long+3)

suBosalt(subirakma_suresi)

goto_position_relativeFrame(0, 0, -yukseklik, 0, vehicle) #yerde surunmemesi icin ilk yukseklige cik
time.sleep(timeToWait_short)
####################################

goto(bitis.lat, bitis.lon, yukseklik)
time.sleep(timeToWait_short)

print("LANDING")
vehicle.mode = VehicleMode("LAND")
vehicle.channels.overrides['3'] = None
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
