from ultralytics import YOLO
import cv2 as cv
import math
import time
import cv2
from robomaster import robot
from robomaster import camera
import numpy as np
import threading
model = YOLO(r"D:\241-251\RoboMaster-SDK\e200.pt")

def show_bb():
    global x_alive,x_dead,y_dead,w_dead,h_dead,x_center_chicken_dead,y_center_chicken_dead,y_center_chicken_alive,y_center_chicken_alive,y_alive,w_alive,h_alive
    global x_center_chicken_alive,ap,dp
    while True:
        img = ep_camera.read_cv2_image(strategy="newest")
        image = img.copy()
        # cv2.circle(image, (image_x, image_y), 5, (0, 0, 255), -1)
        # image_left = image_x/2
        m_dead = None
        m_alive = None
        h1_dead = 0
        h1_alive = 0
        pre = model.predict(image, conf=0.6,verbose = False)
        results = pre[0]
        boxlist = results.boxes
        cls = boxlist.cls
        # coordinates = boxlist.xyxy
        lst_dead = []
        lst_alive = []
        for i in range(len(cls)):
            if cls[i] == 1:  # 1 ไก่ ตาย
                cor_dead = boxlist[i].xyxy
                cor_dead = cor_dead.tolist()
                lst_dead.append(cor_dead[0])
            if cls[i] == 0: 
                cor_alive = boxlist[i].xyxy
                cor_alive = cor_alive.tolist()
                lst_alive.append(cor_alive[0])
        for i in lst_dead:
            # print(i)
            y_dead = int(i[1])
            h_dead = int(i[3]- y_dead)
            if h_dead>h1_dead:
                h1_dead=h_dead
                m_dead = i
        for i in lst_alive:
            # print(i)
            y_alive = int(i[1])
            h_alive = int(i[3]- y_alive)
            if h_alive > h1_alive:
                h1_alive = h_alive
                m_alive = i
        if m_dead is not None:
            if state == 0:
                status = 'DEAD'
                h_dead_test = int(m_dead[3] - y_dead)
                if h_dead_test>70:
                    x_dead = int(m_dead[0])
                    y_dead = int(m_dead[1])
                    w_dead = int(m_dead[2] - x_dead)
                    h_dead = int(m_dead[3] - y_dead)
                    x_center_chicken_dead = (w_dead // 2) + x_dead
                    y_center_chicken_dead = (h_dead // 2) + y_dead
                    xy_center_chicken_dead = [x_dead+w_dead//2 , y_dead+h_dead//2]
                    dp = xy_center_chicken_dead[0]
                else:
                    x_dead = 0
            else :
                x_dead = 0
        else :
            x_dead = 0
        if m_alive is not None:
            status = 'ALIVE'
            h_alive_test = int(m_alive[3] - y_alive)
            if h_alive_test>75:
                x_alive = int(m_alive[0])
                y_alive = int(m_alive[1])
                w_alive = int(m_alive[2] - x_alive)
                h_alive = int(m_alive[3] - y_alive)
                x_center_chicken_alive = (w_alive // 2) + x_alive
                y_center_chicken_alive = (h_alive // 2) + y_alive
                xy_center_chicken_alive = [x_alive+w_alive//2 , y_alive+h_alive//2]
                ap = xy_center_chicken_alive[0]
            else:
                x_alive = 0
        else:
            x_alive = 0

def detect_green_carpet(image):
    global max_contour,xg,yg,wg,hg,cenx_green
    carpet = image
    hsv_chick = cv.cvtColor(carpet, cv.COLOR_BGR2HSV)
    lower_green = np.array([66, 125, 69])
    upper_green = np.array([110, 255, 245])
    green_mask = cv.inRange(hsv_chick, lower_green, upper_green)
    clean_chick = cv.bitwise_and(carpet, carpet, mask=green_mask)
    gray_chick = cv.cvtColor(clean_chick, cv.COLOR_BGR2GRAY)

    _, c_threshold = cv.threshold(gray_chick, 1, 255, cv.THRESH_BINARY)

    # Finding contours in the binary image
    contours, _ = cv.findContours(c_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Finding the largest contour
    max_area = 0
    max_contour = None
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt

    # Drawing a bounding rectangle around the largest contour
    if max_contour is not None:
        x, y, w, h = cv.boundingRect(max_contour)
        if w>150 and h>150:
            xg, yg, wg, hg = cv.boundingRect(max_contour)
            # print(xg)
            cenx_green = (xg)+(wg//2)
            ceny_green = (yg)+(hg//2)
            cv.rectangle(carpet, (xg, yg), (xg + wg, yg + hg), (0, 255, 0), 1)
            cv.circle(carpet, (cenx_green,ceny_green), 1, (0, 255, 0), 2)
        else:
            xg = 0
    else:
        xg  = 0
    return carpet

def obstacle():
    while 440<x_alive<840:
        cen = image_x-x_alive
        if cen<=0:
            if x_alive<840:
                print('1111')
                ep_chassis.drive_wheels(w1=20, w2=-20, w3=20, w4=-20)
                if adc_right>320:
                    ep_chassis.drive_wheels(w1=-20, w2=20, w3=-20, w4=20)
                    time.sleep(2.5)
                time.sleep(0.1)
        elif cen>0:
            if x_alive>440:
                ep_chassis.drive_wheels(w1=-20, w2=20, w3=-20, w4=20)
                if adc_left>500:
                    ep_chassis.drive_wheels(w1=20, w2=-20, w3=20, w4=-20)
                    time.sleep(2.5)
                time.sleep(0.1)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
 
 
def servo_gripper_standart():
    ep_servo.moveto(index=1, angle=-60).wait_for_completed()
    ep_servo.moveto(index=2, angle=60).wait_for_completed()
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

def servo_highest():
    ep_servo.moveto(index=1, angle=-30).wait_for_completed()
    ep_servo.moveto(index=2, angle=60).wait_for_completed()

def servo_mid() :
    ep_servo.moveto(index=2, angle=60).wait_for_completed()
    ep_servo.moveto(index=1, angle=-90).wait_for_completed()

def servo_lowest() :
    ep_servo.moveto(index=1, angle=-110).wait_for_completed() 
    ep_servo.moveto(index=2, angle=90).wait_for_completed()

def gripper_close() :
    global state,servo_2,servo_1
    ep_gripper.close(power=100) 
    time.sleep(2)
    ep_gripper.pause()
    state = 1
    servo_1 = -60
    servo_2 = 60

def gripper_open() :
    ep_gripper.open(power=100) 
    time.sleep(2)
    ep_gripper.pause()

def match_center_xy(x_center_chicken_dead, y_center_chicken_dead): 
    global fl, fr, l, r, delta_x, delta_y  
    time.sleep(0.00001)
    delta_x =   int(x_center_chicken_dead) -640
    delta_y = int(y_center_chicken_dead)- 360
    if -30 < delta_x < 30: 
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        servooo(delta_y)
        time.sleep(0.01)
    elif  delta_x < -20:
        ep_chassis.drive_wheels(w1=fl, w2=fr * -1, w3=0, w4=0)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    elif delta_x > 20:
        ep_chassis.drive_wheels(w1=fl * -1, w2=fr, w3=0, w4=0)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def servooo(delta_y):
    global servo_1, servo_2
    delta_y =  int(y_center_chicken_dead) -360
    if delta_y < 5: 
        servo_2 -= 1
    if delta_y > 5:
        servo_2 += 1
    if servo_2 > 74 :
        servo_1 = -110
        servo_2 = 75  
    ep_servo.moveto(index=1, angle=servo_1).wait_for_completed()
    ep_servo.moveto(index=2, angle=servo_2).wait_for_completed() 
    # print(servo_1,servo_2)

def bumper_adc():
    print(adc_right)
    while adc_left > 500 or adc_right > 320:
        if adc_left > 500 :
            ep_chassis.drive_wheels(w1=20, w2=-20, w3=0, w4=0)
            time.sleep(0.1)
            ep_chassis.drive_wheels(w1=20, w2=20, w3=20, w4=20)
            time.sleep(0.1)
        elif adc_right > 320 :
            ep_chassis.drive_wheels(w1=-20, w2=20, w3=0, w4=0)
            time.sleep(0.1)
            ep_chassis.drive_wheels(w1=20, w2=20, w3=20, w4=20)
            time.sleep(0.1)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    
def move_and_pickup():
    speed = 25
    distance_x = abs(640 - x_center_chicken_dead)  
    distance_y = abs(360 - y_center_chicken_dead)  
    distances = (distance_x * 2 + distance_y * 2) ** 0.5  
    if distances > 10 and state != 1: 
        match_center_xy(x_center_chicken_dead,y_center_chicken_dead)
    if distances < 10 :
        # bumper_adc()
        ep_chassis.drive_wheels(w1 = speed, w2 = speed, w3 = speed, w4 = speed)
        time.sleep(0.08)
        if servo_1 == -110 and servo_2 == 75:
            ep_servo.moveto(index=2, angle=90).wait_for_completed() 
            ep_chassis.drive_wheels(w1 = 15, w2 = 15, w3 = 15, w4 = 15)
            time.sleep(2)
            ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)
            servo_lowest
            gripper_close()
            servo_mid()
            servo_highest()

def move_and_drop():
    global state ,servo_1,servo_2
    # global x_center_chicken_dead,y_center_chicken_dead
    # time.sleep(0.00001)
    # speed = 25
    del_x = cenx_green - 640
    # del_y = int(yg- 360)
    print(del_x)
    if -30 < del_x < 30: 
        if yg-360 > 70:
            ep_chassis.drive_wheels(w1 = 25, w2 = 25, w3 = 25, w4 = 25)
            time.sleep(1.5)
            ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)
            ep_gripper.open(power=100)
            time.sleep(2)
            ep_chassis.drive_wheels(w1 = -25, w2 = -25, w3 = -25, w4 = -25)
            time.sleep(1)
            ep_chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
            state = 0 
            servo_1 = -60
            servo_2 = 60
            time.sleep(1)
        else:
            ep_chassis.drive_wheels(w1 = 15, w2 = 15, w3 = 15, w4 = 15)
    elif  del_x > 20:
        print('555')
        ep_chassis.drive_wheels(w1=-20, w2=20, w3=0, w4=0)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    elif del_x < -20:
        print('111')
        ep_chassis.drive_wheels(w1=20, w2=-20, w3=0, w4=0)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def sub_data_handler(sub_info):
    global tof,adc_left,adc_right
    distance = sub_info
    tof = int(distance[0])
    adc_left = ep_sensor_adaptor.get_adc(id=1, port=2)
    adc_right = ep_sensor_adaptor.get_adc(id=2, port=1 )


def putdown_chicken():
    global state, dp, servo_1, servo_2, ap, state_alive
    time.sleep(0.0001)
    servo_mid()
    servo_lowest()
    gripper_open()
    servo_mid()
    servo_gripper_standart()
    state = 0
    state_alive = 0
    dp = 0
    ap = 0
    servo_1 = -60
    servo_2 = 60



def main():
    global state
    while True :
        time.sleep(0.00001)
        if state == 0 and x_dead == 0: #ยังไม่จับ ยังไม่เจอ
            # print('state1')
            obstacle()
            bumper_adc()
            if tof > 450 :
                ep_chassis.drive_wheels(w1 = 70, w2 = 70, w3 = 70, w4 = 70)
            else:
                ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)
                ep_chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
        elif state == 0 and x_dead !=0 :
            # print('state 2')
            move_and_pickup()

        elif state == 1 and xg==0:
            # print('state 3')
            obstacle()
            bumper_adc()
            if tof > 450 :
                ep_chassis.drive_wheels(w1 = 70, w2 = 70, w3 = 70, w4 = 70)
            else: 
                ep_chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()

        elif state == 1 and xg != 0 :
            move_and_drop()


                

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    ep_servo = ep_robot.servo
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_sensor.sub_distance(freq=5,callback=sub_data_handler)
    ep_gripper.open(power=100) 
    time.sleep(2)
    servo_1 = -60
    servo_2 = 60
    image_x = 640
    image_y = 360
    ep_servo.moveto(index=1, angle=servo_1).wait_for_completed()
    ep_servo.moveto(index=2, angle=servo_2).wait_for_completed()
    status = "waiting"
    state = 0
    fl, fr, l, r = 13, 13, 13, 13
    servo_gripper_standart()
    boundingbox = threading.Thread(target=show_bb)
    move = threading.Thread(target= main)
    boundingbox.start()
    time.sleep(3)
    move.start()
    time.sleep(2)
    while True:
        img = ep_camera.read_cv2_image(strategy="newest")
        if x_dead != 0:
            cv.rectangle(img, (x_dead, y_dead), (x_dead + w_dead, y_dead + h_dead), (220, 110, 120), 1)
            cv2.putText(img, f"{status}", (x_dead, y_dead + h_dead + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv.circle(img, (x_center_chicken_dead, y_center_chicken_dead), 1, (0, 255, 0), 2)
        if x_alive != 0:
            cv.rectangle(img, (x_alive, y_alive), (x_alive + w_alive, y_alive + h_alive), (220, 110, 120), 1)
            cv2.putText(img, "kaipen", (x_alive, y_alive + h_alive + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv.circle(img, (x_center_chicken_alive, y_center_chicken_alive), 1, (0, 255, 0), 2)
        cv2.circle(img, (int(440),image_y) , 5, (0,0,255), -1)
        cv2.circle(img, (int(840),image_y) , 5, (0,0,255), -1)
        cv2.circle(img, (image_x, image_y), 5, (0, 0, 255), -1)
        img = detect_green_carpet(img)
        cv.imshow('Yellow Object with Bounding Box', img)
        cv.waitKey(1)
 