from robomaster import robot
import cv2
import time
import numpy as np

def read_img(ep_camera): #อ่านภาพจากกล้อง
    cap_img = ep_camera.read_cv2_image(strategy ="newest")
    return cap_img

def mask_img(ep_camera):
    cap_img = read_img(ep_camera)
    hsv_img = cv2.cvtColor(cap_img, cv2.COLOR_BGR2HSV)
    lower_y_and_o = np.array([5,145,130])
    upper_y_and_o = np.array([50,255,255])
    list_colors = [(lower_y_and_o, upper_y_and_o)]
    str_color = ["yellow and orange"]
    for idx_color,(low, high) in enumerate(list_colors):
        mask_y_and_o = cv2.inRange(hsv_img, low, high)
    
    return cap_img, mask_y_and_o

def contours(cap_img, mask_y_and_o):
    global ap,dp
    height, width = mask_y_and_o.shape
    min_x1, min_y1 = width, height
    max_x1 = max_y1 = 0

    mask_y_and_o[:4*height//10, :] = 0
    mask_y_and_o[8*height//10:, :] = 0

    contours1, hierarchy1 = cv2.findContours(mask_y_and_o, \
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    try: hierarchy1 = hierarchy1[0]
    except: hierarchy1 = []

    for contour1, hier1 in zip(contour1, hierarchy1):
        (x1,y1,w1,h1) = cv2.boundingRect(contour1)
        min_x1, max_x1 = min(x1, min_x1), max(x1+w1, max_x1)
        min_y1, max_y1 = min(y1, min_y1), max(y1+w1, max_y1)
        y_and_o_center = [x1+w1//2, y1+h1//2]
        if 400 < x1 < 800 and y1 > 400:
            if w1 > 25 and h1 > 25:
                cv2.rectangle(cap_img, (x1,y1), (x1+w1,y1+h1), (255, 0, 0), 2)
                cv2.putText(cap_img, f"({y_and_o_center})", \
                            (x1+w1+10, y1+h1//2+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                if h1 > w1:
                    status = "Alive"
                    cv2.putText(cap_img, f"{status}", (x1, y1+h1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    ap = y_and_o_center[0]
                elif h1 < w1:
                    status = "Dead"
                    cv2.putText(cap_img, f"{status}", (x1, y1+h1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    dp = y_and_o_center[0]
    return ap, dp

def math_center(cap_img, mask_y_and_o, dp):
    global fl, fr, l, r
    delta_x = 640 - int(dp)

    while abs(delta_x) > 15:
        if delta_x > 15:
            #move left
            ep_chassis.drive_wheels(w1 = fl, w2 = fr*-1, w3 = l, w4 = r*-1)
            time.sleep(0.2)
            ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)

        elif delta_x < 15:
            #move right
            ep_chassis.drive_wheels(w1 = fl*-1, w2 = fr, w3 = l*-1, w4 = r)
            time.sleep(0.2)
            ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)
        
        cap_img, mask_y_and_o = mask_img(ep_camera)
        ap, dp = contours(cap_img, mask_y_and_o)
        delta_x = 640 - dp
        print("death_point", dp)
        print(f"delta x = {delta_x}")

def sub_data_handler(sub_info):
    global tof
    distance = sub_info
    tof = int(distance[0])

def read_sensor():
    ep_sensor.sub_distance(freq = 5, callback = sub_data_handler)
    time.sleep(2)
    ep_sensor.unsub_distance()

def move_to_obj():
    global fl, fr, l, r, tof
    read_sensor()
    print(f"tof1 = {tof}")
    while tof > 650:
        ep_chassis.drive_wheels(w1 = fl*1.2, w2 = fr*1.2, w3 = l*1.2, w4 = r*1.2)  
        read_sensor()
        print(f"tof2 = {tof}")
    ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)

def servo_2():
    for i in range(0, 9):
        ep_servo.moveto(index = 2, angle = -10*i).wait_for_complete()
        if i == 60:
            gripper_open()
            time.sleep(1)

def servo_1():
    for i in range(0, 9):
        ep_servo.moveto(index = 1, angle = i).wait_for_complete()

def gripper_open():
    ep_gripper.open()
    time.sleep(2)
    ep_gripper.pause()

def gripper_close():
    ep_gripper.close()
    time.sleep(1)

def servo_recenter(servo):
    if servo == 1:
        for j in range(55, 29, -5):
            ep_servo.moveto(index = 1, angle = j).wait_for_complete()
    elif servo == 2:
        for i in range(-80, 1, 20):
            ep_servo.moveto(index = 2, angle = i).wait_for_complete()

def pickup_obj():
    servo_2()
    servo_1()
    time.sleep(0.5)
    gripper_open()

    ep_chassis.drive_wheels(w1 = 15, w2 = 15, w3 = 15, w4 = 15)
    time.sleep(4)
    ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)

    gripper_close()
    time.sleep(0.5)

    servo_recenter(1)
    servo_recenter(2)
    time.sleep(0.5)
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type = "ap")
    ep_camera = ep_robot.camera
    ep_servo = ep_robot.servo
    ep_gripper = ep_robot.gripper
    ep_chassis = ep_robot.chassis
    ep_camera.start_video_stream(display = False)
    ep_sensor = ep_robot.sensor

    fl, fr, l, r = 15, 15, 15, 15
    tof = 10000
    dp = ap = 0

    print("START")
    gripper_open()
    gripper_close()
    time.sleep(0.5)
    servo_recenter(2)
    servo_recenter(1)
