# encoding:utf-8
#!/usr/bin/env python2

import time
import cv2
import detect_obj_img

class Detect_Img(detect_obj_img.Object_detect):
    def __init__(self, camera_x=150, camera_y=-10):
        super().__init__(camera_x, camera_y)
        self.Pin = [20, 21]
        for i in self.move_coords:
            i[2] -= 20
        
    def move(self, x, y, color):
        # send Angle to move mycobot
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[1], 20)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[2], 20)
        time.sleep(1.5)
        # send coordinates to move mycobot
        self.pub_coords([x, y, 165,  -178.9, -1.57, -66], 20, 1)
        time.sleep(1.5)
        h = 0
        if 165 < x < 180:
            h = 10
        elif x > 180:
            h = 20
        elif x < 135:
            h = -20
        self.pub_coords([x, y, 31.9+h,  -178.9, -1, -66], 20, 1)
        time.sleep(1.5)
        # open pump
        if self.raspi:
            self.gpio_status(True)
        else:
            self.pub_pump(True, self.Pin)
        time.sleep(0.5)
        self.pub_angles(self.move_angles[2], 20)
        time.sleep(3)
        self.pub_marker(
            self.move_coords[2][0]/1000.0, self.move_coords[2][1]/1000.0, self.move_coords[2][2]/1000.0)

        self.pub_angles(self.move_angles[1], 20)
        time.sleep(1.5)
        self.pub_marker(
            self.move_coords[3][0]/1000.0, self.move_coords[3][1]/1000.0, self.move_coords[3][2]/1000.0)

        self.pub_angles(self.move_angles[0], 20)
        time.sleep(1.5)
        self.pub_marker(
            self.move_coords[4][0]/1000.0, self.move_coords[4][1]/1000.0, self.move_coords[4][2]/1000.0)
        print self.move_coords[color]
        self.pub_coords(self.move_coords[color], 20, 1)
        self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
                        [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(2)
        # close pump
        self.pub_pump(False, self.Pin)
        time.sleep(1)
        if color == 1:
            self.pub_marker(
                self.move_coords[color][0]/1000.0+0.04, self.move_coords[color][1]/1000.0-0.02)
        elif color == 0:
            self.pub_marker(
                self.move_coords[color][0]/1000.0+0.03, self.move_coords[color][1]/1000.0)
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(3)
        
    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            if (y < -30 and x > 140) or (x > 150 and y < -10):
                x -= 10
                y += 10
            elif y > -10:
                y += 10
            elif x > 170:
                x -= 10
                y += 10
            print x, y
            self.move(x, y, color)
            
def run():

    # Object_detect().take_photo()
    # Object_detect().cut_photo()
    # goal = Object_detect().distinguist()
    goal = []
    path = os.getcwd()+'/local_photo/img'

    for i, j, k in os.walk(path):
        for l in k:
            goal.append(cv2.imread('local_photo/img/{}'.format(l)))
    cap_num = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot
    detect.run()

    _init_ = 20  #
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)

        if _init_ > 0:
            _init_ -= 1
            continue
        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print "ok"
            continue
        # get detect result
        detect_result = detect.obj_detect(frame, goal)
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot
            real_x, real_y = detect.get_position(x, y)
            if num == 5:
                detect.pub_marker(real_sx/5.0/1000.0, real_sy/5.0/1000.0)
                detect.decide_move(real_sx/5.0, real_sy/5.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x

        cv2.imshow("figure", frame)


if __name__ == "__main__":
    run()