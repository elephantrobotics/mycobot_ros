# ecoding=utf-8
import cv2
import numpy as np
from imutils import contours
import math

'''HSV中的颜色空间
https://blog.csdn.net/wsp_1138886114/article/details/80660014
'''
color_dist = {'red': {'Lower': np.array([0, 120, 120]), 'Upper': np.array([6, 255, 255])},
              # 'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              # 'blue':{'Lower': np.array([100,43,46]),'Upper': np.array([124,255,255])},
              'blue':{'Lower': np.array([100,43,46]),'Upper': np.array([124,255,255])},
              "cyan": {'Lower': np.array([78, 43, 46]),'Upper': np.array([99, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              'yellow': {"Lower": np.array([22, 93, 0]), "Upper": np.array([45, 255, 255])},
              # 'orange': {"Lower": np.array([0, 100, 45]), "Upper":np.array([255, 250, 255])},
              'orange': {"Lower": np.array([11, 43, 46]), "Upper":np.array([25, 255, 255])},
              }

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("src/test_1_2.mp4")
cap.set(3,420)
cap.set(4,360)

cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)
# 内核
kernel = np.ones((5, 5), np.uint8)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        if frame is not None:

            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)# 高斯模糊
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
            erode_hsv = cv2.erode(hsv, None, iterations=2)                   # 腐蚀 作用是粗的变细
            for _color in color_dist:
                _font_x_pos = 0
                _font_y_pos = 0
                ## 颜色阈值识别
                inRange_hsv = cv2.inRange(erode_hsv, color_dist[_color]['Lower'], color_dist[_color]['Upper'])
                # 膨胀操作
                dilation = cv2.dilate(inRange_hsv, kernel, iterations=1)
                # 闭操作
                closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
                # 边缘检测
                edges = cv2.Canny(closing, 10, 20)
                # 检测物体边框
                cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                
                # 判断轮廓数量也就是判断是否寻找到轮廓，如果没有找到轮廓就不继续进行操作
                if len(cnts) > 0:
                    for cnt in cnts:
                        if cv2.contourArea(cnt) >1500:
                            peri = cv2.arcLength(cnt,True)
                            #print(peri)
                            # 用于获得轮廓的近似值，使用cv2.drawCountors进行画图操作 ，
                            # 参数说明：cnt为输入的轮廓值， epsilon为阈值T，通常使用轮廓的周长作为阈值，True表示的是轮廓是闭合的
                            '''
                            cv2.approxPolyDP
                            @:param
                                cnt:
                                epsilon :算法参数
                                True：表示是否闭合
                            '''
                            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
                            # print(len(approx))
                            # 提取拐点
                            '''
                            返回列表元素，列表中的元素代表一个边沿信息。
                            '''
                            objCor = len(approx)

                            if objCor ==3:
                                '''https://blog.51cto.com/hiszm/5201991'''
                                objectType = "Triangle"
                                mm = cv2.moments(cnt)
                                cx = int(mm['m10']/mm['m00'])
                                cy = int(mm['m01']/mm['m00'])
                                _font_x_pos = cx
                                _font_y_pos = cy
                                cv2.circle(frame,(cx,cy),3,(0,0,255),-1)
                                cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 3)
                                print("三角形的坐标",cx,cy)


                            # 检测出是矩形
                            elif objCor == 4:
                                # 可旋转矩形，即最小的外包矩形
                                rect= cv2.minAreaRect(cnt)
                                '''
                                @:param
                                函数 cv2.minAreaRect() 返回一个Box2D结构 rect：（最小外接矩形的中心（x，y），（宽度，高度），旋转角度）。
                                分别对应于返回值：(rect[0][0],  rect[0][1]),  (rect[1][0],  rect[1][1]),  rect[2]
                                '''
                                # 中心点坐标
                                pos_x = int(rect[0][0])
                                pos_y = int(rect[0][1])
                                #print("position",pos_x,pos_y)
                                # 旋转角度
                                theta = np.round(cv2.minAreaRect(cnt)[2],2)
                                box = cv2.boxPoints(rect)
                                box = np.int0(box)
                                print("物体的坐标为",(pos_x,pos_y),"旋转角度为",theta)
                                # 判断长方形还是正方形
                                _font_x_pos = box[0][0]
                                _font_y_pos = box[0][1]
                                _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                                _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                                # 长宽比
                                aspRatio = _W/float(_H)
                                if aspRatio >0.98 and aspRatio <1.03:

                                    objectType= "Square"#正方形
                                else:
                                    objectType="Rectangle"#长方形
                                _pos = (pos_x,pos_y)
                                cv2.circle(frame,_pos,1,(0,0,255),2)#绘制中心点 
                                #cv2.putText(frame, 'teheta = ' + str(theta), (int(_font_x_pos),int(_font_y_pos+20)), cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
                                cv2.drawContours(frame,[box],0,(0,0,255),2)
                                # 圆形
                            elif objCor>4:
                                objectType= "Circles"
                                '''
                                void minEnclosingCircle(InputArray points, Point2f& center, float& radius)
                                @:param
                                    points：输入信息，可以为包含点的容器(vector)或是Mat。
                                    center：包覆圆形的圆心。
                                    radius：包覆圆形的半径。 
                                '''
                                (x,y),radius = cv2.minEnclosingCircle(cnt)
                                center = (int(x),int(y))
                                radius = int(radius)
                                _font_x_pos = center[0]
                                _font_y_pos = center[1]
                                print("物体的圆心为:",(_font_x_pos, _font_y_pos),"半径为",radius)
                                cv2.circle(frame,center,1,(255,0,0),2)#绘制中心点
                                cv2.circle(frame,center,radius,(255,0,0),2)

                            else:
                                objectType="None"
                            # objectType 物体形状
                            print("颜色类别：",_color)
                            cv2.putText(frame, str(_color + objectType), (int(_font_x_pos),int(_font_y_pos)), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 255, 0) )

            cv2.imshow('camera', frame)
            # 存储识别结果视频
            # out.write(frame)

            cv2.waitKey(1)
            if cv2.waitKey(10) & 0xFF == 27:
                break
        else:
            print("无画面")
            break
    else:
        print("无法读取摄像头！")
        break

cap.release()
# out.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
