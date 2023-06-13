# coding:utf-8
from fileinput import filename
import os, cv2, sys


def take_photo():
    # 提醒用户操作字典
    print("************************************************")
    print("*  热键(请在摄像头的窗口使用):                   *")
    print("*  hotkey(please use it in the camera window): *")
    print("*  z: 拍摄图片(take picture)                    *")
    print("*  q: 退出(quit)                               *")
    print("************************************************")

    # 创建/使用local_photo文件夹
    class_name = "res"
    if (os.path.exists("res")):
        pass
    else:
        os.mkdir(class_name)

    # 设置特定值

    index = 'takephoto'
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4, 480)

    while True:
        # 读入每一帧
        ret, frame = cap.read()

        cv2.imshow("capture", frame)

        # 存储
        input = cv2.waitKey(1) & 0xFF
        # 拍照
        if input == ord('z'):
            cv2.imwrite(
                "%s/%s.jpeg" % (class_name, index),
                cv2.resize(frame, (600, 480), interpolation=cv2.INTER_AREA))
            break

        # 退出
        if input == ord('q'):

            # 关闭窗口
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()


def cut_photo():
    
    path = '/home/er/catkin_ws/src/mycobot_ros/mycobot_ai/aikit_320_pi'    # pi

    path_red = path + '/res/A'
    for i, j, k in os.walk(path_red):
        file_len_red = len(k)

    path_gray = path + '/res/B'
    for i, j, k in os.walk(path_gray):
        file_len_gray = len(k)

    path_green = path + '/res/C'
    for i, j, k in os.walk(path_green):
        file_len_green = len(k)

    path_blue = path + '/res/D'
    for i, j, k in os.walk(path_blue):
        file_len_blue = len(k)
    print("请截取要识别的部分")
    print("Please intercept the part to be identified")

    cut = cv2.imread(r"res/takephoto.jpeg")

    cv2.imshow('original', cut)

    # 选择ROI
    roi = cv2.selectROI(windowName="original",
                        img=cut,
                        showCrosshair=False,
                        fromCenter=False)
    x, y, w, h = roi
    print(roi)

    msg = """\
    Image save location:
        1 - 保存至A分拣区文件夹 Save to A folder 
        2 - 保存至B分拣区文件夹 Save to B folder 
        3 - 保存至C分拣区文件夹 Save to C folder 
        4 - 保存至D分拣区文件夹 Save to D folder
        """
    print(msg)
    kw = int(input("请输入保存图片文件夹数字编号(Please enter the number of the folder to save the picture):"))
    # print(kw)

    # 显示ROI并保存图片
    if roi != (0, 0, 0, 0):
        
        crop = cut[y:y + h, x:x + w]
        # cv2.imshow('crop', crop)
        # 选择D区文件夹
        if kw == 1:
            cv2.imwrite(path + '/res/A/goal{}.jpeg'.format(str(file_len_red + 1)),crop)
            print('Saved')
        # 选择B区文件夹
        elif kw == 2:
            cv2.imwrite(path + '/res/B/goal{}.jpeg'.format(str(file_len_gray+1)),crop)
            print('Saved')
        # 选择C区文件夹
        elif kw == 3:
            cv2.imwrite(path + '/res/C/goal{}.jpeg'.format(str(file_len_green+1)),crop)
            print('Saved')
        # 选择A区文件夹
        elif kw == 4:
            cv2.imwrite(path + '/res/D/goal{}.jpeg'.format(str(file_len_blue+1)),crop)
            print('Saved')

    # 退出
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    take_photo()
    cut_photo()
