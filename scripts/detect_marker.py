#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import tf
from tf.broadcaster import TransformBroadcaster
import tf_conversions
from mycobot_ros.srv import (
    GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus)


class ImageConverter:
    def __init__(self):
        self.br = TransformBroadcaster()
        self.bridge = CvBridge()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.aruo_params = cv.aruco.DetectorParameters_create()
        calibrationParams = cv.FileStorage('calibrationFileName.xml', cv.FILE_STORAGE_READ)
        self.dist_coeffs = calibrationParams.getNode('distCoeffs').mat()
        self.camera_matrix = None
        # rospy.wait_for_service('get_joint_coords')
        # rospy.wait_for_service('set_joint_coords')

        # try:
        #     self.get_coords = rospy.ServiceProxy('get_joint_coords', GetCoords)
        #     self.set_coords = rospy.ServiceProxy('set_joint_coords', SetCoords)
        # except:
        #     print('Error: cannot connect service...')
        #     exit(1)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback)


    def callback(self, data):
        '''Callback function.

        Process image with OpenCV, detect Mark to get the pose. Then acccording the
        pose to transforming.
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        size = cv_image.shape
        focal_length = size[1]
        center = [size[1]/2, size[0]/2]
        if self.camera_matrix is None:
            self.camera_matrix = np.array([
                [focal_length,0,center[0]],
                [0, focal_length, center[1]],
                [0,0,1],
            ], dtype=np.float32)
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        corners, ids, rejectImaPoint = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruo_params)
        if len(corners) > 0:
            if ids is not None:
                # print('corners:', corners, 'ids:', ids)
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
                (rvec - tvec).any()

                print('rvec:', rvec, 'tvec:', tvec)

                for i in range(rvec.shape[0]):
                    cv.aruco.drawDetectedMarkers(cv_image, corners)
                    cv.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[i, :, :], tvec[i, :, :], 0.03)

                # Just process first one detected.
                xyz = tvec[0, 0, :]
                xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.01]

                euler = rvec[0, 0, :]
                tf_change = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
                print('tf_change:', tf_change)

                self.br.sendTransform(xyz, tf_change, rospy.Time.now(), 'basic_shapes', 'joint6_flange' )

        # [x, y, z, -172, 3, -46.8]
        cv.imshow("Image", cv_image)

        cv.waitKey(3)
        try:
            pass
        except CvBridgeError as e:
            print e
if __name__ == '__main__':
    try:
        rospy.init_node("detect_marker")
        rospy.loginfo("Starting cv_bridge_test node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv.destroyAllWindows()
