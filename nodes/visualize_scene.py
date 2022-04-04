#!/usr/bin/env python
import rospy
import message_filters
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
import ros_numpy
from cv_bridge import CvBridge
import tf2_ros

import burg_toolkit as burg


class SceneVisualizer:
    """
    Connects to camera input, detects an aruco board based on cv2.aruco package, projects the given scene into the
    camera image and publishes that.

    ROS Parameters:
    scene_fn - path to scene filename (yaml file)
    camera_frame - if other than None, will also publish the camera pose wrt to the printout frame, provide the name
                   of the camera frame in tf2 here
    printout_frame - which name to give the printout_frame

    Input topics:
    ~input/image - the camera image
    ~input/camera_info - the corresponding camera info

    Output topics:
    ~output/image - the camera image with scene objects projected into the image
    ~output/marker_image - the camera image with recognised markers and frames, for debug purposes

    Transform:
    if camera_frame is not None, will broadcast a transform: printout_Frame -> camera_frame
    """
    def __init__(self):
        rospy.init_node('burg_scene_visualizer', anonymous=True)

        # params
        scene_fn = rospy.get_param('~scene_fn', default=None)  # todo
        self.camera_frame = rospy.get_param('~camera_frame', default=None)
        self.printout_frame = rospy.get_param('~printout_frame', default='printout_frame')

        assert scene_fn is not None, 'no scene_fn given, please provide path to a scene'
        self.scene, lib, printout = burg.Scene.from_yaml(scene_fn)
        # burg.visualization.show_geometries([self.scene])
        print('_'*20)
        print(printout.marker_info.to_dict())

        self.printout_detector = burg.printout.PrintoutDetector(printout)
        self.camera = None
        self.render_engine = None
        self._cv_bridge = CvBridge()

        # transform broadcaster and image publisher
        self.pub_tf = tf2_ros.TransformBroadcaster()
        self.pub_image = rospy.Publisher('~output/image', sensor_msgs.msg.Image, queue_size=10)
        self.pub_marker_image = rospy.Publisher('~output/marker_image', sensor_msgs.msg.Image, queue_size=10)

        image_sub = message_filters.Subscriber('~input/image', sensor_msgs.msg.Image)
        info_sub = message_filters.Subscriber('~input/camera_info', sensor_msgs.msg.CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.process_frame)
        rospy.spin()

    def process_frame(self, image_msg, camera_info_msg):
        camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
        distortion_coeffs = np.array(camera_info_msg.D)
        frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        marker_image = self.printout_detector.detect(frame.copy(), camera_matrix, distortion_coeffs)
        if marker_image is not None:
            camera_pose = self.printout_detector.get_camera_pose_opengl()
            width, height = camera_info_msg.width, camera_info_msg.height

            # we may need to set up the camera/render stuff first
            if self.camera is None or self.camera.resolution != [width, height]:  # also check for camera_matrix??
                self.camera = burg.render.Camera.from_camera_matrix(width, height, camera_matrix)
                self.render_engine = burg.render.PyRenderEngine()
                self.render_engine.setup_scene(self.scene, self.camera, ambient_light=[0.3]*3)

            # overlay rendered bits as fancy Augmented Reality
            color, depth = self.render_engine.render(camera_pose=camera_pose)
            mask = depth != 0
            color = color[:, :, ::-1]  # swap from RGB to BGR
            frame[mask] = frame[mask] * 0.1 + color[mask] * 0.9
            marker_image[mask] = marker_image[mask] * 0.1 + color[mask] * 0.9

            # uncomment this if you want to see the frame directly without rviz
            # import cv2
            # cv2.imshow('frame', marker_image)
            # cv2.waitKey(1)

            # publish frame
            if not rospy.is_shutdown():
                self.pub_image.publish(self._cv_bridge.cv2_to_imgmsg(frame))
                self.pub_marker_image.publish(self._cv_bridge.cv2_to_imgmsg(marker_image))

                if self.camera_frame is not None:  # also publish pose
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.frame_id = self.printout_frame
                    t.child_frame_id = self.camera_frame
                    t.header.stamp = rospy.Time.now()
                    t.transform = ros_numpy.msgify(
                        geometry_msgs.msg.Transform, self.printout_detector.get_camera_pose_cv())
                    self.pub_tf.sendTransform(t)


if __name__ == '__main__':
    try:
        SceneVisualizer()
    except rospy.ROSInterruptException:
        print('SceneVisualizer got interrupted...')

