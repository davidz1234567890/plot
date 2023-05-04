#!/usr/bin/env python3
# import turtlesim.msg
# import tf
# import rospy
# import roslib
# roslib.load_manifest('learning_tf')


# def handle_turtle_pose(msg, turtlename):
#     br = tf.TransformBroadcaster()
#     br.sendTransform((msg.x, msg.y, 0),
#                      tf.transformations.quaternion_from_euler(0, 0, msg.theta),
#                      rospy.Time.now(),
#                      turtlename,
#                      "world")


# if __name__ == '__main__':
#     rospy.init_node('turtle_tf_broadcaster')
#     turtlename = rospy.get_param('~turtle')
#     rospy.Subscriber('/%s/pose' % turtlename,
#                      turtlesim.msg.Pose,
#                      handle_turtle_pose,
#                      turtlename)
#     rospy.spin()


# here are some new imports
import numpy as np
import cv2 as cv

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import turtlesim.msg
import tf
import rospy
import time
import roslib
roslib.load_manifest('learning_tf')

# some new code from the plot_150pts.py file
# with np.load('B.npz') as X:
#     mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
#     print(mtx)


# using absolute path
with np.load('/home/huahua/catkin_ws/src/l_tf/nodes/B.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    print(mtx)


criteria = (cv.TERM_CRITERIA_EPS +
            cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*10, 3), np.float32)
objp[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2)
print(objp)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())

    img = cv.line(img, (int(corner[0]), int(corner[1])), (int(
        tuple(imgpts[0].ravel())[0]), int(tuple(imgpts[0].ravel())[1])), (255, 0, 0), 5)
    img = cv.line(img, (int(corner[0]), int(corner[1])), (int(
        tuple(imgpts[1].ravel())[0]), int(tuple(imgpts[1].ravel())[1])), (0, 255, 0), 5)
    img = cv.line(img, (int(corner[0]), int(corner[1])), (int(
        tuple(imgpts[2].ravel())[0]), int(tuple(imgpts[2].ravel())[1])), (0, 0, 255), 5)
    return img


def randrange(n, vmin, vmax):
    return (vmax - vmin)*np.random.rand(n) + vmin
# """
# Helper function to make an array of random numbers having shape (n, )
# with each number distributed Uniform(vmin, vmax).
# """


cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

np.random.seed(19680801)


fig = plt.figure()
#ax = fig.add_subplot(projection='3d')
#ax = fig.add_subplot(1, 2, 1, projection='3d')
ax = fig.add_subplot(111, projection='3d')
#ax = fig.add_subplot()


ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

pts = []


if __name__ == '__main__':
    rospy.init_node('turtle1_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')

    print("!!!")
    print('/%s/pose' % turtlename)

    # rospy.Subscriber('/%s/pose' % turtlename,
    #                  turtlesim.msg.Pose,
    #                  handle_turtle_pose,
    #                  turtlename)
    #print("function is being called")

    br = tf.TransformBroadcaster()
    # br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  turtlename,
    #                  "map")
    # time.sleep(5)
    # br.sendTransform((1, 1, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  turtlename,
    #                  "map")
    # time.sleep(5)
    # br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  turtlename,
    #                  "map")
    #i = 0
    while(True):
        # br.sendTransform((i, i, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, 0),
        #                  rospy.Time.now(),
        #                  turtlename,
        #                  "map")
        # time.sleep(0.5)
        # i += 0.1
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        #cv.imshow('frame', gray)

        #i = 0
        success, corners = cv.findChessboardCorners(
            gray, (10, 7), None)  # new line
        if success == True:  # new conditional
            cv.imshow('img', frame)
            cv.waitKey(1)
            corners2 = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            # Find the rotation and translation vectors.
            # print(corners2)
            ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
            br.sendTransform((tvecs[0]/100, tvecs[1]/100, tvecs[2]/100),
                             tf.transformations.quaternion_from_euler(
                                 rvecs[0], rvecs[1], rvecs[2]),
                             rospy.Time.now(),
                             turtlename,
                             "map")
            # time.sleep(0.5)
            pts.append(tvecs)
            print(len(pts))
            print(rvecs)
            # if (len(pts) == 150):
            #     break
            #distance = np.linalg.norm(tvecs)

        else:
            cv.imshow('img', frame)
            cv.waitKey(1)

        if cv.waitKey(1) == ord('q'):
            break

    rospy.spin()
