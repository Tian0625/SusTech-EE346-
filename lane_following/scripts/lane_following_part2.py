#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                H=numpy.array([[-0.434, -1.33, 229],[-0.0, -2.88, 462],[-0.0, -0.00833, 1.0]])
                BEV=cv2.warpPerspective(image,H,(320,240))
                hsv2=cv2.cvtColor(BEV, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])

                lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 43, 220])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                mask3 = cv2.inRange(hsv2, lower_yellow, upper_yellow)
                mask4 = cv2.inRange(hsv2, lower_white, upper_white)


                h, w, d = image.shape
                search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0
                mask3[0:0, 0:w] = 0
                mask4[0:0, 0:w] = 0
                M3 = cv2.moments(mask3)
                M4 = cv2.moments(mask4)

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M3['m00'] > 0 and M4['m00'] > 0:
                    cx1 = int(M3['m10']/M3['m00'])
                    cy1 = int(M3['m01']/M3['m00'])

                    cx2 = int(M4['m10']/M4['m00'])
                    cy2 = int(M4['m01']/M4['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.4
                    self.twist.angular.z=err/120
                    if abs(err)>16:
                       self.twist.linear.x=0.3
                       self.twist.angular.z = err*5/100
                    print(err)
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(1)
                
                cv2.imshow('BEV',BEV)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
