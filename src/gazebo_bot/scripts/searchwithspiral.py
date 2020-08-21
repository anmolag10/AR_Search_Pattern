
#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped, Twist
import tf
from tf.transformations import euler_from_quaternion
from math import *
from sensor_msgs.msg import Imu, NavSatFix
import time
from pyproj import Geod
#global declarations---------------
yaw = 0
lat1 = 0
lon1 = 0
lat2 = 49.899902563

lon2 = 8.89995389901
ar_id = 0
flag = 0

#-----------------------------------
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()


def imu(pose):
	global yaw
	quaternion = (pose.orientation.x, pose.orientation.y,
	              pose.orientation.z, pose.orientation.w)

	euler = euler_from_quaternion(quaternion)
	yaw = degrees(euler[2])+180
	yaw = abs(yaw-360)
	yaw = yaw % 360


def gps(data):
	global lat1
	lat1 = data.latitude
	global lon1
	lon1 = data.longitude


def markerget(data):
   global ar_id
   ar_id = data.id


def stop():
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = 0
	pub.publish(twist)


def gostraight():
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = -1
	pub.publish(twist)


def goback():
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	twist.linear.x = 1
	pub.publish(twist)


def alignstatic(angle):
	flag = 0
	while 1:
		if flag == 0:
			time.sleep(0.1)
			final_yaw = yaw + angle
			if final_yaw < 0:
				final_yaw = 360+final_yaw
        		if final_yaw > 360:
				final_yaw = final_yaw % 360
			flag = 1
			print(yaw, final_yaw, angle)
		angle_diff = yaw-final_yaw
		if angle_diff < 1 and angle_diff > -1:
				stop()
				break
		if angle > 0:
			twist.angular.z = -1
			pub.publish(twist)
		elif angle < 0:
			twist.angular.z = 1
			pub.publish(twist)


def listener():
	rospy.init_node('bot_yaw', anonymous=True, disable_signals=True)
	rospy.Subscriber("/imu", Imu, imu)
	rospy.Subscriber("/fix", NavSatFix, gps)
	rospy.Subscriber("/camera_1/visualization_marker", Marker, markerget)
	rate = rospy.Rate(100)  # 10hz

        while 1:  # intial alignment to the goal
		geodesic = Geod(ellps='WGS84')
		bearing, reverse_bearing, dist = geodesic.inv(lon1, lat1, lon2, lat2)
		bearing = bearing + 180
		angle_diff = yaw - bearing
		#print("Yaw: ", yaw, "Bearing: ", bearing)
		if angle_diff > 1 or angle_diff < -1:
				print(1)
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 1
				print("Angle_diff:", angle_diff)
				pub.publish(twist)

		if angle_diff < 1 and angle_diff > -1:
			print(dist)
			alignstatic(180)
			stop()
			break
	while 1:  # reaching to the 10 m boundry
		geodesic = Geod(ellps='WGS84')
		bearing, reverse_bearing, dist = geodesic.inv(lon1, lat1, lon2, lat2)
		if dist > 10:
			gostraight()

		else:
			stop()
			break
	flag = 0
	detect=False
 	n=1
	while 1 and detect==False and n<=7:  # aligning to extreme end i.e. 120 degrees
		#detect=False
		if flag == 0:
			alignstatic(-120)
			flag = 1
			if flag == 1:
			  s = rospy.get_time()

			  while(rospy.get_time() < s+10):
					  gostraight()
					  if ar_id == 4:
						  stop()
						  detect=True
						  break
			  stop()
			  s = rospy.get_time()
			  while(rospy.get_time() < s+10 and detect==False):
				     goback()


			  stop()
	
		detect=False
		while n <= 7 and detect==False:
	
			if flag <5:
				if flag ==n:
					alignstatic(48)
					flag = flag+1
					if flag == n+1:
						s = rospy.get_time()
						
						while(rospy.get_time() < s+10):
								gostraight()
								if ar_id ==4:
									stop()
									detect=True
									break
						stop()
						s = rospy.get_time()
						while(rospy.get_time() < s+10 and detect==False):
								goback()
						stop()

				
            
			if flag == 6:
					alignstatic(22.5)
					flag = flag+1
					if flag == 7:
						s = rospy.get_time()

						while(rospy.get_time() < s+10):
								gostraight()
								if ar_id == 4:
									stop()
									detect = True
									break
						stop()
						s = rospy.get_time()
						while(rospy.get_time() < s+10 and detect == False):
								goback()
						stop()
			if flag == 7:
					alignstatic(80)
					flag = flag+1
					if flag == 8:
						s = rospy.get_time()

						while(rospy.get_time() < s+10):
								gostraight()
								if ar_id == 4:
									stop()
									detect = True
									break
						stop()
						s = rospy.get_time()
						while(rospy.get_time() < s+10 and detect == False):
								goback()
						stop()
			n=n+1
		stop()
	if detect==False:
            index=1
            d=2
            while d<=18 and detect==False:
                alignstatic(90)
                s=rospy.get_time()
                while(rospy.get_time() < s+d):
					  gostraight()
					  if ar_id == 4:
						  stop()
						  detect = True
						  break
                stop()
                if index %2 ==0:
                    d=d+4 
                index=index+1   
                
            
      
                

	rospy.spin()


if __name__ == '__main__':
    listener()

