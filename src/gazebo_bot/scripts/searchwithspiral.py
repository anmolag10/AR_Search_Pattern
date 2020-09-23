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
#--------------------------------------------------------------------------------------------
yaw = 0
lat1 = 0
lon1 = 0
lat2 = 49.8999196864
lon2 =8.89986477857
ar_id = 0
idlist=[4,5,255]
flag = 0
ar_distance=0
ar_pose=0
#---------------------------------------------------------------------------------------
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

#--------------------------callbacks----------------------------------------------------
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
   global ar_distance
   global ar_pose
   ar_id = data.id
   ar_distance=hypot(data.pose.position.z,data.pose.position.x)
   ar_pose=degrees(atan(data.pose.position.x/data.pose.position.z))
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
	twist.linear.x = -0.5
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

def backdist(d):
	t1=rospy.get_time()
	while(rospy.get_time()-t1<d):
		goback()
	stop()


#----------------------------------------------------------------------------------------
def listener():
	detect=False
	rospy.init_node('bot_yaw', anonymous=True, disable_signals=True)
	rospy.Subscriber("/imu", Imu, imu)
	rospy.Subscriber("/fix", NavSatFix, gps)
	rospy.Subscriber("/camera_1/visualization_marker", Marker, markerget)
	rate = rospy.Rate(100)  # 10hz
	op=0
	flag=0
##--------------------------------------------PUT TRAVERSAL CODE-------------------------------------------------------
	while 1 and detect==False:  # intial alignment to the goal
		geodesic = Geod(ellps='WGS84')
		bearing, reverse_bearing, dist = geodesic.inv(lon1, lat1, lon2, lat2)
		angle_diff=bearing-yaw+360
		pub.publish(twist)
		if op==0:
			print('Alignment Start')
			op+=1
		if angle_diff>0.5:
			twist.angular.z=-1
			pub.publish(twist)
		elif angle_diff<-0.5:
			twist.angular.z=1
			pub.publish(twist)
		else:
			print('Alignment Complete')
			stop()
			break
			#The above alignment is for my testing only#
# --------------------------------------------------------------------------------------------Moving to the 10 m boundry
	bound=False
	while 1 and detect==False and bound==False:
		geodesic = Geod(ellps='WGS84')
		bearing, reverse_bearing, dist = geodesic.inv(lon1, lat1, lon2, lat2)
		while ar_id!=idlist[0] and dist>2:
			geodesic = Geod(ellps='WGS84')
			bearing, reverse_bearing, dist = geodesic.inv(lon1, lat1, lon2, lat2)
			print(dist)
			print("Going to destination")
			gostraight()
		stop()
		if ar_id==idlist[0]:
			if ar_id==4:
				print "Tag seen"						
				alignstatic(ar_pose)
				while ar_distance>2:
					gostraight()
				stop()
				print "Tag reached"
				while(ar_id!=4):
					twist.angular.z=0.5
				detect=True
				break
		else:
				stop()
				bound=True
				break
	flag=0
	n=1
	while 1 and detect==False:
		print('Loop  1')
		if flag==0:
			alignstatic(-120)
			flag+=1
		if flag ==1:
			t1=rospy.get_time()
			while ar_id!=4 and rospy.get_time()<t1+20:
				gostraight()
			stop()
			if ar_id==4:
				print "Tag seen"						
				alignstatic(ar_pose)
				while ar_distance>2:
					gostraight()
				stop()
				print "Tag reached"
				while(ar_id!=4):
					twist.angular.z=0.5
				detect=True
				break
			else:
			  backdist(10)
			  stop()
			  break
	while  flag<5 and detect==False:
			print("Secondloop")
			print(flag)
			if flag ==n:
					alignstatic(48)
					flag = flag+1
					if flag == n+1:
						t1=rospy.get_time()
						while ar_id!=4 and rospy.get_time()<t1+20:
							gostraight()
						stop()
						if ar_id==4:
							print "Tag seen"						
							alignstatic(ar_pose)
							while ar_distance>2:
								gostraight()
							stop()
							print "Tag reached"
							while(ar_id!=4):
								twist.angular.z=0.5
							detect=True
							break
						else:
							backdist(10)
							stop()
			n=n+1
	if flag ==5 and detect==False:
		print('Third Loop')
		alignstatic(22.5)
		flag = flag+1
		if flag == n+1:
			t1=rospy.get_time()
			while ar_id!=4 and rospy.get_time()<t1+20:
				gostraight()
			stop()
			if ar_id==4:
				print "Tag seen"						
				alignstatic(ar_pose)
				while ar_distance>5:
					gostraight()
				stop()
				print "Tag reached"
				while(ar_id!=4):
					twist.angular.z=0.5
				detect=True
			else:
				backdist(10)
				stop()
	if flag==6 and detect==False:
		print('Third Loop')
		alignstatic(22.5)
		flag = flag+1
		if flag == n+1:
			t1=rospy.get_time()
			while ar_id!=4 and rospy.get_time()<t1+20:
				gostraight()
			stop()
			if ar_id==4:
				print "Tag seen"						
				alignstatic(ar_pose)
				while ar_distance>5:
					gostraight()
				stop()
				print "Tag reached"
				while(ar_id!=4):
					twist.angular.z=0.5
				detect=True
			else:
				backdist(10)
				stop()
	if detect==False:
		index=1
        d=2
        while d<=18 and detect==False:
			alignstatic(90)
			t1=rospy.get_time()
			while ar_id!=4 and rospy.get_time()<t1+10:
				gostraight()
			stop()
			if ar_id==4:
				print "Tag seen"						
				alignstatic(ar_pose)
				while ar_distance>5:
					gostraight()
				stop()
				print "Tag reached"
				while(ar_id!=4):
					twist.angular.z=0.5
				detect=True
			if index %2 ==0:
				d=d+4 
			index=index+1   
		
					

				



	print("Search complete")
	rospy.spin()


if __name__ == '__main__':
    listener()


