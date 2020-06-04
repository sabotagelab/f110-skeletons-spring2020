#!/usr/bin/env python
import os
import time
import csv
from itertools import izip_longest
import rospy
from geometry_msgs.msg import Point, Quaternion,PoseWithCovarianceStamped
from nav_msgs.msg import  Odometry
from std_msgs.msg import Duration, Header, String,Bool,ColorRGBA
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from visualization_msgs.msg import Marker
from race_monitor.msg import RaceStats


class RaceMonitor:
    def __init__(self):
        rospy.init_node("race_monitor_node")
        odom_topic = rospy.get_param('odom_topic_name', '/odom')
        key_topic = rospy.get_param('key_topic_name', '/key')
        self.car_frame = rospy.get_param('car_frame', 'base_link')

        self.L = rospy.get_param('car_length', 0.325)
        self.lap_count = 0
        self.COUNT_FLAG = False

        self.current_pose = Point()
        self.current_angle = 0.0
        self.current_vel = 0
        self.max_vel = 0
        self.current_steering = 0

        self.NAVIGATION_ENABLE = False
        self.ODOM_INIT = False
        self.PROG_COMPLETE_FLAG = False
        self.CAR_START = False
        self.NAVIGATION_MODE = 0

        self.checkpoint = 0
        self.last_checkpoint = 0
        self.next_checkpoint = 1
        self.lap_times =[]
        self.checkpoints_times=[]
        self.start_time = 0
        self.num_laps =0
        self.num_collisions = 0
        self.RACE_TIME= rospy.get_param('race_time', 5)*60.0        #race time in minutes
        self.c1_line = [Point(12.06,-10.42,0),Point(14.75,-10.5, 0.0)]
        self.c2_line = [Point(0.82, -10.65, 0.0),Point(1.97, -13.09, 0)]
        self.c3_line = [Point(-19.69, -27.61, 0.0),Point(-17.94, -29.6, 0)]
        self.c4_line = [Point(-23.33, -22.10, 0), Point(-24.71, -20.17, 0.0)]
        self.c5_line = [Point(-16.73, -6.24, 0), Point(-16.91, -3.62, 0.0)]
        self.home_line = [Point(0.0, -1.25, 0.0), Point(0.0, 1.25, 0.0)]
        self.checkpoints_line_list =[self.home_line,self.c1_line,self.c2_line,self.c3_line,self.c4_line,self.c5_line,self.home_line]
        self.reset_checkpoints_list = self.find_mid_points(self.checkpoints_line_list)
        self.checkpoints_orientation_list=[0.0,-1.5725,-2.4501,-2.43,0.7263,0.0,0.0]

        # Publishers
        self.keyboard_pub = rospy.Publisher(key_topic, String, queue_size=10)
        self.restart_car_pub = rospy.Publisher("reset_car", Bool, queue_size=10)
        self.reset_pub = rospy.Publisher("reset_checkpoint", String, queue_size=1)
        self.pose_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped,queue_size=1)
        self.race_stats_pub = rospy.Publisher("race_stats",RaceStats,queue_size=1)
        self.time_marker_pub = rospy.Publisher("time_marker",Marker,queue_size=1)

        # Subscribers
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/collision_status',Bool,self.collision_callback,queue_size=1)

        self.STATS_FREQ = rospy.get_param('stats_frequency', 100)
        rospy.Timer(rospy.Duration(1.0 / self.STATS_FREQ), self.publish_stats)

    def find_mid_points(self,line_list):
        mid_points_list=[]
        for line in line_list:
            mid_point = Point((line[0].x+line[1].x)/2.0,(line[0].y+line[1].y)/2.0,0.0)
            mid_points_list.append(mid_point)
        return mid_points_list

    def odom_callback(self, msg):
        '''odometry callback from simulator'''
        self.current_pose = msg.pose.pose.position
        self.current_angle = self.quaternion_to_euler_yaw(msg.pose.pose.orientation)
        self.current_vel = msg.twist.twist.linear.x
        self.max_vel = max(self.max_vel,self.current_vel)
        if not self.ODOM_INIT:
            self.ODOM_INIT = True
            print(str(self.RACE_TIME/60.0)+" minutes of Race time started..")
            self.start_time = time.time()

    def collision_callback(self,msg):
        if msg.data:
            self.num_collisions +=1
            self.reset_to_last_checkpoint()
            print("Collision occured.")

    def create_header(self, frame_id):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return header

    def heading(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def quaternion_to_euler_yaw(self, orientation):
        _, _, yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return yaw

    def reset_to_last_checkpoint(self):
        pose_info = PoseWithCovarianceStamped()
        pose_info.pose.pose.position = self.reset_checkpoints_list[self.last_checkpoint]
        pose_info.pose.pose.orientation = self.heading(self.checkpoints_orientation_list[self.last_checkpoint])
        self.pose_pub.publish(pose_info)
        rospy.sleep(0.1)
        self.pose_pub.publish(pose_info)
        self.restart_car_pub.publish(Bool(True))

    def check_line_cross_with_crossproduct(self,point,line_p1,line_p2):
        line_vector = Point(line_p2.x-line_p1.x,line_p2.y-line_p1.y,0)
        point_vector = Point(point.x-line_p1.x,point.y-line_p1.y,0)
        cross_product= (point_vector.x*line_vector.y)-(point_vector.y*line_vector.x)
        return (cross_product>0)

    def check_car_start(self):
        return self.check_line_cross_with_crossproduct(self.current_pose,self.home_line[0],self.home_line[1])

    def publish_time_marker(self):
        time_marker = Marker()
        time_marker.header = self.create_header(self.car_frame)
        time_marker.type = time_marker.TEXT_VIEW_FACING
        curr_time = time.time()-self.start_time
        time_marker.text = str(int(curr_time/60))+':'+str(int(curr_time%60))
        time_marker.pose.position = Point(-2.5,0.0,0.0)
        time_marker.scale = Point(2.0,2.0,1.0)
        time_marker.color = ColorRGBA(0.0,0.1,1.0,1.0)
        self.time_marker_pub.publish(time_marker)

    def publish_stats(self,event):
        if self.CAR_START:
            stats_msg = RaceStats()
            stats_msg.header = self.create_header(self.car_frame)
            if self.lap_times ==[]:
                stats_msg.lap_times = [0]
            else:
                stats_msg.lap_times = self.lap_times
            stats_msg.fastest_lap_time = min(stats_msg.lap_times)
            stats_msg.top_speed = self.max_vel
            stats_msg.number_collisions = self.num_collisions
            stats_msg.number_laps = self.num_laps
            self.race_stats_pub.publish(stats_msg)
            self.publish_time_marker()
            if(time.time()-self.start_time>=self.RACE_TIME):
                print("Race time completed. Writing stats to file.")
                self.last_checkpoint =0
                self.next_checkpoint = 1
                self.PROG_COMPLETE_FLAG=True
                self.CAR_START = False
                self.reset_to_last_checkpoint()

                timestr = time.strftime("%Y-%m-%d_%I-%M-%S_%p")
                csv_columns = ['Fastest lap time', 'Lap times','Top Speed', 'Number_Collisions', 'Number_laps']
                dirname = os.path.dirname(__file__)
                csv_location = '../race_log/'+timestr+'.csv'
                csv_filepath = os.path.join(dirname, csv_location)
                try:
                    with open(csv_filepath, 'w') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(csv_columns)
                        for row in izip_longest([stats_msg.fastest_lap_time],stats_msg.lap_times,[stats_msg.top_speed],[stats_msg.number_collisions],[stats_msg.number_laps]):
                            writer.writerow(row)
                except IOError:
                    print("I/O error")
                rospy.signal_shutdown('Race time completed. Program terminated.')

    def start_monitoring(self):
        print("Monitoring started...")
        while not rospy.is_shutdown():
            if self.ODOM_INIT and not self.PROG_COMPLETE_FLAG:
                if self.check_car_start() and not self.CAR_START:
                    self.lap_start_time = time.time()
                    print("Recording time..")
                    self.last_checkpoint = 0
                    self.next_checkpoint = 1
                    self.CAR_START = True
                if self.check_line_cross_with_crossproduct(self.current_pose,self.checkpoints_line_list[self.next_checkpoint][0],self.checkpoints_line_list[self.next_checkpoint][1]):
                    checkpoint_time = time.time()-self.lap_start_time
                    print("checkpoint time for c",self.next_checkpoint,"=",checkpoint_time)
                    self.checkpoints_times.append(checkpoint_time)
                    if self.next_checkpoint ==len(self.checkpoints_line_list)-1:
                        final_lap_time = time.time()-self.lap_start_time
                        self.num_laps +=1
                        self.lap_start_time = time.time()
                        print("Final lap time=",final_lap_time)
                        self.lap_times.append(final_lap_time)
                        self.next_checkpoint = 1
                        self.last_checkpoint = 0
                    else:
                        self.last_checkpoint +=1
                        self.next_checkpoint +=1
                rospy.sleep(0.005)

if __name__ == '__main__':
    race_node = RaceMonitor()
    race_node.start_monitoring()
    rospy.spin()


