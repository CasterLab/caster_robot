#!/usr/bin/env python
import time
import rospy
import csv  
import sys
from math import pow, sqrt
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class Log():
    """docstring for Log"""
    def __init__(self):
        self._sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnosticsCallback)
        self._sub = rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self._sub = rospy.Subscriber("/imu_data", Imu, self.imuCallback)

        self.dict_log = dict()
        self.datas_log = []
        self.time_init = [0, 0, 0]
        self.time_now = [0, 0, 0]
        self.time_use = [0, 0, 0]
        self.time_log = 0
        self.file_name = ''
        self.time_now_name = ''
        self.args_num = len(sys.argv)
        self.ROCS_writer = False
        self.IMU_writer = False
        self.ODOM_writer = False
        self.distance = 0
        self.pose_x_last = 0
        self.pose_y_last = 0

    def diagnosticsCallback(self, msg):
        if self.ROCS_writer:
            self.time_log = self.time_now[0] - self.time_init[0]
            if msg.status[0].name == 'hongfu_bms_status_node: BMS':
                self.time_now[0] = msg.header.stamp.secs
                if self.time_init[0] == 0 or self.time_log >= 600:
                    ROCS = msg.status[0].values[0].value
                    dict_ROCS = {'ROCS':ROCS, 'time1':self.time_use[0]}
                    print(dict_ROCS)
                    # dict_log.update(xx)
                    # self.datas_log.append(self.dict_log)
                    self.logWriter('ROCS', 'ab+', 'time1', dict_ROCS)
                    self.time_init[0] = self.time_now[0]
                    self.time_use[0] += 10

    def imuCallback(self, msg):
        # rospy.loginfo('imu1')
        if self.IMU_writer:
            self.time_log = self.time_now[1] - self.time_init[1]
            self.time_now[1] = msg.header.stamp.secs
            if self.time_init[1] == 0 or self.time_log >= 1800:
                x = msg.orientation.x
                y = msg.orientation.y
                z = msg.orientation.z
                w = msg.orientation.w
                orientation = 'x:' + str(x) + ' y:' + str(y) + ' z:' + str(z) +' w:' + str(w)
                dict_orientation = {'orientation':orientation, 'time2':self.time_use[1]}
                # self.dict_log.update(dict_orientation)
                print(self.dict_log)
                self.logWriter('orientation', 'ab+', 'time2', dict_orientation)
                self.time_init[1] = self.time_now[1]
                self.time_use[1] += 30

    def odomCallback(self, msg):
        if self.ODOM_writer:
            self.pose_x_now = msg.pose.pose.position.x
            self.pose_y_now = msg.pose.pose.position.y
            if self.pose_x_last == 0:
                self.pose_x_last = msg.pose.pose.position.x
                self.pose_y_last = msg.pose.pose.position.y
            distance_last = sqrt(pow(self.pose_x_now - self.pose_x_last, 2) + pow(self.pose_y_now - self.pose_y_last, 2))
            self.distance = self.distance + distance_last
            self.time_log = self.time_now[2] - self.time_init[2]
            self.time_now[2] = msg.header.stamp.secs            
            if self.time_init[2] == 0 or self.time_log >= 120:
                dict_odom = {'odom':self.distance, 'time3':self.time_use[2]}
                # self.dict_log.update(dict_odom)
                print(self.dict_log)
                self.logWriter('odom', 'wb+', 'time3', dict_odom)
                self.time_init[2] = self.time_now[2]
                self.time_use[2] += 2

    def logWriter(self, status, file_option, time_option, dict_option):
        # rospy.loginfo('writer1')
        if self.file_name == '':
            # rospy.loginfo('writer2')
            self.time_now_name = time.strftime("%Y%m%d%H%M", time.localtime())
        self.file_name = 'caster-test-log-' + status + self.time_now_name + '.csv'
        with open('/home/caster/Documents/caster-tests/logs/'+self.file_name, file_option) as f:
            writer = csv.DictWriter(f, [status, time_option])
            rospy.loginfo('write' + status) 
            # if csv.Sniffer().has_header(f.read(1024)):
            if f.read(1024) == '':
                writer.writeheader()
            # for row in dict_log:
            writer.writerow(dict_option)
            # writer.writerows(self.dict_log)

    def getArgs(self):
        args_number = len(sys.argv)
        args_list = sys.argv
        if args_number != 1:
            for index in range(args_number):
                if args_list[index] ==  'ROCS':
                    self.ROCS_writer = True
                elif args_list[index] ==  'IMU':
                    self.IMU_writer = True
                elif args_list[index] ==  'ODOM':    
                    self.ODOM_writer = True
        else:
            rospy.logerr('please input args')
            rospy.signal_shutdown('please input args')

def main():
    rospy.init_node('caster_test_log')
    Logs = Log()
    Logs.getArgs()
    rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass

