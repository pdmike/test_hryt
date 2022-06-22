#!/usr/bin/python
#coding=UTF-8
from __future__ import division, with_statement
try:
    from cStringIO import StringIO as BufferType
except ImportError:
    from io import BytesIO as BufferType
from rostopic import ROSTopicHz
import rostopic
import os
import rospy
import roslib
import PyKDL
import math
import threading
import csv
from datetime import datetime
import time
import sys
from copy import deepcopy
import math
import Queue
import rosnode
import subprocess
import re

from std_msgs.msg import String
from std_msgs.msg import Bool
from msgs.msg import VehiclePose
from msgs.msg import VehicleState
from msgs.msg import VehicleInfoSet_ForHMI
from msgs.msg import Diagnose
from msgs.msg import TaskRequest
from msgs.msg import SenseingPose
from msgs.msg import DriveJobInfo
from msgs.msg import DriveCommandLongtiControl
from msgs.msg import PowerStatus
from msgs.msg import AVPVCU
from msgs.msg import TargetSlot
from msgs.msg import RouteInfo_ForHMI
from msgs.msg import RouteInfoSet_ForHMI
from msgs.msg import VehicleInfo_ForHMI
from mvp_msgs.msg import RSUHeart

from mvp_msgs.msg import VehInfo
from mvp_msgs.msg import FusionStates

from perception_msg.msg import PerceptionMsg

from sensor_msgs.msg import PointCloud

from srvs.srv import RouteOperation, RouteOperationRequest, RouteOperationResponse
from srvs.srv import VehicleOperation, VehicleOperationRequest, VehicleOperationResponse

import numpy as np
import logging




# pwd = os.path.dirname(__file__)
# # print(pwd + "/hvp_lidar_topic.txt")
# path_diagnose_file = "hvp_vehicle_diagnose.csv"
# path_log_folder = "/home/hvp/work/HMI/diagnose_log"

# path_lidar_topic_list = "hvp_lidar_topic.txt"
# path_trajectory_file_path1 = "path1.txt"
# path_trajectory_file_path2 = "path2.csv"


pwd = os.path.dirname(__file__)
path_diagnose_file = pwd + "/hvp_vehicle_diagnose.csv"
path_log_folder = "/home/shengshi/catkin_ws/src/diagnose_log"

path_lidar_topic_list = pwd + "/hvp_lidar_topic.txt"
path_trajectory_file_path1 = pwd + "/path1.txt"
path_trajectory_file_path2 = pwd + "/path2.csv"

lidar_offline_rate_standard = 7.0
trajectory_ignore_range_route1 = [-100, 115]
trajectory_ignore_range_route2 = [60, 110]
max_no_pose_error_dis = 3.5
max_no_perception_detected_counter = 10
max_trajectory_error = 3
dp_pub_level = "000003"
pad_timeout = 10
side_by_side_distance_limit = 13
vehicle_offline_check_time = 7
vehicle_offline_timeout_time = 900
network_check_ip_list = ["192.168.1.40", "192.168.1.50"]
region1_ip_list = {
        0:["192.168.1.200", "192.168.1.201", "192.168.1.202", "192.168.1.203"],
        1:["192.168.1.204", "192.168.1.203", "192.168.1.202", "192.168.1.207"],
        2:["192.168.1.207", "192.168.1.208", "192.168.1.209"],
        3:["192.168.1.211", "192.168.1.208", "192.168.1.209", "192.168.1.210"],
        4:["192.168.1.211", "192.168.1.212", "192.168.1.213", "192.168.1.214", "192.168.1.215"],
        5:["192.168.1.213", "192.168.1.215", "192.168.1.216", "192.168.1.217"],
        6:["192.168.1.219", "192.168.1.216", "192.168.1.217", "192.168.1.218"],
        7:["192.168.1.219", "192.168.1.218", "192.168.1.220", "192.168.1.221"],
        8:["192.168.1.222", "192.168.1.220", "192.168.1.221", "192.168.1.223"],
        9:["192.168.1.222", "192.168.1.223", "192.168.1.224", "192.168.1.225"],
        10:["192.168.1.226", "192.168.1.225", "192.168.1.224", "192.168.1.227", "192.168.1.228"],
        11:["192.168.1.229", "192.168.1.226", "192.168.1.228", "192.168.1.230"],
        12:["192.168.1.229", "192.168.1.230", "192.168.1.231", "192.168.1.232"],
        13:["192.168.1.234", "192.168.1.231", "192.168.1.232", "192.168.1.233"],
        14:["192.168.1.234", "192.168.1.233", "192.168.1.235", "192.168.1.236"]  
        }
region2_ip_list = {
        0:["192.168.1.250", "192.168.1.251", "192.168.1.252", "192.168.1.240", "192.168.1.241"],
        1:["192.168.1.250", "192.168.1.241", "192.168.1.246", "192.168.1.247", "192.168.1.189"],
        2:["192.168.1.240", "192.168.1.249", "192.168.1.244", "192.168.1.241", "192.168.1.195"],
        3:["192.168.1.194", "192.168.1.212", "192.168.1.214", "192.168.1.190", "192.168.1.195", "192.168.1.196", "192.168.1.244"],         
        5:["192.168.1.199", "192.168.1.192", "192.168.1.193", "192.168.1.197", "192.168.1.198"],              
        6:["192.168.1.239", "192.168.1.237", "192.168.1.197", "192.168.1.198", "192.168.1.199", "192.168.1.191"],
        7:["192.168.1.239", "192.168.1.237", "192.168.1.243", "192.168.1.245", "192.168.1.248", "192.168.1.191", "192.168.1.186"],
        }

########DP parameter#####
max_pedestrian_check_count = 4
max_pedestrian_lose_frame_count = 1
max_pedestrian_area_lose_frame_count = 2
max_pedestrian_detect_frame_count = 0
max_diff_timeout_frame_count = 150
max_diff_queue_size = 4
minus_1_area = [5, 6, 7, 8, 9, 10, 11, 12, 13]
minus_2_area = [0, 1, 2, 3, 4, 14]
default_max_tracking_distance = 0.2
max_size_for_sizequeue = 5
diff_check_route2_x_limit = 35
#######################

area_vehicle_avoid_route1 = [[0, 1.33], [6.6, 1.33], [6.6, -1.33], [0, -1.33]]
# area_vehicle_avoid_route1 = [[0, 2.33], [6.6, 2.33], [6.6, -2.33], [0, -2.33]]
# area_vehicle_avoid_route1 = [[2.6, 3.13], [6.6, 3.13], [6.6, -3.13], [2.6, -3.13]]
area_vehicle_avoid_route2 = [[0, 1.33], [6.6, 1.33], [6.6, -1.33], [0, -1.33]]
area_vehicle_avoid_R_route2 = [[-2, 1.03],[0, 1.03],[0, -1.03], [-2, -1.03]]
area_vehicle = [[-2.6, 1.13], [2.6, 1.13], [2.6, -1.13], [-2.6, -1.13]]

area_pedestrian_checker_origin_route1 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]
area_pedestrian_checker_origin_1_route1 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]
area_pedestrian_checker_origin_2_route1 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]
area_pedestrian_checker_origin_5_6_7_8_route1 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]
area_pedestrian_checker_origin_11_12_13_14_route1 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]

diff_area_route1 = [[-4.6, 3.13], [6.6, 3.13], [6.6, -3.13], [-4.6, -3.13]]
diff_area_5_6_7_8_route1 = [[-4.6, 3.13], [6.6, 3.13], [6.6, -2.33], [-4.6, -2.33]]
diff_area_1_route1 = [[-4.6, 2.33], [6.6, 2.33], [6.6, -2.83], [-4.6, -2.83]]
diff_area_2_route1 = [[-4.6, 2.13], [6.6, 2.13], [6.6, -2.13], [-4.6, -2.13]]
diff_area_11_12_13_14_route1 = [[-4.6, 2.83], [6.6, 2.83], [6.6, -2.53], [-4.6, -2.53]]

area_pedestrian_checker_origin_route2 = [[-3.4, 2.33], [6.6, 2.33], [6.6, -2.33], [-3.4, -2.33]]
# area_pedestrian_checker_origin_route2 = [[-2.6, 2.33], [6.6, 2.33], [6.6, -2.33], [-2.6, -2.33]]
area_pedestrian_checker_origin_R_route2 = [[-5.1, 2.33],[2.6, 2.33],[2.6, -2.33], [-5.1, -2.33]]
diff_area_route2 = [[-4.6, 3.13], [6.6, 3.13], [6.6, -3.13], [-4.6, -3.13]]
diff_area_R_route2 = [[-2.6, 3.13], [4.6, 3.13], [4.6, -3.13], [-2.6, -3.13]]

lidar_pole_list = [[36.6, 13.4],[41.9, 13.4],[0.09, 91.6], [24.1, 91.5],[24.3, 83.2],[-23.8, 86.1],[-47.6, 85.3],[-71.3, 85.1],[-95.3, 85.2],[-111, 86.3],[-111, 118],[-119, 97.4],[-119,117],[-119,137],[-119,157]]
ignore_lidar_list = [[18.7, 2.07],[18.7, -3.4],[12.7, 2.07],[12.7, -3.4],[36.6, 13.4],[41.9, 13.4],[0.09, 91.6], [24.1, 91.2],[24.3, 83.2],[-23.8, 86.1],[-47.6, 85.3],[-71.3, 85.1],[-95.3, 85.2],[-111, 86.3],[-111, 118],[-119, 97.4],[-119,117],[-119,137],[-119,157]]
pedestrian_ignore_list = []
lidar_ignore_radius = 2
ignore_detect_area_list = [[[35.6, 44],[35.6, 12.6],[42.3, 12.6],[42.3, 44]], 
                            [[10.8, 3],[10.8, -4.32],[20.0, -4.32],[20.0, 3]]]

# region1_list = [ [[0, -5],  [18, -5], [18, 5], [0, 5]],
#                 [[18, -5], [45, -5], [45, 14], [18, 14]],
#                 [[33, 14], [44, 14], [44, 37], [33, 37]],
#                 [[30, 37], [43, 37], [43, 60], [30, 60]],
#                 [[24, 60], [43, 60], [43, 95], [24, 95]],
#                 [[0, 82], [24, 82], [24, 95], [0, 95]],
#                 [[-24, 77], [0, 77], [0, 94], [-24, 94]],
#                 [[-47, 77], [-24, 77], [-24, 94], [-47, 94]],
#                 [[-71, 77], [-47, 77], [-47, 94], [-71, 94]],
#                 [[-95, 77], [-71, 77], [-71, 94], [-95, 94]],
#                 [[-124, 77], [-95, 77], [-95, 97], [-124, 97]],
#                 [[-120, 97], [-108, 97], [-108, 118], [-120, 118]],
#                 [[-120, 118], [-110, 118], [-110, 137], [-120, 137]],
#                 [[-120, 137], [-110, 137], [-110, 156], [-120, 156]],
#                 [[-120, 156], [-91, 156], [-91, 178], [-120, 178]] 
#                 ] 
# region2_list = [ [[12, 147], [39, 147], [39, 131], [12, 131]],
#                  [[39, 150], [53, 150], [53, 130], [39, 130]],
#                  [[39, 130], [53, 130], [53, 106], [39, 106]],
#                  [[39, 106], [53, 106], [53, 76], [39, 76]],
#                  [[47, 84], [78, 84], [78, 70], [47, 70]],
#                  [[64, 106], [78, 106], [78, 76], [64, 76]],
#                  [[64, 130], [78, 130], [78, 106], [64, 106]],
#                  [[64, 145], [78, 145], [78, 130], [64, 130]],
#                  [[62, 176], [78, 176], [78, 145], [62, 145]],
#                  [[62, 176], [78, 176], [78, 145], [62, 145]],
#                  [[46, 176], [62, 176], [62, 165], [46, 165]],
#                  [[39, 165], [54, 165], [54, 150], [39, 150]]
#                 ]

region1_list = {
        0:[[0, -5],  [18, -5], [18, 5], [0, 5]],
        1:[[18, -5], [45, -5], [45, 14], [18, 14]],
        2:[[33, 14], [44, 14], [44, 37], [33, 37]],
        3:[[30, 37], [43, 37], [43, 60], [30, 60]],
        4:[[24, 60], [43, 60], [43, 94], [24, 94]],
        5:[[0, 81], [24, 81], [24, 94], [0, 94]],
        6:[[-24, 77], [0, 77], [0, 94], [-24, 94]],
        7:[[-47, 77], [-24, 77], [-24, 94], [-47, 94]],
        8:[[-71, 77], [-47, 77], [-47, 94], [-71, 94]],
        9:[[-95, 77], [-71, 77], [-71, 94], [-95, 94]],
        10:[[-124, 77], [-95, 77], [-95, 97], [-124, 97]],
        11:[[-120, 97], [-110, 97], [-110, 118], [-120, 118]],
        12:[[-120, 118], [-110, 118], [-110, 137], [-120, 137]],
        13:[[-120, 137], [-110, 137], [-110, 156], [-120, 156]],
        14:[[-120, 156], [-91, 156], [-91, 178], [-120, 178]]  
        }


# region2_list = {
#         0:[[12, 147], [39, 147], [39, 131], [12, 131]],
#         1:[[39, 150], [59.5, 150], [59.5, 128.8], [39, 128.8]],
#         2:[[39, 128.8], [59.5, 128.8], [59.5, 106], [39, 106]],
#         3:[[39, 106], [59.5, 106], [59.5, 84], [54, 84], [54, 76], [39, 70]],
#         4:[[47, 76], [54, 76], [54, 84], [66, 84],[66, 76], [72, 76], [72, 70], [47, 70]],         
#         5:[[59.5, 106], [78, 106], [78, 76], [66, 76], [66, 84], [59.5, 84]],                 
#         6:[[59.5, 130], [78, 130], [78, 106], [59.5, 106]],
#         7:[[59.5, 176], [78, 176], [78, 130], [59.5, 130]],
#         11:[[39, 176], [59.5, 176], [59.5, 150]region2_list, [39, 150]],
#         }    

region2_list = {
        0:[[12, 147], [39, 147], [39, 131], [12, 131]],
        1:[[39, 176], [59.5, 176], [59.5, 140], [39, 140]],
        2:[[39, 140], [59.5, 140], [59.5, 104], [39, 104]],
        3:[[39, 104], [59.5, 104], [59.5, 70], [39, 70]],         
        5:[[59.5,106],[78,106],[78,70],[59.5,70]],              
        6:[[59.5, 130], [78, 130], [78, 106], [59.5, 106]],
        7:[[59.5, 176], [78, 176], [78, 130], [59.5, 130]],
        }   



region1_ignore_area = [
    [[-108, 168.5], [-101, 168.5], [-101, 164.5], [-108, 164.5]],
    [[-108, 174.5], [-101, 174.5], [-101, 169.5], [-108, 169.5]]
]                

region2_ignore_area = [
                [[40, 161], [47.5, 161], [47.5, 150], [40, 150]],
                [[40, 134], [49, 134], [49, 76], [40, 76]],
                [[70, 163], [78, 163], [78, 78], [70, 78]],
                [[51, 167], [67, 167], [67, 83], [51, 83]],
                [[48, 86], [71, 86], [71, 71], [48, 71]]
                ]


region1_parking_space = {101:[[-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203]],
                        102:[[-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203]],
                        103:[[-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203], [-115.422744751, 146.462158203]],
                        104:[[-110, 169], [-95, 169], [-95, 163], [-110, 163]],
                        105:[[-110, 175], [-95, 175], [-95, 168], [-110, 168]]
                        # 105:[[-110, 169], [-90, 169], [-90, 163], [-110, 163]],
                        # 104:[[-110, 175], [-95, 175], [-95, 168], [-110, 168]]
                        }


region2_parking_space = {201:[[41,162.254],[48.5,162.263],[48.5,158.611],[41,158.602]], 
                         202:[[41,159.202], [48.5,159.211], [48.5,155.549], [41,155.558]],
                         203:[[41,156.158], [48.5,156.149], [48.5,152.485], [41,152.478]],
                         204:[[41,153.078], [48.5,153.085], [48.5,149.439], [41,149.427]],
                         205:[[41,135.24], [48.5,135.24], [48.5,131.606], [41,131.608]],
                         206:[[41,132.208], [48.5,132.206], [48.5,128.529], [41,128.527]],
                         207:[[41,129.127], [48.5,129.129], [48.5,125.503], [41,125.503]], 
                         208:[[41,126.103], [48.5,126.103], [48.5,122.443], [41,122.443]], 
                         209:[[41,123.043], [48.5,123.043], [48.5,119.281], [41,119.281]],
                         210:[[41,119.981], [48.5,119.981], [48.5,116.332], [41,116.332]],
                         211:[[41,116.932], [48.5,116.932], [48.5,113.271], [41,113.271]],
                         212:[[41,113.871], [48.5,113.871], [48.5,110.22], [41,110.22]],
                         213:[[41,110.82], [48.5,110.82], [48.5,107.125], [41,107.125]],
                         214:[[41,107.725], [48.5,107.725], [48.5,104.04], [41,104.04]],
                         215:[[41,104.64], [48.5,104.64], [48.5,101.005], [41,101.005]],
                         216:[[41,101.605], [48.5,101.605], [48.5,98], [41,98]],
                         217:[[41,98.559], [48.5,98.559], [48.5,94.9], [41,94.9]],
                         218:[[41,95.502], [48.5,95.502], [48.5,91.8], [41,91.8]],
                         219:[[41,92.447], [48.5,92.447], [48.5,88.7], [41,88.7]],
                         220:[[41,89.375], [48.5,89.375], [48.5,85.7], [41,85.7]],
                         221:[[41,86.331], [48.5,86.331], [48.5,82.648], [41,82.648]],
                         222:[[41,83.248], [48.5,83.248], [48.5,79.584], [41,79.584]],
                         223:[[41,80.184], [48.5,80.184], [48.5,76.35], [41,76.35]],
                         224:[[71,82.005], [78,82.005], [78,78.439], [71,78.439]],
                         225:[[71,85.046], [78,85.046], [78,81.405], [71,81.405]],
                         226:[[71,88.072], [78,88.072], [78,84.446], [71,84.446]],
                         227:[[71,91.114], [78,91.114], [78,87.472], [71,87.472]],
                         228:[[71,94.105], [78,94.105], [78,90.514], [71,90.514]], 
                         229:[[71,97.148], [78,97.148], [78,93.505], [71,93.505]],
                         230:[[71,100.178], [78,100.178], [78,96.548], [71,96.548]],
                         231:[[71,103.141], [78,103.141], [78,99.578], [71,99.578]],
                         232:[[71,106.166], [78,106.166], [78,102.541], [71,102.541]],
                         233:[[71,109.181], [78,109.181], [78,105.566], [71,105.566]],
                         234:[[71,112.189], [78,112.189], [78,108.581], [71,108.581]],
                         235:[[71,115.208], [78,115.208], [78,111.589], [71,111.589]],
                         236:[[71,118.211], [78,118.211], [78,114.608], [71,114.608]],
                         237:[[71,121.232], [78,121.232], [78,117.611], [71,117.611]],
                         238:[[71,124.248], [78,124.248], [78,120.632], [71,120.632]],
                         239:[[71,127.253], [78,127.253], [78,123.648], [71,123.648]],
                         240:[[71,130.3], [78,130.3], [78,126.653], [71,126.653]],
                         241:[[71,133.308], [78,133.308], [78,129.7], [71,129.7]],
                         242:[[71,136.378], [78,136.378], [78,132.708], [71,132.708]],
                         243:[[71,139.335], [78,139.335], [78,135.778], [71,135.778]],
                         244:[[71,142.358], [78,142.358], [78,138.735], [71,138.735]],
                         245:[[71,145.371], [78,145.371], [78,141.758], [71,141.758]],
                         246:[[71,148.387], [78,148.387], [78,144.771], [71,144.771]],
                         247:[[71,151.406], [78,151.406], [78,147.787], [71,147.787]],
                         248:[[71,154.432], [78,154.432], [78,150.806], [71,150.806]],
                         249:[[71,157.46], [78,157.46], [78,153.832], [71,153.832]],
                         250:[[71,160.49], [78,160.49], [78,156.86], [71,156.86]],
                         251:[[71,163.549], [78,163.549], [78,159.89], [71,159.89]]
                         }

# region2_parking_space = {201:[[41,161.954],[48,161.963],[47,158.911],[41,158.902]], 
#                          202:[[41,158.902], [48,158.911], [47,155.849], [41,155.858]],
#                          203:[[41,155.858], [48,155.849], [47,152.785], [41,152.778]],
#                          204:[[41,152.778], [48,152.785], [48,149.739], [41,149.727]],
#                          205:[[41,134.94], [48,134.94], [48,131.906], [41,131.908]],
#                          206:[[41,131.908], [48,131.906], [48,128.829], [41,128.827]],
#                          207:[[41,128.827], [48,128.829], [48,125.803], [41,125.803]], 
#                          208:[[41,125.803], [48,125.803], [48,122.743], [41,122.743]], 
#                          209:[[41,122.743], [48,122.743], [48,119.681], [41,119.681]],
#                          210:[[41,119.681], [48,119.681], [48,116.632], [41,116.632]],
#                          211:[[41,116.632], [48,116.632], [48,113.571], [41,113.571]],
#                          212:[[41,113.571], [48,113.571], [48,110.52], [41,110.52]],
#                          213:[[41,110.52], [48,110.52], [48,107.425], [41,107.425]],
#                          214:[[41,107.425], [48,107.425], [48,104.34], [41,104.34]],
#                          215:[[41,104.34], [48,104.34], [48,101.305], [41,101.305]],
#                          216:[[41,101.305], [48,101.305], [48,98.259], [41,98.259]],
#                          217:[[41,98.259], [48,98.259], [48,95.202], [41,95.202]],
#                          218:[[41,95.202], [48,95.202], [48,92.147], [41,92.147]],
#                          219:[[41,92.147], [48,92.147], [48,89.075], [41,89.075]],
#                          220:[[41,89.075], [48,89.075], [48,86.031], [41,86.031]],
#                          221:[[41,86.031], [48,86.031], [48,82.948], [41,82.948]],
#                          222:[[41,82.948], [48,82.948], [48,79.884], [41,79.884]],
#                          223:[[41,79.884], [48,79.884], [48,76.85], [41,76.85]],
#                          224:[[71,81.705], [78,81.705], [78,78.739], [71,78.739]],
#                          225:[[71,84.746], [78,84.746], [78,81.705], [71,81.705]],
#                          226:[[71,87.772], [78,87.772], [78,84.746], [71,84.746]],
#                          227:[[71,90.814], [78,90.814], [78,87.772], [71,87.772]],
#                          228:[[71,93.805], [78,93.805], [78,90.814], [71,90.814]], 
#                          229:[[71,96.848], [78,96.848], [78,93.805], [71,93.805]],
#                          230:[[71,99.878], [78,99.878], [78,96.848], [71,96.848]],
#                          231:[[71,102.841], [78,102.841], [78,99.878], [71,99.878]],
#                          232:[[71,105.866], [78,105.866], [78,102.841], [71,102.841]],
#                          233:[[71,108.881], [78,108.881], [78,105.866], [71,105.866]],
#                          234:[[71,111.889], [78,111.889], [78,108.881], [71,108.881]],
#                          235:[[71,114.908], [78,114.908], [78,111.889], [71,111.889]],
#                          236:[[71,117.911], [78,117.911], [78,114.908], [71,114.908]],
#                          237:[[71,120.932], [78,120.932], [78,117.911], [71,117.911]],
#                          238:[[71,123.948], [78,123.948], [78,120.932], [71,120.932]],
#                          239:[[71,126.953], [78,126.953], [78,123.948], [71,123.948]],
#                          240:[[71,130], [78,130], [78,126.953], [71,126.953]],
#                          241:[[71,133.008], [78,133.008], [78,130], [71,130]],
#                          242:[[71,136.078], [78,136.078], [78,133.008], [71,133.008]],
#                          243:[[71,139.035], [78,139.035], [78,136.078], [71,136.078]],
#                          244:[[71,142.058], [78,142.058], [78,139.035], [71,139.035]],
#                          245:[[71,145.071], [78,145.071], [78,142.058], [71,142.058]],
#                          246:[[71,148.087], [78,148.087], [78,145.071], [71,145.071]],
#                          247:[[71,151.106], [78,151.106], [78,148.087], [71,148.087]],
#                          248:[[71,154.132], [78,154.132], [78,151.106], [71,151.106]],
#                          249:[[71,157.16], [78,157.16], [78,154.132], [71,154.132]],
#                          250:[[71,160.19], [78,160.19], [78,157.16], [71,157.16]],
#                          251:[[71,163.249], [78,163.249], [78,160.19], [71,160.19]]
#                          }


def print_msg(level, str_msg):
    msg = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + " " + str(str_msg)
    if level == 0:
        print ("\033[32m" + msg + "\033[0m")
    elif level == 1:
        print ("\033[31m" + msg + "\033[0m")
    else:
        print(msg)

class Logger(object):
    _instance_lock = threading.Lock()
    default_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    def __init__(self):
        self.common_folder = path_log_folder + "/common"
        self.create_common_folder()
        self.common_diagnose_log_path = self.common_folder + "/hvp_diagnose_log_common_" + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + ".txt"
        self.log_folder_name = path_log_folder
        self.log_diagnose_path = ""
        self.path_vehicle_log_folder_dict = {}
        self.path_vehicle_diagnose_path_dict = {}
        self.path_vehicle_pd_path_dict = {}
        # self.path_log_file = self.log_folder_name + "/hvp_diagnose_log_" + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + ".txt"
        
    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, '_instance'):
            with Logger._instance_lock:
                if not hasattr(cls, '_instance'):
                    Logger._instance = object.__new__(cls, *args, **kwargs)
        return Logger._instance
    
    def create_common_folder(self):
        is_exists = os.path.exists(path_log_folder)
        if is_exists == False:
            os.mkdir(path_log_folder)
        is_exists_common = os.path.exists(self.common_folder)
        if is_exists_common == False:
            os.mkdir(self.common_folder)

    def create_vehicle_folder(self, VIN):
        path_vehicle_log_folder = self.log_folder_name + "/" + VIN
        path_vehicle_log_diagnose_folder = path_vehicle_log_folder + "/diagnose"
        path_vehicle_log_PD_folder = path_vehicle_log_folder + "/pedestrian_detector"
        is_exist_log_folder = os.path.exists(path_vehicle_log_folder)
        is_exist_diagnose_folder = os.path.exists(path_vehicle_log_diagnose_folder)
        is_exist_PD_folder = os.path.exists(path_vehicle_log_PD_folder)
        
        if is_exist_log_folder == False:
            os.mkdir(path_vehicle_log_folder)
            os.mkdir(path_vehicle_log_diagnose_folder)
            os.mkdir(path_vehicle_log_PD_folder)
        else:
            if is_exist_diagnose_folder == False:
                os.mkdir(path_vehicle_log_diagnose_folder)
            if is_exist_PD_folder == False:
                os.mkdir(path_vehicle_log_PD_folder)    
        self.path_vehicle_diagnose_path_dict[VIN] = path_vehicle_log_diagnose_folder + "/hvp_diagnose_log_" + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + ".txt"              
        self.path_vehicle_pd_path_dict[VIN] = path_vehicle_log_PD_folder +  "/pd_log_" + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + ".txt"
        self.path_vehicle_log_folder_dict[VIN] = path_vehicle_log_folder

    def write_diagnose_log(self, VIN, errorCode, CancelReason, errorDescription, diagnoseType):       
        log_str = "%-15s%-25s%-15s%-15s%-55s%-15s\n" % (str(time.time()), time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), errorCode, CancelReason, errorDescription, diagnoseType)
        if VIN == "COMMON":
            with open(self.common_diagnose_log_path, "a") as log_file:
                log_file.write(log_str)
        else:
            if self.path_vehicle_diagnose_path_dict.__contains__(VIN):
                with open(self.path_vehicle_diagnose_path_dict[VIN], "a") as log_file:
                    log_file.write(log_str)
            else:
                with open(self.common_diagnose_log_path, "a") as log_file:
                    log_file.write(log_str)

    def write_dp_log(self, VIN, vehicle_status, pedestrian_id, pedestrian_status):
        log_str = "%-15s%-25s%-76s%-76s%-50s\n" % (str(time.time()), time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), vehicle_status, pedestrian_id, pedestrian_status)
        if self.path_vehicle_pd_path_dict.__contains__(VIN):
            with open(self.path_vehicle_pd_path_dict[VIN], "a") as log_file:
                log_file.write(log_str)

class Vehicle_Diagnose():
    def __init__(self,number, discription, diag_type):
        self.number = int(number)
        self.discription = discription
        self.type = diag_type
        
class Topic_Info(ROSTopicHz):
    def __init__(self, topic_name, topic_type):
        super(Topic_Info, self).__init__(100)
        self._topic_name = topic_name
        self._subscriber = None
        self.monitoring = False
        self._reset_data()
        self.message_class = None
        self.perception_msg_vehicle = []
        self.perception_msg_pedestrian = []
        self.area = 0
        try:
            self.message_class = roslib.message.get_message_class(topic_type)
            #self.perception_msg = object.__new__(self.message_class)
        except Exception as e:
            self.message_class = None
            #self.perception_msg = None

        if self.message_class is None:
            self.error = 'can not get message class for type "%s"' % topic_type

    def _reset_data(self):
        self.last_message = None
        self.times = []
        self.timestamps = []
        self.sizes = []
    
    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        if self.message_class is not None:
            self.monitoring = True
            # FIXME: subscribing to class AnyMsg breaks other subscribers on same node
            self._subscriber = rospy.Subscriber(
                self._topic_name, self.message_class, self.message_callback, queue_size=1, tcp_nodelay= True)

    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self._subscriber is not None:
            self._subscriber.unregister()

    def message_callback(self, message):
        ROSTopicHz.callback_hz(self, message)
        self.area = message.device_id.data
        with self.lock:
            self.timestamps.append(rospy.get_time())
            if hasattr(message, 'lidarframe'):
                vehicle_list = []
                pedestrian_list = []
                ped_id = 0
                for obstacle in message.lidarframe.objects.objects:
                    if obstacle.coreInfo.type.data == 3:
                        pose_x_y_vehicle = []
                        heading = math.atan2(obstacle.coreInfo.direction.y.data, (obstacle.coreInfo.direction.x.data))
                        vehicle_x_v = obstacle.coreInfo.velocity.x.data
                        vehicle_y_v = obstacle.coreInfo.velocity.y.data                       
                        pose_x_y_vehicle.append(obstacle.coreInfo.center.x.data)
                        pose_x_y_vehicle.append(obstacle.coreInfo.center.y.data)                        
                        pose_x_y_vehicle.append(message.timestamp.data)
                        pose_x_y_vehicle.append(heading)
                        pose_x_y_vehicle.append(vehicle_x_v)
                        pose_x_y_vehicle.append(vehicle_y_v)
                        vehicle_list.append(pose_x_y_vehicle)
                    elif obstacle.coreInfo.type.data == 1 or obstacle.coreInfo.type.data == 2:
                        ignoreFlag = False
                        pose_x_y_pedestrian = []
                        obstacle_x = obstacle.coreInfo.center.x.data
                        obstacle_y = obstacle.coreInfo.center.y.data
                        obstacle_x_v = obstacle.coreInfo.velocity.x.data
                        obstacle_y_v = obstacle.coreInfo.velocity.y.data
                        for pedestrian_ignore in pedestrian_ignore_list:
                            if (obstacle_x > pedestrian_ignore[0]-0.5) and (obstacle_x < pedestrian_ignore[0] + 0.5) and \
                               (obstacle_y > pedestrian_ignore[1]-0.5) and (obstacle_y < pedestrian_ignore[1] + 0.5): 
                               ignoreFlag = True
                               break
                        if ignoreFlag == False:
                            detect_type = obstacle.coreInfo.type.data
                            detect_type_conf = obstacle.coreInfo.type_confidence.data
                            pose_x_y_pedestrian.append(detect_type)                       
                            pose_x_y_pedestrian.append(obstacle_x)
                            pose_x_y_pedestrian.append(obstacle_y)
                            pose_x_y_pedestrian.append(self.area)
                            pose_x_y_pedestrian.append(obstacle_x_v)
                            pose_x_y_pedestrian.append(obstacle_y_v)
                            pose_x_y_pedestrian.append(detect_type_conf)
                            # if abs(obstacle_y - 24.4684467316 ) < 0.01:
                            #     print message.timestamp.data
                            pedestrian_list.append(pose_x_y_pedestrian)

                self.perception_msg_vehicle = deepcopy(vehicle_list)   
                self.perception_msg_pedestrian = deepcopy(pedestrian_list)                 
                        
            #print self.perception_msg
            # FIXME: this only works for message of class AnyMsg
            # self.sizes.append(len(message._buff))
            # time consuming workaround...
            buff = BufferType()
            message.serialize(buff)
            self.sizes.append(len(buff.getvalue()))

            if len(self.timestamps) > self.window_size - 1:
                self.timestamps.pop(0)
                self.sizes.pop(0)
            assert(len(self.timestamps) == len(self.sizes))

            self.last_message = message

    def get_bw(self):
        if len(self.timestamps) < 2:
            return None, None, None, None
        current_time = rospy.get_time()
        if current_time <= self.timestamps[0]:
            return None, None, None, None

        with self.lock:
            total = sum(self.sizes)
            bytes_per_s = total / (current_time - self.timestamps[0])
            mean_size = total / len(self.timestamps)
            max_size = max(self.sizes)
            min_size = min(self.sizes)
            return bytes_per_s, mean_size, min_size, max_size

    # def get_hz(self):
    #     if not self.times:
    #         return None, None, None, None
    #     with self.lock:
    #         n = len(self.times)
    #         mean = sum(self.times) / n
    #         rate = 1. / mean if mean > 0. else 0
    #         min_delta = min(self.times)
    #         max_delta = max(self.times)
    #     return rate, mean, min_delta, max_delta

class Vehicle_checker():
    def __init__(self, VIN, routeId, log, vehiclePoseCounter = -1, vehicleSeqCounter = -1, pose_center_x = 0,  pose_center_y = 0, pose_center_h = 0, \
                pedestrian_check_state = "NORMAL", normal_checker_counter = 0, manual_flag = 1, in_out_diff = 0, in_area_counter = 0, stop_timeout_counter = -1, \
                frame_number = -1, vehicleSpeed = 0, area_located = -1, dif_init_flag = True, exceed_number_check_counter = 0, in_diff_area_counter = 0,\
                clear_in_area_flag = False, clear_check_conter = -1, remote_stop_flag = False, pose_receive_timestamp = 0, engine_state = -1, lose_frame_check = -1,\
                is_vehicle_avoiding = False , vehicle_avoiding_status = "NORMAL" ):
        self.VIN = VIN
        self.routeId = routeId
        self.log = log
        self.vehiclePoseCounter = vehiclePoseCounter
        self.vehicleSeqCounter = vehicleSeqCounter
        self.pedestrianDetectorList = []
        self.pedestrianInList = []
        self.pedestrianInDiffList = []
        self.area_in = []
        self.area_diff = []
        self.area_vehicle_avoid = []
        self.pose_center_x = pose_center_x
        self.pose_center_y = pose_center_y
        self.pose_center_h = pose_center_h
        self.pedestrian_check_state = pedestrian_check_state
        self.normal_checker_counter = normal_checker_counter
        self.exceed_number_check_counter = exceed_number_check_counter
        self.manual_flag = manual_flag
        self.in_out_diff  = in_out_diff
        self.in_area_counter = in_area_counter
        self.in_diff_area_counter = in_diff_area_counter
        self.stop_timeout_counter = stop_timeout_counter
        self.frame_number = frame_number
        self.vehicleSpeed = vehicleSpeed
        self.area_located = area_located
        self.dif_init_flag = dif_init_flag
        self.clear_in_area_flag = clear_in_area_flag
        self.clear_check_conter = clear_check_conter
        self.remote_stop_flag = remote_stop_flag
        self.pose_receive_timestamp = pose_receive_timestamp
        self.max_tracking_distance = default_max_tracking_distance
        self.engine_state = engine_state
        self.lose_frame_check = max_pedestrian_lose_frame_count
        self.perception_vehicle_list = []
        self.is_vehicle_avoiding = is_vehicle_avoiding
        self.vehicle_avoiding_status = vehicle_avoiding_status
        self.targetSlotId = -1
        self.parking_space_expcetion_counter = {}
        self.parking_avoid_location = []
        self.cancel_task_flag = False
        self.main_path_is_complete = False
        self.main_path_check_point = []
        self.last_frame_perception_timestamp = -1
        self.no_perception_detected_counter = 0
        self.pose_receiver_compensate = 0
        self.pose_error_check_counter = 0
        self.active_time = datetime.now()
        self.side_by_side_stop_flag = False
        self.side_by_side_VIN = ""
        self.thead_pool = []
        self.offline_timout_check_counter = vehicle_offline_timeout_time / vehicle_offline_check_time
        self.offline_counter = 0

    # def __del__(self):
    #     self.reset_pedestrian_detect_status()
    #     self.remote_stop_flag = False
    #     for thread in self.thead_pool:
    #         thread.join()

        

    def CancelTask(self,pub):
        drive_job_info = DriveJobInfo()
        drive_job_info.header.seq = 0
        drive_job_info.header.stamp.secs = 0
        drive_job_info.header.stamp.nsecs = 0
        drive_job_info.header.frame_id = self.VIN
        drive_job_info.GarageID = 1
        drive_job_info.RecordIdentifier = 0  
        self.cancel_task_flag = True      
        pub.publish(drive_job_info)
       
##############需要添加与register交互接口 用于单台车的remote start 和 stop###############################
    def RemoteStart(self,pub):
        drive_command_longti_control = DriveCommandLongtiControl()
        drive_command_longti_control.header.frame_id = self.VIN
        drive_command_longti_control.MaxDistanceToDrive = 10
        drive_command_longti_control.MaxVelocity = 2
        pub.publish(drive_command_longti_control)
        self.remote_stop_flag = False
        thread = threading.Thread(target = self.vehicle_start_checker, args=(pub,drive_command_longti_control,))
        thread.setDaemon(True)
        thread.start()

    def RemoteStop(self,pub):
        drive_command_longti_control = DriveCommandLongtiControl()
        drive_command_longti_control.header.frame_id = self.VIN
        drive_command_longti_control.MaxDistanceToDrive = -30
        drive_command_longti_control.MaxVelocity = 0
        pub.publish(drive_command_longti_control)
        self.remote_stop_flag = True
    
    def vehicle_start_checker(self, pub, drive_command_longti_control):
        while self.vehicleSpeed == 0 and self.remote_stop_flag == False:
            pub.publish(drive_command_longti_control)
            if self.vehicleSpeed == 0:
                print_msg(2, "Waiting for vehicle " + self.VIN + " start")
            rospy.sleep(1)

    def vehicle_detection(self):
        if len(self.perception_vehicle_list) == 0:
            return 0
        percepted_vehicle_localization_list = []
        for perception_vehicle in self.perception_vehicle_list:
            is_detected_flag = 0
            # parking_space_no = -1
            heading_p = perception_vehicle[3]
            x_p = perception_vehicle[0]
            y_p = perception_vehicle[1]
            x_v = perception_vehicle[4]
            y_v = perception_vehicle[5]
            obstacle_vehicle_area = []
            exception_points = []
            cos_sita_p = math.cos(heading_p)
            sin_sita_p = math.sin(heading_p)
            matrix_R_p = np.mat([[cos_sita_p, sin_sita_p], [-sin_sita_p, cos_sita_p]])
            matrix_center_p = np.mat([[x_p], [y_p]])
            parking_space_no = self.get_parking_space_no(x_p,y_p)
            if parking_space_no > 0 and parking_space_no == self.targetSlotId:
                return 2
            # time1 = datetime.now() 
            for trans in area_vehicle:
                point_list = []
                point = matrix_R_p.I * np.mat([[trans[0]],[trans[1]]]) + matrix_center_p
                point_list.append(point.tolist()[0][0])
                point_list.append(point.tolist()[1][0])
                obstacle_vehicle_area.append(point_list)
                if self.isInterArea(point_list, self.area_vehicle_avoid):
                    is_detected_flag = 1
                    exception_points.append(point_list)
                    self.write_vehicle_dp_log("obstacle vehicle in self vehicle area", "point: "  + str(point_list[0]) + " " + str(point_list[1]) + " vehicle pose: " + str(self.pose_center_x) + " " + str(self.pose_center_y))

                if parking_space_no > 0 and parking_space_no in self.parking_avoid_location and region2_parking_space.__contains__ (parking_space_no):
                    if self.isInterArea(point_list, region2_parking_space[parking_space_no]) == False:
                        if self.parking_space_expcetion_counter.__contains__(parking_space_no):
                            self.parking_space_expcetion_counter[parking_space_no] = self.parking_space_expcetion_counter[parking_space_no] + 1
                            if self.parking_space_expcetion_counter[parking_space_no] > 5:
                                print_msg (2, "vehicle parking excption :  point: " + str(point_list))
                                print_msg (2, "vehicle parking excption :  parking area: " + str(parking_space_no))
                                self.write_vehicle_dp_log("vehicle parking excption", "point: "  + str(point_list))
                                self.write_vehicle_dp_log("vehicle parking excption", "parking area: "  + str(parking_space_no))
                                self.parking_space_expcetion_counter[parking_space_no] = 0
                                return parking_space_no
                        else:
                            self.parking_space_expcetion_counter[parking_space_no] = 0

                    
            if is_detected_flag == 0 and len(obstacle_vehicle_area) == 4:
                for point in self.area_vehicle_avoid:
                    if self.isInterArea(point, obstacle_vehicle_area):
                        is_detected_flag = 2
                        # exception_points.append(point)
                        self.write_vehicle_dp_log("self vehicle in obstacle vehicle area", "point: "  + str(point[0]) + " " + str(point[1]) + " vehicle pose: " + str(self.pose_center_x) + " " + str(self.pose_center_y))
            check_points = []
            if is_detected_flag == 0:      
                percepted_vehicle_localization_list.append(0)
                continue
            elif is_detected_flag == 1:
                check_points = exception_points
                # check_points = obstacle_vehicle_area
            else:
                check_points = obstacle_vehicle_area                    

            if self.routeId == "line2":
                if self.engine_state == 1 and parking_space_no in self.parking_avoid_location: 
                    return 1                                                                         
                for region in region2_ignore_area:
                    is_in_area = True
                    for point1 in check_points:
                        if self.isInterArea(point1, region) == False:
                            is_in_area = False
                            break
                    if is_in_area == True:
                        percepted_vehicle_localization_list.append(0)                         
                        break
                        # if is_avoiding == True:
                        #     if self.parking_space_expcetion_counter.__contains__(parking_space_no):                              
                        #         self.parking_space_expcetion_counter[parking_space_no] = self.parking_space_expcetion_counter[parking_space_no] + 1
                        #         print "current counter: " +  str(self.parking_space_expcetion_counter[parking_space_no]) + " park_no: " + str(parking_space_no)
                        #         if self.parking_space_expcetion_counter[parking_space_no] > 3:
                        #             self.parking_space_expcetion_counter[parking_space_no] = 0
                        #             self.write_vehicle_dp_log("vehicle avoiding ", "park no: "  + str(parking_space_no))
                        #         else:
                        #             percepted_vehicle_localization_list.append(0)
                        #     else:
                        #         self.parking_space_expcetion_counter[parking_space_no] = 0
                        #         percepted_vehicle_localization_list.append(0)
            else:
                # return 1 
                for region in region1_ignore_area:
                    is_in_area = True
                    for point1 in check_points:
                        if self.isInterArea(point1, region) == False:
                            is_in_area = False
                            break
                    if is_in_area == True:
                        percepted_vehicle_localization_list.append(0)                         
                        break
                                
        
        return 0 if len(percepted_vehicle_localization_list) == len(self.perception_vehicle_list) else 1
    

    def get_parking_space_no(self, perception_x , perception_y ):
        # if self.engine_state != 1 or self.routeId == "line1":
        #     return -1
        point = [perception_x, perception_y]
        if self.routeId == "line1":
            for area_no, area in region1_parking_space.items():
                if self.isInterArea(point, area):
                    return area_no
            return -3
        else:
            if self.engine_state == 1:
                for area_no, area in region2_parking_space.items():
                    if self.isInterArea(point, area):
                        return area_no
                return -3
            else:
                return -1

    def get_parking_check_space(self):
        check_list = []
        if self.targetSlotId == 201:
            check_list.append(202)
        elif self.targetSlotId == 204:
            check_list.append(203)
        elif self.targetSlotId == 223:
            check_list.append(222)
        elif self.targetSlotId == 224:
            check_list.append(225)
        elif self.targetSlotId == 251:
            check_list.append(250)
        elif self.targetSlotId == -1:
            check_list.append(-2)    
        else:           
            check_list.append(self.targetSlotId - 1)
            check_list.append(self.targetSlotId + 1)

        return check_list

    def area_checker(self, pedestrianList):  
        self.in_area_counter = self.pedestrian_counter_in_area_checker(self.area_in, pedestrianList, self.pedestrianInList)
        self.in_diff_area_counter = self.pedestrian_counter_in_area_checker(self.area_diff, pedestrianList, self.pedestrianInDiffList)

    def diff_checker(self, pedestrianList):
        vehicle_center_x_y = [self.pose_center_x, self.pose_center_y]
        ignore_flag = False
        if self.routeId == "line2" and self.pose_center_x > diff_check_route2_x_limit:
            self.in_out_diff = 0
            return

        if len(ignore_detect_area_list) > 0:
            for ignore_detect_area in ignore_detect_area_list:
                if len(ignore_detect_area) == 4:
                    if self.isInterArea_ignorelidar(vehicle_center_x_y, ignore_detect_area):
                        ignore_flag = True
                        break
        
        if ignore_flag == True:
            self.in_out_diff = 0
            return

        area_diff = self.area_diff
        area_in = self.area_in
        if self.dif_init_flag == True:
            self.in_out_diff = self.in_diff_area_counter
            self.dif_init_flag = False
        ot = ObjectionTracker(pedestrianList, self.pedestrianDetectorList)
        self.pedestrianDetectorList = ot.tracker()
        for pedestrian_detector in self.pedestrianDetectorList:
            if pedestrian_detector.isUpdate == True:
                pedestrian_point = []
                pedestrian_point.append(pedestrian_detector.poseX)
                pedestrian_point.append(pedestrian_detector.poseY)
                if self.isInterArea_ignorelidar(pedestrian_point, area_diff):
                    if self.lidar_pole_area_checker_ignore(pedestrian_point):
                        self.in_out_diff = 0
                        return
                    if pedestrian_detector.state == "OUT":
                        pedestrian_detector.state = "UNKNOWN"
                    pedestrian_detector.state_queue_update(1)
                else:
                    pedestrian_detector.state_queue_update(0)               
                pedestrian_detector.timeoutCounter = 0 
        pedestrian_temp = []        
        if len(self.pedestrianDetectorList) > 0:           
            for pedestrian_detector in self.pedestrianDetectorList:
                if pedestrian_detector.isUpdate == True:
                    pedestrian_temp.append(pedestrian_detector) 
                    pedestrian_detector.tracking_distance = default_max_tracking_distance                  
                    tmp_state_list = list(pedestrian_detector.state_queue.queue)
                    #if len(tmp_state_list) == pedestrian_detector.state_queue.maxsize:
                    if len(tmp_state_list) >= 2:
                        if pedestrian_detector.dif_in_out_checker() == 1:
                            if pedestrian_detector.state != "IN":
                                self.in_out_diff = self.in_out_diff + 1  
                                self.write_vehicle_dp_log("Pedestrain In", pedestrian_detector)
                                pedestrian_detector.state = "IN"
                        elif pedestrian_detector.dif_in_out_checker() == 0:
                            if pedestrian_detector.state != "OUT":
                                if self.in_out_diff > 0:
                                    if pedestrian_detector.pedestrian_type == 2:                                            
                                        if self.in_out_diff - self.in_diff_area_counter >= 2:
                                            self.in_out_diff = self.in_out_diff - 2
                                        else:
                                            self.in_out_diff = self.in_out_diff - 1                                                                                       
                                    else:
                                        self.in_out_diff = self.in_out_diff - 1
                                    # self.in_out_diff = self.in_out_diff - 1
                                                                
                                self.write_vehicle_dp_log("Pedestrain Out", pedestrian_detector)
                                pedestrian_detector.state = "OUT"
                                pedestrian_detector.state_queue_set_zero()                        
                else:
                    pedestrian_detector.timeoutCounter = pedestrian_detector.timeoutCounter + 1
                    # pedestrian_detector.tracking_distance = pedestrian_detector.timeoutCounter * 0.4 + default_max_tracking_distance
                    pedestrian_detector.poseX = pedestrian_detector.velX * 0.4 + pedestrian_detector.poseX
                    pedestrian_detector.poseY = pedestrian_detector.velY * 0.4 + pedestrian_detector.poseY
                    # pedestrian_detector.poseX =  pedestrian_detector.poseX
                    # pedestrian_detector.poseY =  pedestrian_detector.poseY
                    if pedestrian_detector.timeoutCounter <= self.lose_frame_check:
                        pedestrian_temp.append(pedestrian_detector)
                    else:
                        # if self.area_located == 4 or self.area_located == 11:
                        tmp_state_list = list(pedestrian_detector.state_queue.queue)
                        if len(tmp_state_list) > 2:
                            pedestrian_point = []
                            pedestrian_point.append(pedestrian_detector.poseX)
                            pedestrian_point.append(pedestrian_detector.poseY)
                            if tmp_state_list[-1] == 1 and self.isInterArea_ignorelidar(pedestrian_point, area_in) == False and pedestrian_detector.state != "OUT":
                                if pedestrian_detector.pedestrian_type == 2:                                            
                                    if self.in_out_diff - self.in_diff_area_counter >= 2:
                                        self.in_out_diff = self.in_out_diff - 2
                                    else:
                                        self.in_out_diff = self.in_out_diff - 1                                                                                       
                                else:
                                    self.in_out_diff = self.in_out_diff - 1
                                pedestrian_detector.state = "OUT"

                                if self.in_out_diff < 0:
                                # if self.in_out_diff < 0:
                                    self.in_out_diff = 0
                                elif self.in_out_diff > 3:
                                    self.in_out_diff = 3       
                                self.write_vehicle_dp_log("Pedestrain Out by Abnormal minus1", pedestrian_detector)
                                pedestrian_detector.state_queue_set_zero()  
                                continue
                        if self.area_located == 4 or self.area_located == 11 or self.area_located == 1 and (len(tmp_state_list) == 2 or len(tmp_state_list) == 1):
                            if 0 not in tmp_state_list and pedestrian_detector.state != "OUT":
                                if pedestrian_detector.pedestrian_type == 2:                                            
                                    if self.in_out_diff - self.in_diff_area_counter >= 2:
                                        self.in_out_diff = self.in_out_diff - 2
                                    else:
                                        self.in_out_diff = self.in_out_diff - 1                                                                                       
                                else:
                                    self.in_out_diff = self.in_out_diff - 1
                                pedestrian_detector.state = "OUT"
                                
                                if self.in_out_diff < 0:
                                # if self.in_out_diff < 0:
                                    self.in_out_diff = 0
                                elif self.in_out_diff > 3:
                                    self.in_out_diff = 3        
                                self.write_vehicle_dp_log("Pedestrain Out by Abnormal minus3", pedestrian_detector)
                                pedestrian_detector.state_queue_set_zero() 

                        pedestrian_detector.timeoutCounter = 0
                        pedestrian_detector.tracking_distance = default_max_tracking_distance
        self.pedestrianDetectorList = pedestrian_temp
        if self.in_out_diff < 0:
            self.in_out_diff = 0
        elif self.in_out_diff > 3:
            self.in_out_diff = 3 

    def pedestrian_counter_in_area_checker(self, area_detect, pedestrianList, pedestrianCheckList):
        in_area_counter = 0
        if len(pedestrianCheckList) > 0:
            for pedestrian_detector in pedestrianCheckList:
                pedestrian_detector.isUpdate = False

        if len(pedestrianList) > 0:
            if len(pedestrianCheckList) == 0:
                for pedestrian in pedestrianList:
                    #if pedestrian[3] == self.area_located:
                        pedestrian_point = []
                        pedestrian_point.append(pedestrian[1])
                        pedestrian_point.append(pedestrian[2])
                        if self.isInterArea_ignorelidar(pedestrian_point, area_detect):
                            pedestrian_detector_new = Pedestrian_Detector(pedestrian[0], pedestrian[1], pedestrian[2], pedestrian[3], pedestrian[4], pedestrian[5], pedestrian[6],  "UNKNOWN")
                            pedestrian_detector_new.in_counter = pedestrian_detector_new.in_counter + 1
                            pedestrianCheckList.append(pedestrian_detector_new)                
            else:
                for pedestrian in pedestrianList:
                    #if pedestrian[3] == self.area_located:
                        for pedestrian_detector in pedestrianCheckList:
                            if self.overlay_checker(pedestrian, pedestrian_detector.poseX, pedestrian_detector.poseY, self.max_tracking_distance):
                                if pedestrian_detector.isUpdate == False:                               
                                    pedestrian_point = []
                                    pedestrian_point.append(pedestrian[1])
                                    pedestrian_point.append(pedestrian[2])
                                    if self.isInterArea_ignorelidar(pedestrian_point, area_detect):
                                        pedestrian_detector.pedestrian_type = pedestrian[0]
                                        pedestrian_detector.poseX = pedestrian[1]
                                        pedestrian_detector.poseY = pedestrian[2]
                                        pedestrian_detector.region = pedestrian[3]
                                        pedestrian_detector.in_counter = pedestrian_detector.in_counter + 1
                                        pedestrian_detector.isUpdate = True
                                        pedestrian_detector.timeoutCounter = 0
                                    else:
                                        pedestrianCheckList.remove(pedestrian_detector)                               
                                break
                            else:
                                if pedestrian_detector == pedestrianCheckList[-1]:
                                    pedestrian_point = []
                                    pedestrian_point.append(pedestrian[1])
                                    pedestrian_point.append(pedestrian[2])
                                    if self.isInterArea_ignorelidar(pedestrian_point, area_detect):
                                        pedestrian_detector_new = Pedestrian_Detector(pedestrian[0], pedestrian[1], pedestrian[2], pedestrian[3],  pedestrian[4], pedestrian[5], pedestrian[6], "UNKNOWN")
                                        pedestrian_detector_new.in_counter = pedestrian_detector_new.in_counter + 1
                                        pedestrianCheckList.append(pedestrian_detector_new)
        else:
            if len(pedestrianCheckList) > 0:
                for pedestrian_detector in pedestrianCheckList:
                    pedestrian_detector.isUpdate = False
        if len(pedestrianCheckList) > 0:
            pedestrian_temp = []
            for pedestrian_detector in pedestrianCheckList:
                if pedestrian_detector.isUpdate == True:
                    if pedestrian_detector.in_counter > max_pedestrian_detect_frame_count:
                        pedestrian_detector.in_counter = 0
                    in_area_counter = in_area_counter + 1
                    pedestrian_temp.append(pedestrian_detector)
                else:
                    pedestrian_detector.timeoutCounter = pedestrian_detector.timeoutCounter + 1
                    if pedestrian_detector.timeoutCounter <= self.lose_frame_check:
                        pedestrian_temp.append(pedestrian_detector) 

            pedestrianCheckList = pedestrian_temp
        return in_area_counter

    def lidar_pole_area_checker(self, testPoint):
        is_in_area = False
        if len(lidar_pole_list) > 0:
            for lidar_pole in lidar_pole_list:
                abs_xx = abs(lidar_pole[0] - testPoint[0])
                abs_yy = abs(lidar_pole[1] - testPoint[1])
                distance = math.sqrt((abs_xx**2) + (abs_yy**2))
                if distance <= 1:
                    is_in_area = True
                    break
                else:
                    is_in_area = False
            return is_in_area
        else:
            return False
    
    def lidar_pole_area_checker_ignore(self, testPoint):
        is_in_area = False
        if len(ignore_lidar_list) > 0:
            for lidar_pole in ignore_lidar_list:
                abs_xx = abs(lidar_pole[0] - testPoint[0])
                abs_yy = abs(lidar_pole[1] - testPoint[1])
                distance = math.sqrt((abs_xx**2) + (abs_yy**2))
                if distance <= lidar_ignore_radius:
                    is_in_area = True
                    break
                else:
                    is_in_area = False
            return is_in_area
        else:
            return False

    def isInterArea_ignorelidar(self,testPoint,AreaPoint):
        if self.isInterArea(testPoint, AreaPoint):
            if self.lidar_pole_area_checker(testPoint):
                return False
            else:
                return True
        else:
            return False
        
    def isInterArea(self,testPoint,AreaPoint):
        LBPoint = AreaPoint[0]
        LTPoint = AreaPoint[1]
        RTPoint = AreaPoint[2]
        RBPoint = AreaPoint[3]
        a = (LTPoint[0]-LBPoint[0])*(testPoint[1]-LBPoint[1])-(LTPoint[1]-LBPoint[1])*(testPoint[0]-LBPoint[0])
        b = (RTPoint[0]-LTPoint[0])*(testPoint[1]-LTPoint[1])-(RTPoint[1]-LTPoint[1])*(testPoint[0]-LTPoint[0])
        c = (RBPoint[0]-RTPoint[0])*(testPoint[1]-RTPoint[1])-(RBPoint[1]-RTPoint[1])*(testPoint[0]-RTPoint[0])
        d = (LBPoint[0]-RBPoint[0])*(testPoint[1]-RBPoint[1])-(LBPoint[1]-RBPoint[1])*(testPoint[0]-RBPoint[0])

        if (a>0 and b>0 and c>0 and d>0) or (a<0 and b<0 and c<0 and d<0):
            return True
        else:
            return False 


    def reorder_clockwise(self,polygon_point):
        pp = np.array(polygon_point)  
        
        if (pp[0] == pp[-1]).all():
            pp = np.delete(pp, -1, axis=0)
        x = pp[:, 0]
        y = pp[:, 1]
        
        max_y_index = np.argmax(y)    
        
        pre_index = max_y_index -1
        next_index = 0 if max_y_index == len(pp) - 1 else max_y_index + 1        
    
        if x[pre_index] < x[next_index]:

            return polygon_point
        else:

            return polygon_point[::-1]
    
    
    def is_inpolygon(self, ploygon_point, scatter_point, border=True):

        new_pp = self.reorder_clockwise(ploygon_point)
        pp = np.array(new_pp)
        sp = np.array(scatter_point)
        if (pp[0] == pp[-1]).all():
            pp = np.delete(pp, -1, axis=0)    
        x = pp[:, 0]
        y = pp[:, 1]
        px = sp[0]
        py = sp[1]
        dy = y - py   
        dy_next = dy.copy()
        dy_next[0] = dy[-1]
        dy_next[1:] = dy[:-1]
        dy_mu = dy * dy_next   
        negative_index = np.where(dy_mu <=0)
        if len(negative_index) >0 :
            isp = pp[np.where(dy_mu <= 0)]
            
            if not border:

                isp = np.unique(isp, axis=0)
                left_num = len(isp[np.where(isp[:, 0] <= px)]) 
            else:
                left_num = len(isp[np.where(isp[:, 0] < px)])  
    
            if left_num % 2 ==0 :
                return False
            else:
                return True
    
        else:
            return False

    def overlay_checker(self, pedestrian, poseX, poseY, trackingDistance):
        abs_xx = abs(pedestrian[1] - poseX)
        abs_yy = abs(pedestrian[2] - poseY)
        distance = math.sqrt((abs_xx**2) + (abs_yy**2))
        if distance < trackingDistance:
            return True
        else:
            return False  

    def reset_pedestrian_detect_status(self):
        self.pedestrianInList = []
        self.pedestrianInDiffList = []
        self.pedestrianDetectorList = []
        self.in_out_diff = 0  
        self.stop_timeout_counter = 0
        self.dif_init_flag = True                                              
        self.pedestrian_check_state = "NORMAL"
        self.clear_in_area_flag = False
        self.clear_check_conter = 0
        self.pose_receive_timestamp = 0
        self.pose_receiver_compensate = 0.4
        self.remote_stop_flag = True

    def pedestrian_region_filter(self, pedestrianList):
        self.set_area_parameter()
        pedestrian_temp = []
        pedestrian_check_temp = deepcopy(pedestrianList)
        if self.routeId == "line1":
            region_list = region1_list
        else:
            region_list = region2_list
        if len(pedestrianList) > 0:
            for pedestrian in pedestrianList:
                if pedestrian[3] in region_list.keys():
                    pedestrian_point = []
                    pedestrian_point.append(pedestrian[1])
                    pedestrian_point.append(pedestrian[2])
                    # if self.isInterArea(pedestrian_point, region_list[pedestrian[3]]) == True:
                    if self.is_inpolygon(region_list[pedestrian[3]], pedestrian_point) == True:
                        pedestrian_temp.append(pedestrian)
                    else:
                        overlay_counter = 0
                        for pedestrian_check in pedestrian_check_temp:
                            if self.overlay_checker(pedestrian, pedestrian_check[1], pedestrian_check[2], 1) == True:
                                overlay_counter = overlay_counter + 1
                        if overlay_counter == 1 and (pedestrian[0] != 2 or not self.isInterArea(pedestrian_point, self.area_in)):
                            pedestrian_temp.append(pedestrian)
                else:
                    print_msg (1, "region number out of range")
        return pedestrian_temp

    def set_area_parameter(self):
        area_vehicle_avoid_check = []
        pedestrian_temp = []
        self.area_in = []
        self.area_diff = []
        self.area_vehicle_avoid = []
        cos_sita = math.cos(self.pose_center_h)
        sin_sita = math.sin(self.pose_center_h)
        matrix_R = np.mat([[cos_sita, sin_sita], [-sin_sita, cos_sita]])
        matrix_center = np.mat([[self.pose_center_x], [self.pose_center_y]])
        self.pose_receiver_compensate = 0.4
        if self.routeId == "line1":
            area_vehicle_avoid_check = area_vehicle_avoid_route1
            parking_space = region1_parking_space
            self.parking_avoid_location = []
            if self.area_located == 1:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_1_route1
                area_diff_size = diff_area_1_route1
            elif self.area_located == 2:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_2_route1
                area_diff_size = diff_area_2_route1
            elif self.area_located == 3 or self.area_located == 4:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_route1
                area_diff_size = diff_area_route1
            elif  self.area_located == 6 or self.area_located == 7:
                self.lose_frame_check = 3
                area_in_size = area_pedestrian_checker_origin_5_6_7_8_route1
                area_diff_size = diff_area_5_6_7_8_route1
            elif self.area_located == 5 or self.area_located == 8:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_5_6_7_8_route1
                area_diff_size = diff_area_5_6_7_8_route1
            elif self.area_located == 11 or self.area_located == 12 or self.area_located == 13 or self.area_located == 14:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_11_12_13_14_route1
                area_diff_size = diff_area_11_12_13_14_route1
            else:
                self.lose_frame_check = max_pedestrian_lose_frame_count
                area_in_size = area_pedestrian_checker_origin_route1
                area_diff_size = diff_area_route1
        else:
            parking_space = region2_parking_space
            self.parking_avoid_location = self.get_parking_check_space()
            if self.engine_state == 1:
                area_in_size = area_pedestrian_checker_origin_R_route2
                area_diff_size = diff_area_R_route2
                area_vehicle_avoid_check = area_vehicle_avoid_R_route2
            else:
                area_in_size = area_pedestrian_checker_origin_route2
                area_diff_size = diff_area_route2
                area_vehicle_avoid_check = area_vehicle_avoid_route2

        for trans in area_in_size:
            point_list = []
            point = matrix_R.I * np.mat([[trans[0]],[trans[1]]]) + matrix_center
            point_list.append(point.tolist()[0][0])
            point_list.append(point.tolist()[1][0])
            self.area_in.append(point_list) 

        for trans in area_diff_size:
            point_list = []
            point = matrix_R.I * np.mat([[trans[0]],[trans[1]]]) + matrix_center
            point_list.append(point.tolist()[0][0])
            point_list.append(point.tolist()[1][0])
            self.area_diff.append(point_list) 

        for trans in area_vehicle_avoid_check:
            point_list = []
            point = matrix_R.I * np.mat([[trans[0]],[trans[1]]]) + matrix_center
            point_list.append(point.tolist()[0][0])
            point_list.append(point.tolist()[1][0])
            self.area_vehicle_avoid.append(point_list) 

        if len(self.main_path_check_point) == 3:
            if self.main_path_is_complete == False:
                self.main_path_is_complete = self.overlay_checker(self.main_path_check_point, self.pose_center_x, self.pose_center_y, 10)
                if self.main_path_is_complete == True:
                    print_msg(0, self.VIN + " reach check point") 
                    # self.write_vehicle_diagnose_log("000001", " reach check point")
            else:
                pass
        else:
            if parking_space.__contains__(self.targetSlotId):
                self.main_path_check_point.append(0)
                # if self.targetSlotId == 201 or self.targetSlotId == 202 or self.targetSlotId == 203 or self.targetSlotId == 204:
                if self.targetSlotId == 251 or self.targetSlotId == 202 or self.targetSlotId == 203 or self.targetSlotId == 204:
                    self.main_path_check_point.append(50)
                    self.main_path_check_point.append(parking_space[204][3][1] - abs(parking_space[204][0][1] - parking_space[204][3][1])/2)
                elif (self.targetSlotId > 204 and self.targetSlotId < 224) or self.targetSlotId == 201:
                    self.main_path_check_point.append(50)
                    self.main_path_check_point.append(parking_space[self.targetSlotId][3][1] + abs(parking_space[self.targetSlotId][0][1] - parking_space[self.targetSlotId][3][1])/2)
                elif self.targetSlotId <= 105 and self.targetSlotId >= 101:
                    self.main_path_check_point.append(-115)
                    self.main_path_check_point.append(146)
                else:
                    self.main_path_check_point.append(68)
                    self.main_path_check_point.append(parking_space[self.targetSlotId][3][1] - abs(parking_space[self.targetSlotId][0][1] - parking_space[self.targetSlotId][3][1])/2)
                # print_msg(2, self.VIN + " check point: " + str(self.main_path_check_point)) 
            else:
                self.main_path_check_point = []
            
    def is_ignore_pose_error_checker(self):
        if self.routeId == "line1":
            if self.pose_center_y > 10:
                return True
            else:
                return False
             
        else:
            if self.pose_center_x > 30:
                return True
            else:
                return False

    def tracking_distance_controller(self):
        if self.pose_receive_timestamp == 0:
            self.pose_receive_timestamp = time.time()
            self.max_tracking_distance = default_max_tracking_distance
            return False
        if time.time() - self.pose_receive_timestamp < 10:
            if time.time() - self.pose_receive_timestamp > 0.7:
                self.max_tracking_distance = (time.time() - self.pose_receive_timestamp) + default_max_tracking_distance
                self.pose_receiver_compensate =  (time.time() - self.pose_receive_timestamp) + 0.4
            if self.pose_receiver_compensate > 2:
                self.pose_receiver_compensate = 2
            if self.max_tracking_distance > 2:
                self.max_tracking_distance = 2
            self.pose_receive_timestamp = time.time()
            return False
        else:
            self.pose_receive_timestamp = time.time()
            return True

    def write_vehicle_dp_log(self, title, info):
        if self.log.path_vehicle_pd_path_dict.__contains__(self.VIN):       
            self.log.write_dp_log(self.VIN, "--------------", title, "--------------") 
            if type(info) == str:
                self.log.write_dp_log(self.VIN,"--------------", info, "--------------") 
            elif type(info) == list:
                for pedestrianDetecor in info:
                    temp_list = list(pedestrianDetecor.state_queue.queue) 
                    self.log.write_dp_log(self.VIN, "vehicle x: " + str(self.pose_center_x) + "   " + "vehicle y: " + str(self.pose_center_y) + "   " + "in_out_dif" + "   " + str(self.in_out_diff)+ "   " , "id: " + str(pedestrianDetecor)+ "   " , \
                                    "queue: " + str(temp_list) + "    " + "type: " + str(pedestrianDetecor.pedestrian_type) + "   " + "x: " + str(pedestrianDetecor.poseX) + "    " + "y: " + str(pedestrianDetecor.poseY) + "    " + "region: " + str(pedestrianDetecor.region)+ "    " + "is_update" + str(pedestrianDetecor.isUpdate)) 
            else:
                temp_list = list(info.state_queue.queue)
                self.log.write_dp_log(self.VIN, "vehicle x: " + str(self.pose_center_x) + "   " + "vehicle y: " + str(self.pose_center_y) + "   " + "in_out_dif" + "   " + str(self.in_out_diff)+ "   ", "id: " + str(info)+ "   "  , \
                                    "queue: " + str(temp_list) + "    " + "type: " + str(info.pedestrian_type) + "   " + "x: " + str(info.poseX) + "    " + "y: " + str(info.poseY) + "    " + "region: " + str(info.region)+ "    " + "is_update" + str(info.isUpdate)) 
            self.log.write_dp_log(self.VIN, "--------------", "--------------", "--------------")
            self.log.write_dp_log(self.VIN, "", "", "")
            self.log.write_dp_log(self.VIN, "", "", "")
        else:
            print_msg(2, "waiting for log file creating")

    def write_vehicle_diagnose_log(self, errorCode, diagnoseDescription):
        # cancelReason, errorDescription, diagnoseType = diagnoseDescription.split(r"\\")
        self.log.write_diagnose_log(self.VIN, errorCode, "", diagnoseDescription, "")

class ObjectionTracker():
    def __init__(self, pedestrianList, pedestrian_detector_list):
        self.pedestrianList = pedestrianList
        self.pedestrian_detector_list = pedestrian_detector_list
        self.match_init_list = []
        self.pedestrian_match_list = []
        self.detector_list = []
        self.visited = []
        self.detector_tmp_list = []
        self.init_match()

    def find(self, j):
        for i in range(len(self.pedestrianList)):
            if (i in self.match_init_list[j]) and self.visited[i] == 0:
                self.visited[i] = 1
                if self.pedestrian_match_list[i] == -1 or self.find(self.pedestrian_match_list[i]):
                    self.pedestrian_match_list[i] = j
                    return True
        return False

    def tracker(self):
        result = 0
        pedestrian_match_list_list = []
        counter_list = []
        for i in range(len(self.pedestrianList)):
            self.pedestrian_match_list.append(-1)
        for pedestrian_detector in self.pedestrian_detector_list:
            pedestrian_detector.isUpdate = False          
        for j in range(len(self.pedestrian_detector_list)):
            for k in range(len(self.pedestrianList)):
                self.visited.append(0)
            if self.find(j):
                result = result + 1
                pedestrian_match_list_list.append(self.pedestrian_match_list)
        if len(pedestrian_match_list_list) > 0:  
            for k in range(len(pedestrian_match_list_list)):
                counter = 0
                counter = pedestrian_match_list_list[k].count(-1)
                counter_list.append(counter)
            index = counter_list.index(min(counter_list)) 
            update_index_list = pedestrian_match_list_list[index]
            for h in range(len(update_index_list)):
                if update_index_list[h] == -1:
                    continue
                self.pedestrian_detector_list[update_index_list[h]].isUpdate = True
                self.pedestrian_detector_list[update_index_list[h]].pedestrian_type = self.pedestrianList[h][0]
                self.pedestrian_detector_list[update_index_list[h]].poseX = self.pedestrianList[h][1]
                self.pedestrian_detector_list[update_index_list[h]].poseY = self.pedestrianList[h][2]  
                self.pedestrian_detector_list[update_index_list[h]].region = self.pedestrianList[h][3]
                self.pedestrian_detector_list[update_index_list[h]].velX = self.pedestrianList[h][4]  
                self.pedestrian_detector_list[update_index_list[h]].velY = self.pedestrianList[h][5]
                self.detector_tmp_list.append(self.pedestrian_detector_list[update_index_list[h]])

            for pedestrian_detector in self.pedestrian_detector_list:
                if pedestrian_detector.isUpdate == False:
                    self.detector_tmp_list.append(pedestrian_detector)           
        return self.detector_tmp_list


    def init_match(self):
        for k in range(len(self.pedestrian_detector_list)): 
            self.match_init_list.append([])
        if len(self.pedestrianList) > 0:                
            for i in range(len(self.pedestrianList)):
                is_updated = False
                for j in range(len(self.pedestrian_detector_list)):
                    dis = self.overlay_checker_m(self.pedestrianList[i], self.pedestrian_detector_list[j])
                    # if  dis < 300 and dis > 0:
                    if dis < 30 and dis > 0:
                        is_updated = True
                        self.match_init_list[j].append(i)
                if is_updated == False:
                    pedestrian_detector_new = Pedestrian_Detector(self.pedestrianList[i][0], self.pedestrianList[i][1], self.pedestrianList[i][2], self.pedestrianList[i][3], self.pedestrianList[i][4], self.pedestrianList[i][5], self.pedestrianList[i][6],"UNKNOWN")
                    self.detector_tmp_list.append(pedestrian_detector_new)

    def overlay_checker(self, pedestrian, poseX, poseY, trackingDistance):
        abs_xx = abs(pedestrian[1] - poseX)
        abs_yy = abs(pedestrian[2] - poseY)
        distance = math.sqrt((abs_xx**2) + (abs_yy**2))
        if distance < trackingDistance:
            return True
        else:
            return False  


    def overlay_checker_m(self, pedestrian, pedestrian_detector):
        dt = 0.1
        sigmaA = 1.5
        # S = np.matrix([[(dt**4)/4, 0, (dt**3)/2, 0],
        #                 [0, (dt**4)/4, 0, (dt**3)/2],
        #                 [(dt**3)/2, 0, dt**2, 0],
        #                 [0, (dt**3)/2, 0, dt**2]]) * (sigmaA **2)
        S = np.matrix([[(dt**4 + 4*(dt**3)+ 4*(dt**2))/4, 0, (dt**3 + 2 * (dt**2))/2, 0],
                        [0, (dt**4 + 4*(dt**3)+ 4*(dt**2))/4, 0, (dt**3 + 2 * (dt**2))/2],
                        [(dt**3 + 2 * (dt**2))/2, 0, dt**2, 0],
                        [0, (dt**3 + 2 * (dt**2))/2, 0, dt**2]]) * (sigmaA **2)

        SI = np.linalg.inv(S)
        abs_xx = pedestrian[1] - pedestrian_detector.poseX
        abs_yy = pedestrian[2] - pedestrian_detector.poseY
        abs_xx_v = pedestrian[4] - pedestrian_detector.velX
        abs_yy_v = pedestrian[5] - pedestrian_detector.velY
        dis = np.matrix([[abs_xx], [abs_yy], [abs_xx_v], [abs_yy_v]])
        Maha_distance =float(np.sqrt(abs(dis.T * SI* dis))) 
        return Maha_distance / (10000000)
        
class Pedestrian_Detector():
    def __init__(self, pedestrian_type, poseX, poseY,  region, velX, velY, detect_type_conf, state, isUpdate = True, timeoutCounter = -1, in_counter = 0 ):
        self.pedestrian_type = pedestrian_type
        self.isUpdate = isUpdate
        self.timeoutCounter = timeoutCounter
        self.poseX = poseX
        self.poseY = poseY
        self.velX = velX
        self.velY = velY
        self.state = state
        self.in_counter = in_counter
        self.state_queue = Queue.Queue(maxsize = max_diff_queue_size)
        self.region = region 
        self.tracking_distance = default_max_tracking_distance
        self.pose_queue = Queue.Queue(maxsize = 6)
        self.detect_type_conf = detect_type_conf

    def state_queue_update(self, state):
        if self.state_queue.full():
            self.state_queue.get()
            self.state_queue.put(state)
        else:
            self.state_queue.put(state)

    def pose_queue_update(self, pose):
        if self.pose_queue.full():
            self.pose_queue.get()
            self.pose_queue.put(pose)
        else:
            self.pose_queue.put(pose)

    def get_pose_cov(self):
        tmp_list = list(self.pose_queue.queue)
        mat = np.matrix(tmp_list)
        cov = []
        if len(tmp_list) == 6:
            cov = np.cov(mat.T)
        return cov

    def state_queue_set_zero(self):
        tmp_state_list = list(self.state_queue.queue)
        for i in range(len(tmp_state_list)):
            self.state_queue.get()
            self.state_queue.put(0)

    def dif_in_out_checker(self):
        # temp_list = []
        temp_list = list(self.state_queue.queue)
        if self.state_queue.full():            
            half_size = self.state_queue.maxsize / 2
            # if (0 not in temp_list[0:int(half_size)]) and (1 not in temp_list[int(half_size):]):
            #     return 0
            if (1 not in temp_list[0:int(half_size)]) and (0 not in temp_list[int(half_size):]):
                return 1
            elif (0 not in temp_list[0:int(half_size)]) and (1 not in temp_list[int(half_size):]):
                return 0
            elif temp_list[-1] == 0 and temp_list[-2] == 1:
                return 0
            # elif temp_list[1] == 0 and temp_list[0] == 1:
            #     return 0
            else:
                return -1
        else:
            if len(temp_list) >= 2 and temp_list[-1] == 0 and temp_list[-2] == 1:
                # if len(temp_list) == 3 and temp_list[-1] == 0 and temp_list[-2] == 1 and temp_list[-2] == 0:
                #     return -1
                # else:
                return 0
    
    # def lidar_pole_area_checker(self):
    #     is_in_area = False
    #     if len(lidar_pole_list) > 0:
    #         for lidar_pole in lidar_pole_list:
    #             abs_xx = abs(lidar_pole[0] - self.poseX)
    #             abs_yy = abs(lidar_pole[1] - self.poseY)
    #             distance = math.sqrt((abs_xx**2) + (abs_yy**2))
    #             if distance <= 0.8:
    #                 is_in_area = True
    #                 break
    #             else:
    #                 is_in_area = False
    #         return is_in_area
    #     else:
    #         return False


class HVP_Diagnose():
    def __init__(self):
        self.vehicle_diagnose_list = []

        #self.pub_network_diagnose = rospy.Publisher('/diagnose', Diagnose, queue_size=10)
        self.pub_diagnose = rospy.Publisher('/diagnose', Diagnose, queue_size=1)
        #self.pub_lidar_status = rospy.Publisher('/diagnose', Diagnose, queue_size=10)
        #self.pub_car_status = rospy.Publisher('/diagnose', Diagnose, queue_size=10)
        self.pub_remove_route1_vehicle = rospy.Publisher("/hvp_fusion_path1/server/removevehinfo", VehInfo, queue_size=1)
        self.pub_remove_route2_vehicle = rospy.Publisher("/hvp_fusion_path2/server/removevehinfo", VehInfo, queue_size=1)
        self.pub_remove_route3_vehicle = rospy.Publisher("/hvp_fusion_path3/server/removevehinfo", VehInfo, queue_size=1)
        self.pub_cancel_task = rospy.Publisher('/mec_1_DriveJobInfo', DriveJobInfo, queue_size=1)
        self.pub_remote_stop = rospy.Publisher('/mec_1_DriveCommandLongtiControl', DriveCommandLongtiControl, queue_size=1)
        self.pub_remote_start = rospy.Publisher('/mec_1_DriveCommandLongtiControl', DriveCommandLongtiControl, queue_size=1)

        self.log = Logger()
        self.log.write_diagnose_log("COMMON", "errorCode", "CancelReason", "errorDescription", "diagnoseType")
        self.dic_lidar_topic = {}
        self.dic_vehicle = {}
        self.dic_rsu_heart = {}
        self.dic_diagnose ={}
        self.dic_lidar_exception_counter = {}
        self.network_check_total_pack = {}
        self.network_check_total_pack_loss = {}
        self.vehicle_checker_list = []
        # self.path1_trajectory_points = []
        # self.path2_trajectory_points = []
        self.path_trajectory_points_route1 = []
        self.path_trajectory_points_route2 = []
        self.pedestrian_list_route1 = []
        self.pedestrian_list_route2 = []
        self.routeList = []
        self.is_pose_error = False
        self.close_route1_flag = False
        self.close_route2_flag = False
        self.start_check_pad1_flag = False
        self.start_check_pad2_flag = False
        self.pad_route1_heart_counter = 0
        self.pad_route2_heart_counter = 0

        self.init_lidar_topic_list()
        self.ReadVehicleDiagnoseList()
        # self.create_log_file()
        self.init_trajectory_points()
        self.init_slot_route_info()
        
        self.lock = threading.Lock()
        self.network_check_ip_list = network_check_ip_list
        for ip in self.network_check_ip_list:
            self.network_check_total_pack[ip] = 0
            self.network_check_total_pack_loss[ip] = 0
        for ip_list in region1_ip_list.values():
            for ip in ip_list:
                self.network_check_total_pack[ip] = 0
                self.network_check_total_pack_loss[ip] = 0
        for ip_list in region2_ip_list.values():
            for ip in ip_list:
                self.network_check_total_pack[ip] = 0
                self.network_check_total_pack_loss[ip] = 0
        

                
    def init_lidar_topic_list(self):
        dic_lidar_topic_msg_tmp = {}
        with open(path_lidar_topic_list, "r") as topic_list:
            lidar_list_tmp = topic_list.readlines()
            if len(lidar_list_tmp)>0:
                for lidar_topic in lidar_list_tmp:
                    lidar_topic = lidar_topic.strip('\n')
                    topic = lidar_topic.split(r"\\")[0]
                    msg_type = lidar_topic.split(r"\\")[1]
                    dic_lidar_topic_msg_tmp[topic] = msg_type
            else:
                print_msg (1, "Lidar list is empty") 
        for topic, msg_type_str in dic_lidar_topic_msg_tmp.items():
            topic_info = Topic_Info(topic, msg_type_str)
            self.dic_lidar_topic[topic] = topic_info
            self.dic_lidar_exception_counter[topic] = 0
        # topic = '/percept_topic_0'
        # msg_type_str = 'perception_msg/PerceptionMsg'
        # topic_info = Topic_Info(topic, msg_type_str)
        # self.dic_lidar_topic[topic] = topic_info

    def init_trajectory_points(self):
        with open(path_trajectory_file_path1, "r") as path1_list:
            tracjory1_points = path1_list.readlines()
            for line in tracjory1_points:
                line = line.split(" ")
                if line[0].isdigit():
                    point_x_y = []
                    point_x_y.append(line[0])
                    point_x_y.append(line[1])
                    self.path_trajectory_points_route1.append(point_x_y)

        f = csv.reader(open(path_trajectory_file_path2,'r'))
        for line in f:
            point_x_y = []
            point_x_y.append(line[0])
            point_x_y.append(line[1])
            self.path_trajectory_points_route2.append(point_x_y)

    def init_slot_route_info(self):
        routeInfo1 = RouteInfo_ForHMI()
        routeInfo1.routeId = "line1"
        routeInfo1.state = "On"
        routeInfo2 = RouteInfo_ForHMI()
        routeInfo2.routeId = "line2"
        routeInfo2.state = "On"
        routeInfo3 = RouteInfo_ForHMI()
        routeInfo3.routeId = "line3"
        routeInfo3.state = "Off"
        self.routeList.append(routeInfo1)
        self.routeList.append(routeInfo2)
        self.routeList.append(routeInfo3)


    def get_lidar_hz(self):
        rospy.rostime.wallsleep(3)

        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(1)
            if len(self.dic_lidar_topic) > 0:
                for topic_name, topicInfo in self.dic_lidar_topic.items():
                    if topicInfo.monitoring:    
                        ret = topicInfo.get_hz()
                        # rate, min_delta, max_delta, std_dev, n = ret
                        # if topic_name == "/percept_topic_parkzone2":
                        #     print_msg (2, rate) 
                        if ret != None:
                            if self.dic_lidar_exception_counter.__contains__ (topic_name):
                                self.dic_lidar_exception_counter[topic_name] = 0
                            rate, min_delta, max_delta, std_dev, n = ret
                            if rate < lidar_offline_rate_standard:
                               # print_msg (1, topic_name + ": " + str(rate))
                                lidar_number = "Region " + topic_name.split('_')[-1]
                                # self.publish_diagnose("000001", lidar_number + " Abnormal lidar rate " )
                                # print_msg (1, lidar_number + " Abnormal lidar rate " + str(rate)) 
                        else:
                            if self.dic_lidar_exception_counter.__contains__ (topic_name):
                                self.dic_lidar_exception_counter[topic_name] = 1 + self.dic_lidar_exception_counter[topic_name]
                                if self.dic_lidar_exception_counter[topic_name] > 4:
                                    if "parkzone" not in str(topic_name):
                                        log = "Area1" + " Region " + topic_name.split('_')[-1] + " Not Received!"
                                        self.publish_diagnose_system("000001", log,)
                                        print_msg (1, log)
                                        self.log.write_diagnose_log("COMMON","", "start check line1 region " + str(topic_name.split('_')[-1]),"","")
                                        for ip in region1_ip_list[int(topic_name.split('_')[-1][-1])]:
                                            self.network_loss_check(ip)
                                        self.log.write_diagnose_log("COMMON","", "check line1 region " + str(topic_name.split('_')[-1]) + " complete","","")
                                    else:
                                        log = "Area2" + " Region " + topic_name.split('_')[-1] + " Not Received!"
                                        self.publish_diagnose_system("000001", log,)
                                        print_msg (1, log)
                                        self.log.write_diagnose_log("COMMON","", "start check line2 region " + str(topic_name.split('_')[-1]),"","")
                                        for ip in region2_ip_list[int(topic_name.split('_')[-1][-1])]:
                                            self.network_loss_check(ip)
                                        self.log.write_diagnose_log("COMMON","", "check line2 region " + str(topic_name.split('_')[-1]) + " complete","","")


    def ReadVehicleDiagnoseList(self):
        f = csv.reader(open(path_diagnose_file,'r'))
        for line in f:
            if line[0] != 'Number':
                vehicle_diagnose = Vehicle_Diagnose(line[0],line[1],line[2])
                self.vehicle_diagnose_list.append(vehicle_diagnose)

    def create_log_file(self, VIN):
        self.log.create_vehicle_folder(VIN)
        self.log.write_dp_log(VIN, "vehicleStatus", "pedestrianId", "pedestrianStatus")
        self.log.write_diagnose_log(VIN, "errorCode", "CancelReason", "errorDescription", "diagnoseType")

    def network_loss_check(self, ip_address):
        if ip_address not in self.network_check_total_pack or ip_address not in self.network_check_total_pack_loss:
            print (ip_address + " not exist")
            return 
        self.network_check_total_pack[ip_address] = self.network_check_total_pack[ip_address] + 1
        p = subprocess.Popen(
            "ping -c 3 {0} \n".format(ip_address),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True)
        out = p.stdout.read().decode('gbk')
        regIP = r'\d+\.\d+\.\d+\.\d+'
        regLost = r', (\d.*?%) packet loss'
        regAverage = r'= (.*?) ms'
        ip = re.search(regIP, out)
        lost = re.findall(regLost, out)[0]
        reaverage = re.findall(regAverage, out)    
        
        if ip:
            ip = ip.group()
            if len(reaverage) ==0:
                self.network_check_total_pack_loss[ip_address] = self.network_check_total_pack_loss[ip_address] + 1
                total_loss_rate = self.network_check_total_pack_loss[ip_address] / self.network_check_total_pack[ip_address]
                # print total_loss
                # print total_pack
                log = ip_address + " lost:100 result:全部丢包" + " total:" + str(total_loss_rate)
                # print (log)
                self.log.write_diagnose_log("COMMON","", log,"","")
                return
            average = float(reaverage[0].split('/')[1])
            if lost:
                lost = lost.split(',')[-1].lstrip()
                lost = float(lost.replace('%', ''))
                lost_rate = lost/100
                self.network_check_total_pack_loss[ip_address] = self.network_check_total_pack_loss[ip_address] + lost_rate
                total_loss_rate = self.network_check_total_pack_loss[ip_address] / self.network_check_total_pack[ip_address]
                if lost >= 10 :
                    log = ip_address + " lost: " + str(lost) + " result:发生丢包" + " avarge:"  + str(average)+ " total:" + str(total_loss_rate)
                    # print(log)
                    self.log.write_diagnose_log("COMMON","", log,"","")
                    return                
                if average >= 300:
                    log = ip_address + " lost: " + str(lost) + " result:网络延时过大" + " avarge:"  + str(average) + " total:" + str(total_loss_rate)
                    # print(log)
                    self.log.write_diagnose_log("COMMON","", log,"","")
                    return 

    def network_checker(self):
        while True:      
            for ip in self.network_check_ip_list:
                self.network_loss_check(ip)
            

    def vehicle_tbox_checker(self):
        rospy.Subscriber('/mec_1_VehicleState', VehicleState, self.vehicle_state_callback, queue_size = 1)
        # rospy.Subscriber("/mec_1_VehiclePose", VehiclePose, self.vehicle_pose_tbox_callback, queue_size = 1, tcp_nodelay=True)
        rospy.Subscriber('/hvp_fusion_path1/server/removevehinfo', VehInfo, self.route_task_cancel_callback, queue_size = 1)
        rospy.Subscriber('/hvp_fusion_path2/server/removevehinfo', VehInfo, self.route_task_cancel_callback, queue_size = 1)
        rospy.Subscriber('/terminal/taskrequest', TaskRequest, self.active_callback, queue_size = 1) 
        rospy.Subscriber('/mec_1_SenseingPose', SenseingPose, self.vehicle_perception_compare, queue_size = 1)
        rospy.Subscriber('/hvp_fusion_path1/fusion_states', FusionStates, self.fusion_car_state_callback, queue_size = 1)
        rospy.Subscriber('/manual_operation', PowerStatus, self.remote_stop_start_callback, queue_size = 1)
        rospy.Subscriber("/hvp_fusion_path1/fusion_out", PointCloud, self.fusion_out_callback_route1, queue_size = 1)
        rospy.Subscriber("/hvp_fusion_path2/fusion_out", PointCloud, self.fusion_out_callback_route2, queue_size = 1)
        rospy.Subscriber("/mec_1_AVPVCU", AVPVCU, self.getAVPVCU_callback, queue_size = 1)
        rospy.Subscriber("/mec_1_VehicleTargetSlot", TargetSlot, self.vehicle_target_slot_callback, queue_size = 1)
        rospy.Subscriber("/padHeart1", RSUHeart, self.pad_heart_callback1, queue_size = 10)
        rospy.Subscriber("/padHeart2", RSUHeart, self.pad_heart_callback2, queue_size = 10)
        rospy.Subscriber('/routeInfoList', RouteInfoSet_ForHMI, self.route_state_callback, queue_size=5)

        if len(self.dic_lidar_topic)>0:
            for topicInfo in self.dic_lidar_topic.values():
                topicInfo.start_monitoring()
        rospy.spin()

    def start_lidar_hz_listen(self):
        if len(self.dic_lidar_topic)>0:
            for topicInfo in self.dic_lidar_topic.values():
                topicInfo.start_monitoring()
        
        rospy.spin()

    def vehicle_pose_checker(self):
        rospy.Subscriber("/mec_1_VehiclePose", VehiclePose, self.vehicle_pose_tbox_callback, queue_size = 1)

    def vehicle_target_slot_callback(self, data):
        if len(self.vehicle_checker_list):
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.VIN == data.header.frame_id:
                    vehicle_checker.targetSlotId = data.targetSlotId
                    break

    def remote_stop_start_callback(self, data):
        for vehicle_checker in self.vehicle_checker_list:
            if vehicle_checker.VIN == data.VIN:
                if data.status == "MANUALSTOPPED":
                    vehicle_checker.manual_flag = 1
                    self.publish_diagnose_VIN(vehicle_checker, "000002", "Remote Stop By manual", "", "dp")
                    # vehicle_checker.write_vehicle_dp_log("Remote Stop By manual",  "--------------------")
                    #vehicle_checker.pedestrian_check_state = "MANUALSTOPPED"
                elif data.status == "MANUALSTARTED":
                    vehicle_checker.manual_flag = 0
                    self.publish_diagnose_VIN(vehicle_checker, "000002", "Remote Start By manual", "", "dp")
                    # vehicle_checker.write_vehicle_dp_log("Remote Start By manual", "--------------------")
                    #vehicle_checker.pedestrian_check_state = "NORMAL"
                break

    def getAVPVCU_callback(self, data):
        if len(self.vehicle_checker_list) >0:
            for vehicle_checker in self.vehicle_checker_list:
                if data.header.frame_id == vehicle_checker.VIN:
                    vehicle_checker.vehicleSpeed = data.sigVCUVehcleSpeed
                    vehicle_checker.engine_state = data.sigAVPVCUEngineState
                    break

    def active_callback(self, data):
        routeId_common = "line" + str(data.TernimalID)
        active_message = routeId_common + " " + data.VIN + " Actived"
        print_msg (2, active_message)
        self.log.write_diagnose_log("COMMON","", active_message,"","")
        if len(self.vehicle_checker_list) >0:
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.VIN == data.VIN:
                    return
                else:
                    if vehicle_checker == self.vehicle_checker_list[-1]:
                        routeId = "line" + str(data.TernimalID)
                        vehicle_checker_new = Vehicle_checker(data.VIN, 
                                                                routeId,
                                                                self.log)
                        self.vehicle_checker_list.append(vehicle_checker_new)
                        self.create_log_file(data.VIN)
        else:
            routeId = "line" + str(data.TernimalID)
            vehicle_checker_new = Vehicle_checker(data.VIN, 
                                                    routeId,
                                                    self.log)
            self.vehicle_checker_list.append(vehicle_checker_new)  
            self.create_log_file(data.VIN)                  

    def route_task_cancel_callback(self,data):
        release_message = data.VIN + " to be removed"
        self.log.write_diagnose_log("COMMON","", release_message,"","")
        if len(self.vehicle_checker_list) > 0:
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.VIN == data.VIN:                   
                    self.RemoveVehicle(data.VIN) 
                    # self.log.write_diagnose_log("COMMON","", "removed " + vehicle_checker.VIN,"","")   
                    break


    def hvpserver_mec_checker(self):
        pass
    
   # def rsu_state_callback(self, data):
   #     self.dic_rsu_heart[str(data.header.frame_id)] = data.alive

    def fusion_out_callback_route1(self,data):
        pedestrian_list_tmp = [] 
        type = "type(0: unknown 1: pedestrian 2: bicycle 3: car 4: truck)"
        channelTempDic = {"id":[], "x":[], "y":[], "index":[], "region":[]} 
        length = len(data.channels[0].values)
        for channel in data.channels:
            if channel.name == type:
                for i in range(0,length):
                    if channel.values[i] == 1.0:
                        channelTempDic["index"].append(i)
            elif channel.name == "x":
                for i in range(0,length):
                    channelTempDic["x"].append(channel.values[i])
            elif channel.name == "y":
                for i in range(0,length):
                    channelTempDic["y"].append(channel.values[i])
            elif channel.name == "id":
                for i in range(0,length):
                    channelTempDic["id"].append(channel.values[i])
            elif channel.name == "region":
                for i in range(0,length):
                    channelTempDic["region"].append(channel.values[i])
        if len(channelTempDic["index"]) > 0:
            for pedestrian_index in channelTempDic["index"]:
                pedestrian_tmp = []
                ignore_flag = False
                if len(pedestrian_ignore_list) > 0:
                    for pedestrian_ignore in pedestrian_ignore_list:
                        if (channelTempDic["x"][pedestrian_index] > pedestrian_ignore[0]-0.5) and (channelTempDic["x"][pedestrian_index] < pedestrian_ignore[0] + 0.5) and \
                            (channelTempDic["y"][pedestrian_index] > pedestrian_ignore[1]-0.5) and (channelTempDic["y"][pedestrian_index] < pedestrian_ignore[1] + 0.5): 
                            ignore_flag = True
                            break
                if ignore_flag == False:
                    pedestrian_tmp.append(channelTempDic["id"][pedestrian_index])
                    pedestrian_tmp.append(channelTempDic["x"][pedestrian_index])
                    pedestrian_tmp.append(channelTempDic["y"][pedestrian_index])
                    pedestrian_list_tmp.append(pedestrian_tmp)

        self.pedestrian_list_route1 = pedestrian_list_tmp
        if len(self.vehicle_checker_list) > 0:
            for channel in data.channels:
                for vehicle_checker in self.vehicle_checker_list:
                    if channel.name == vehicle_checker.VIN:
                        vehicle_checker.area_located = int(channelTempDic["region"][int(channel.values[0])])
                        break

    def fusion_out_callback_route2(self,data):
        pedestrian_list_tmp = []
        type = "type(0: unknown 1: pedestrian 2: bicycle 3: car 4: truck)"
        channelTempDic = {"id":[], "x":[], "y":[], "index":[], "region":[]} 
        length = len(data.channels[0].values)
        for channel in data.channels:
            if channel.name == type:
                for i in range(0,length):
                    if channel.values[i] == 1.0:
                        channelTempDic["index"].append(i)
            elif channel.name == "x":
                for i in range(0,length):
                    channelTempDic["x"].append(channel.values[i])
            elif channel.name == "y":
                for i in range(0,length):
                    channelTempDic["y"].append(channel.values[i])
            elif channel.name == "id":
                for i in range(0,length):
                    channelTempDic["id"].append(channel.values[i])
            elif channel.name == "region":
                for i in range(0,length):
                    channelTempDic["region"].append(channel.values[i])
        if len(channelTempDic["index"]) > 0:
            for pedestrian_index in channelTempDic["index"]:
                pedestrian_tmp = []
                ignore_flag = False
                if len(pedestrian_ignore_list) > 0:
                    for pedestrian_ignore in pedestrian_ignore_list:
                        if (channelTempDic["x"][pedestrian_index] > pedestrian_ignore[0]-0.5) and (channelTempDic["x"][pedestrian_index] < pedestrian_ignore[0] + 0.5) and \
                            (channelTempDic["y"][pedestrian_index] > pedestrian_ignore[1]-0.5) and (channelTempDic["y"][pedestrian_index] < pedestrian_ignore[1] + 0.5): 
                            ignore_flag = True
                            break
                if ignore_flag == False:
                    pedestrian_tmp.append(channelTempDic["id"][pedestrian_index])
                    pedestrian_tmp.append(channelTempDic["x"][pedestrian_index])
                    pedestrian_tmp.append(channelTempDic["y"][pedestrian_index])
                    pedestrian_list_tmp.append(pedestrian_tmp)

        self.pedestrian_list_route2 = pedestrian_list_tmp
        if len(self.vehicle_checker_list) > 0:
            for channel in data.channels:
                for vehicle_checker in self.vehicle_checker_list:
                    if channel.name == vehicle_checker.VIN:
                        vehicle_checker.area_located = int(channelTempDic["region"][int(channel.values[0])])
                        break

    def fusion_car_state_callback(self,data):
        if len(self.vehicle_checker_list) > 0:
            for fusion_car_state in data.states:       
                for vehicle_checker in self.vehicle_checker_list:
                    if vehicle_checker.VIN == fusion_car_state.vin:
                        if fusion_car_state.car_state == 1:
                            self.publish_diagnose_VIN(vehicle_checker, "000001", " FUSION_TRY_TO_REMATCH", "", "diagnose")
                            print_msg (1, fusion_car_state.vin + " FUSION_TRY_TO_REMATCH")
                        elif fusion_car_state.car_state == 2:
                            self.publish_diagnose_VIN(vehicle_checker, "000001", " FUSION_LOST", "", "diagnose")
                            print_msg (1, fusion_car_state.vin + " FUSION_LOST")
                        elif fusion_car_state.car_state == 3:
                            self.publish_diagnose_VIN(vehicle_checker, "000001", " FUSION_ADD_FAILED",  "", "diagnose")
                            print_msg (1,fusion_car_state.vin + " FUSION_ADD_FAILED")
                        else:
                            pass
                        break

    def vehicle_state_callback(self,data):
        # self.log.write_diagnose_log(data.header.frame_id, \
        #                             "vehicle state time: " + str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs),\
        #                             " ",\
        #                             " ", " ")
        routeId = ""
        if data == None: 
            self.publish_diagnose_system("000001", "Tbox Not Work!")
        else:
            if len(self.vehicle_checker_list) > 0:
                for vehicle_checker in self.vehicle_checker_list:
                    if vehicle_checker.VIN == data.header.frame_id:                        
                        if data.ReachEndofPath == 1 :
                            # self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)
                            self.RemoveVehicle(vehicle_checker.VIN)
                            print_msg (0, "Vehicle " + vehicle_checker.VIN + " is removed by task completed")
                            return
                        if data.Active == 1 or data.Active == 0:
                            vehicle_checker.vehicleSeqCounter = data.header.seq
                            routeId = vehicle_checker.routeId
                        elif data.Active == 2 or data.Active == -1:
                            # self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)
                            self.RemoveVehicle(vehicle_checker.VIN)
                            print_msg (2, "Vehicle " + vehicle_checker.VIN + " is removed by task cancelled")                                
                        if data.CancelReason != 0:            
                            for vehicle_diagnose in self.vehicle_diagnose_list:
                                if vehicle_diagnose.number == data.CancelReason:
                                    if vehicle_diagnose.type == "fault":
                                        str1 = routeId + " " + data.header.frame_id + " " + vehicle_diagnose.discription
                                        self.publish_diagnose_VIN(vehicle_checker, "000001",vehicle_diagnose.discription, "", "diagnose")
                                        print_msg(1, str1)
                                    else:
                                        self.publish_diagnose_VIN(vehicle_checker, "000002",vehicle_diagnose.discription, "", "diagnose")
                                        print_msg(2, routeId + " " + data.header.frame_id + " " + vehicle_diagnose.discription)
                                    break
                        else:
                            diagnoseInfo = Diagnose()
                            diagnoseInfo.errorCode = "000000"
                            diagnoseInfo.errorDescription = "Normal"
                            self.publish_diagnose_reducer(diagnoseInfo)
                        break
    
    def RemoveVehicle(self, VIN):   
        if len(self.vehicle_checker_list) > 0:
            vehicle_list = []
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.side_by_side_stop_flag == True and vehicle_checker.side_by_side_VIN == VIN:
                    vehicle_checker.side_by_side_VIN = ""
                    vehicle_checker.vehicle_avoiding_status = "VEHICLE_CLEAR_CHECKING"
                    vehicle_checker.side_by_side_stop_flag = False
                    break

            for vehicle in self.vehicle_checker_list:
                if vehicle.VIN == VIN:
                    if vehicle.remote_stop_flag == False:
                        vehicle.remote_stop_flag = True
                    self.log.write_diagnose_log("COMMON","", "removed " + VIN,"","") 
                    print_msg(2, "removed " + vehicle_checker.VIN)   
                else:
                    vehicle_list.append(vehicle)
            self.vehicle_checker_list = vehicle_list

    
    def vehicle_pose_receiver(self, data):
        if len(self.vehicle_checker_list) >0:
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.VIN == data.header.frame_id:
                    vehicle_checker.vehiclePoseCounter = data.header.seq
                    # if vehicle_checker.vehiclePoseCounter >= 10000:
                    #     vehicle_checker.vehiclePoseCounter = 0
                    break

    def vehicle_pose_tbox_callback(self,data):
        
        self.vehicle_pose_receiver(data)
        
        self.vehicle_start_stop_checker(data)
        
        # if self.is_pose_error == False:
        #     self.trajectory_checker(data)
            

    def vehicle_perception_compare(self,data):
        compare_list = []
        if len(self.dic_lidar_topic) >0:
            for topic_name, topicInfo in self.dic_lidar_topic.items():
                if len(topicInfo.perception_msg_vehicle) >0:
                    for pose_perception in topicInfo.perception_msg_vehicle:
                        abs_x = abs(pose_perception[0] - data.VehiclePoseX)
                        abs_y = abs(pose_perception[1] - data.VehiclePoseY)
                        if abs_x < 2 and abs_y < 2 :
                            distance = math.sqrt((abs_x**2) + (abs_y**2))
                            compare_list.append(distance)
            if len(compare_list) >0:
                #print_msg (2, "Minimun distance: " + str(min(compare_list)))
                if min(compare_list) > max_no_pose_error_dis:
                    for vehicle_checker in self.vehicle_checker_list:
                        if vehicle_checker.VIN == data.header.frame_id:
                            self.publish_diagnose_VIN(vehicle_checker, "000001", " location error " + str(min(compare_list)), "", "diagnose")
                            break
       
    def trajectory_checker(self, pose_vehicle):
        if (pose_vehicle.LocalizedPoseX == 0.0 and pose_vehicle.LocalizedPoseY == 0.0) :
            #print_msg(2, str(pose_vehicle.LocalizedPoseX) + "      " + str(pose_vehicle.LocalizedPoseY))
            return

        trajectory_points = []
        compare_list = []
        path_min_point = []
        min_before = 10000
        min_after = 10000
        heading_is_abnormal = False
        vehicle_checker = None

        if len(self.vehicle_checker_list)>0:
            for _vehicle_checker in self.vehicle_checker_list:
                if _vehicle_checker.VIN == pose_vehicle.header.frame_id:
                    vehicle_checker = _vehicle_checker
                    break
        else:
            return

        if vehicle_checker is None:
            return
        if vehicle_checker.main_path_is_complete == True:
            return
        trajectory_points = self.path_trajectory_points_route1 if vehicle_checker.routeId == "line1" else self.path_trajectory_points_route2
        # vehicleCenterX = self.GetCenterfromRear(pose_vehicle.LocalizedPoseX,pose_vehicle.LocalizedPoseY,1.55,pose_vehicle.LocalizedPoseH)[0]
        # vehicleCenterY = self.GetCenterfromRear(pose_vehicle.LocalizedPoseX,pose_vehicle.LocalizedPoseY,1.55,pose_vehicle.LocalizedPoseH)[1]
        
        if len(trajectory_points) > 0:
            for point in trajectory_points:
                # trajectory_point_x = self.GetCenterfromRear(float(point[0]),float(point[1]),1.55,float(point[2]))[0]
                # trajectory_point_y = self.GetCenterfromRear(float(point[0]),float(point[1]),1.55,float(point[2]))[1]
                trajectory_point_x = float(point[0])
                trajectory_point_y = float(point[1])
                if len(compare_list)>0:
                    min_before = min(compare_list)
                dis_x = abs(trajectory_point_x - pose_vehicle.LocalizedPoseX)
                dis_y = abs(trajectory_point_y - pose_vehicle.LocalizedPoseY)
                distance = math.sqrt((dis_x**2) + (dis_y**2))
                compare_list.append(distance)
                if len(compare_list)>0:
                    min_after = min(compare_list)
                if min_after < min_before:
                    path_min_point = point

            if len(compare_list) > 0 and len(path_min_point) > 0:
                #print_msg (2, "Minimun trajectory distance: " + str(min(compare_list)))
                if min(compare_list) > max_trajectory_error:
                    for i in range(10): 
                        vehicle_checker.CancelTask(self.pub_cancel_task)
                    for j in range(10):
                        self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)

                    self.publish_diagnose_VIN(vehicle_checker, "000001", "Trajectory Deviates error and Task Cancelled", "" , "diagnose")
                    self.log.write_diagnose_log(vehicle_checker.VIN, "vehicle: ", str(pose_vehicle.LocalizedPoseX) + " " + str(pose_vehicle.LocalizedPoseY) + " " + str(pose_vehicle.LocalizedPoseH), "   \
                                                path: ", str(path_min_point[0]) + " " + str(path_min_point[1]) )
                    self.log.write_diagnose_log(vehicle_checker.VIN, "remove", "" , pose_vehicle.header.frame_id, "")
                    str1 = vehicle_checker.routeId + " " + pose_vehicle.header.frame_id + " " + "Pose trajectory error and Task Cancelled"
                    # print_msg(1, str1 )
                    return
          
    def vehicle_start_stop_checker(self, pose_vehicle):
        if pose_vehicle.LocalizedPoseX == 0.0 and pose_vehicle.LocalizedPoseY == 0.0:
            return

       
        compare_list = []
        perception_temp_vehicle_nearest = []
        perception_temp_vehicle_list = []
        perception_temp_pedestrian = []
        min_before = 10000
        min_after = 10000
        vehicleCenterX = self.GetCenterfromRear(pose_vehicle.LocalizedPoseX,pose_vehicle.LocalizedPoseY,1.55,pose_vehicle.LocalizedPoseH)[0]
        vehicleCenterY = self.GetCenterfromRear(pose_vehicle.LocalizedPoseX,pose_vehicle.LocalizedPoseY,1.55,pose_vehicle.LocalizedPoseH)[1]
        vehicle_checker = None
        if len(self.vehicle_checker_list)>0:
            for _vehicle_checker in self.vehicle_checker_list:
                if _vehicle_checker.VIN == pose_vehicle.header.frame_id:
                    vehicle_checker = _vehicle_checker
                    break
        else:
            return

        if vehicle_checker is None:
            return
        
        if len(self.dic_lidar_topic) == 0:
            return
        if vehicle_checker.cancel_task_flag == True:
            return 

        # print_msg(2, str(vehicle_checker.remote_stop_flag))
        for topic_name, topicInfo in self.dic_lidar_topic.items():
            if "parkzone" in str(topicInfo._topic_name) and vehicle_checker.routeId != "line2":             
                continue
            if "parkzone" not in str(topicInfo._topic_name) and vehicle_checker.routeId != "line1":
                continue
            if vehicle_checker.routeId == "line2" and (vehicle_checker.area_located + 1 < topicInfo.area or vehicle_checker.area_located - 1 > topicInfo.area):
                continue

            if len(topicInfo.perception_msg_vehicle) >0:
                for pose_perception_vehicle in topicInfo.perception_msg_vehicle:
                    if len(compare_list)>0:
                        min_before = min(compare_list)
                    abs_x = abs(pose_perception_vehicle[0] - vehicleCenterX)
                    abs_y = abs(pose_perception_vehicle[1] - vehicleCenterY)
                    if abs_x < 15 and abs_y < 15 :
                    # if abs_x < 4 and abs_y < 4 :
                        distance = math.sqrt((abs_x**2) + (abs_y**2))
                        compare_list.append(distance)
                        pose_temp = []
                        pose_temp = deepcopy(pose_perception_vehicle)
                        perception_temp_vehicle_list.append(pose_temp)

                    if len(compare_list)>0:
                        min_after = min(compare_list)
                    if min_after < min_before:
                        perception_temp_vehicle_nearest = pose_perception_vehicle
                    # pose_temp = []
                    # pose_temp = deepcopy(pose_perception_vehicle)
                    # perception_temp_vehicle_list.append(pose_temp)

            if len(topicInfo.perception_msg_pedestrian) >0:
                for peception_pedestrian in topicInfo.perception_msg_pedestrian:
                    abs_xx = abs(vehicleCenterX - peception_pedestrian[1])
                    abs_yy = abs(vehicleCenterY - peception_pedestrian[2])
                    distance_ = math.sqrt((abs_xx**2) + (abs_yy**2))
                    if distance_ < 12:
                        perception_temp_pedestrian.append(peception_pedestrian)

        # if len(perception_temp_vehicle_nearest) >0:
        if abs(vehicle_checker.vehicleSpeed) > 0 or vehicle_checker.manual_flag == 0:
            vehicle_checker.pose_center_x = vehicleCenterX
            vehicle_checker.pose_center_y = vehicleCenterY
            vehicle_checker.pose_center_h = pose_vehicle.LocalizedPoseH

        # is_ignore_pose_error = vehicle_checker.is_ignore_pose_error_checker()
        is_ignore_pose_error = False

        if len(compare_list) == 0 :
            vehicle_checker.no_perception_detected_counter = vehicle_checker.no_perception_detected_counter + 1
            if vehicle_checker.no_perception_detected_counter > max_no_perception_detected_counter: 
                self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle " + pose_vehicle.header.frame_id + " vehicle pose exception for no perception" , "" , "diagnose") 
                if is_ignore_pose_error == False:
                    self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle " + pose_vehicle.header.frame_id + " vehicle pose exception to cancel: No perception detected  ", "" , "diagnose")               
                    for i in range(10): 
                        vehicle_checker.CancelTask(self.pub_cancel_task)
                    for j in range(10):
                        self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)
                    self.publish_diagnose_VIN(vehicle_checker, "000001", "No perception detected Pose Error and Task Cancelled", "" , "diagnose")
                    self.log.write_diagnose_log(vehicle_checker.VIN, "vehicle", str(vehicleCenterX) + " " + str(vehicleCenterY) , "perception", " " + " " + " ")                             
                    str1 = vehicle_checker.routeId + " " + pose_vehicle.header.frame_id + " " + "No perception detected Pose Error and Task Cancelled"
                    print_msg(1, str1)
                    vehicle_checker.no_perception_detected_counter = 0
                return
        else:
            vehicle_checker.no_perception_detected_counter = 0
            # print_msg (2, "Minimun distance: " + str(min(compare_list)))
            self.log.write_diagnose_log(pose_vehicle.header.frame_id , \
                                        "Minimun distance: "+ str(min(compare_list)), \
                                        "vehicletimestamp: "+ str(pose_vehicle.header.stamp.secs) +"." + str(pose_vehicle.header.stamp.nsecs) + "     " +" vehiclepose: " + str(pose_vehicle.LocalizedPoseX) + "   " + str(pose_vehicle.LocalizedPoseY) + "   " + str(pose_vehicle.LocalizedPoseH),\
                                            " center: " + str(vehicleCenterX) + " " + str(vehicleCenterY) , " pertimestamp: "+ str(perception_temp_vehicle_nearest[2]) + "    " + str(perception_temp_vehicle_nearest[0]) + " " + str(perception_temp_vehicle_nearest[1]))
            if min(compare_list) > max_no_pose_error_dis :             
                if abs(perception_temp_vehicle_nearest[2] - pose_vehicle.header.stamp.secs) < 3 :
                    vehicle_checker.pose_error_check_counter = vehicle_checker.pose_error_check_counter + 1 
                    if vehicle_checker.pose_error_check_counter > 5:
                        self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle " + pose_vehicle.header.frame_id + " vehicle pose exception " + str(min(compare_list)), "" , "diagnose") 
                        if is_ignore_pose_error == False: 
                            vehicle_checker.pose_error_check_counter = 0             
                            self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle " + pose_vehicle.header.frame_id + " vehicle pose exception for error " + str(min(compare_list)), "" , "diagnose")               
                            for i in range(10): 
                                vehicle_checker.CancelTask(self.pub_cancel_task)
                            for j in range(10):
                                self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)
                                
                            self.publish_diagnose_VIN(vehicle_checker, "000001", "Pose Exception for Error and Task Cancelled", "" , "diagnose")
                            self.log.write_diagnose_log(vehicle_checker.VIN, "vehicle", str(vehicleCenterX) + " " + str(vehicleCenterY) , "perception", str(perception_temp_vehicle_nearest[0]) + " " + str(perception_temp_vehicle_nearest[1]))                             
                            str1 = vehicle_checker.routeId + " " + pose_vehicle.header.frame_id + " " + "Pose Exception for Error and Task Cancelled"
                            print_msg(1, str1)                   
                        return                   
                else:
                    if is_ignore_pose_error == False:
                        self.log.write_diagnose_log(pose_vehicle.header.frame_id  + "vehicle timestamp and perception timestamp alignment error", \
                                            "vehicle timestamp: "+ str(pose_vehicle.header.stamp.secs), \
                                            "",\
                                            "perception timestamp: "+ str(perception_temp_vehicle_nearest[2]), "") 
            else:
                vehicle_checker.pose_error_check_counter = 0 
            if is_ignore_pose_error == False: 
                if vehicle_checker.last_frame_perception_timestamp == -1:
                    vehicle_checker.last_frame_perception_timestamp = perception_temp_vehicle_nearest[2]
                elif vehicle_checker.last_frame_perception_timestamp != -1 and \
                    vehicle_checker.last_frame_perception_timestamp - perception_temp_vehicle_nearest[2] >= 1:
                    self.log.write_diagnose_log(pose_vehicle.header.frame_id , \
                                            "perception last frame timestamp: "+ str(vehicle_checker.last_frame_perception_timestamp), \
                                            "",\
                                            "perception this frame timestamp: "+ str(perception_temp_vehicle_nearest[2])  , "")
                    print_msg (1, "percetpion timestamp error") 
                    vehicle_checker.last_frame_perception_timestamp = perception_temp_vehicle_nearest[2]
                    return
                elif vehicle_checker.last_frame_perception_timestamp != -1 and \
                    vehicle_checker.last_frame_perception_timestamp < perception_temp_vehicle_nearest[2]:
                    vehicle_checker.last_frame_perception_timestamp = perception_temp_vehicle_nearest[2] 

        self.trajectory_checker(pose_vehicle) 
        if len(perception_temp_vehicle_nearest) >0:         
            perception_filtered = vehicle_checker.pedestrian_region_filter(perception_temp_pedestrian)
            vehicle_checker.perception_vehicle_list = []
            
            if len(perception_temp_vehicle_list)>0:
                for perception_temp_vehicle in perception_temp_vehicle_list:
                    absx = abs(perception_temp_vehicle[0] - perception_temp_vehicle_nearest[0])
                    absy = abs(perception_temp_vehicle[1] - perception_temp_vehicle_nearest[1])
                    if math.sqrt((absx**2) + (absy**2)) > 2:
                        vehicle_checker.perception_vehicle_list.append(perception_temp_vehicle)
            
            vehicle_detection_result = vehicle_checker.vehicle_detection()
            if vehicle_detection_result > 0:                            
                vehicle_checker.is_vehicle_avoiding = True
                print_msg(2, vehicle_checker.VIN + " remote stop by vehicle avoiding" )
                if abs(vehicle_checker.vehicleSpeed) > 0:
                    
                    vehicle_checker.RemoteStop(self.pub_remote_stop) 
                    vehicle_checker.RemoteStop(self.pub_remote_stop)
                    vehicle_checker.RemoteStop(self.pub_remote_stop)         
                    vehicle_checker.RemoteStop(self.pub_remote_stop)
                vehicle_checker.vehicle_avoiding_status = "VEHICLE_AVOIDING"
                
                if vehicle_detection_result == 1:
                    self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle Avioding and stopped", "", "dp")
                elif vehicle_detection_result == 2:
                    self.publish_diagnose_VIN(vehicle_checker, "000001", "Target slot is not avalible", "", "dp")                                 
                else:
                    self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle parking exception in Parking space number " + str(vehicle_detection_result) , "", "dp")
            else:                                       
                if vehicle_checker.is_vehicle_avoiding == True:                           
                    # self.publish_diagnose_VIN(vehicle_checker, "000002", "Vehicle Clear", "", "dp")
                    vehicle_checker.is_vehicle_avoiding = False
                    vehicle_checker.vehicle_avoiding_status = "VEHICLE_CLEAR_CHECKING"
            
            self.vehicle_side_by_side_checker(vehicle_checker)
            if vehicle_checker.side_by_side_stop_flag == True:
                self.publish_diagnose_VIN(vehicle_checker, "000001", "Vehicle Avioding by too close", "", "dp")
                vehicle_checker.vehicle_avoiding_status = "VEHICLE_AVOIDING"
                if abs(vehicle_checker.vehicleSpeed) > 0:
                    print_msg(2, vehicle_checker.VIN + " remote stop by vehicle 2 vehicle too close" )
                    vehicle_checker.RemoteStop(self.pub_remote_stop) 
                    vehicle_checker.RemoteStop(self.pub_remote_stop)
                    vehicle_checker.RemoteStop(self.pub_remote_stop)         
                    vehicle_checker.RemoteStop(self.pub_remote_stop)

            
            if vehicle_checker.tracking_distance_controller() == True:
                # vehicle_checker.RemoteStart(self.pub_remote_start)
                vehicle_checker.reset_pedestrian_detect_status()
                print_msg(1, vehicle_checker.VIN +" Vehicle Pose received Timeout")
                self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Vehicle Pose received Timeout", "", "dp")
                return
                                                
            vehicle_checker.area_checker(perception_filtered)
            if vehicle_checker.in_area_counter > 0 and vehicle_checker.in_area_counter < max_pedestrian_check_count and vehicle_checker.clear_in_area_flag == False:
                if abs(vehicle_checker.vehicleSpeed) > 0:
                    print_msg(2, vehicle_checker.VIN + " remote stop by pedestrian detected" )
                    vehicle_checker.RemoteStop(self.pub_remote_stop) 
                    vehicle_checker.RemoteStop(self.pub_remote_stop)
                    vehicle_checker.RemoteStop(self.pub_remote_stop)         
                    vehicle_checker.RemoteStop(self.pub_remote_stop)
                vehicle_checker.diff_checker(perception_filtered)                                        
                self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrain Update", vehicle_checker.pedestrianDetectorList, "dp")
                vehicle_checker.pedestrian_check_state = "DETECTED"
                            
                vehicle_checker.normal_checker_counter = 0
                vehicle_checker.stop_timeout_counter = 0
                vehicle_checker.clear_check_conter = 0
                self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrain Detected and Remote Stopped" + "  " + str(vehicle_checker.in_out_diff), "----------", "dp")
            else:                                       
                if vehicle_checker.pedestrian_check_state != "NORMAL" or vehicle_checker.in_area_counter >= max_pedestrian_check_count or vehicle_checker.clear_in_area_flag == True:
                    if vehicle_checker.in_area_counter == 0:                                               
                        if vehicle_checker.clear_check_conter >= max_pedestrian_lose_frame_count :
                            vehicle_checker.clear_check_conter = max_pedestrian_lose_frame_count
                            vehicle_checker.clear_in_area_flag = True 
                        else:   
                            vehicle_checker.clear_check_conter = vehicle_checker.clear_check_conter + 1
                            for pedestrianDetecor in vehicle_checker.pedestrianInList:
                                # vehicle_checker.write_vehicle_dp_log("No Detection Checking", pedestrianDetecor)
                                self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "No Detection Checking", pedestrianDetecor, "dp")

                        if vehicle_checker.clear_in_area_flag == True:                                             
                            if vehicle_checker.in_out_diff > 0:                                                                                                       
                                vehicle_checker.normal_checker_counter = 0                                                    
                                vehicle_checker.stop_timeout_counter = vehicle_checker.stop_timeout_counter + 1
                                if  vehicle_checker.stop_timeout_counter > max_diff_timeout_frame_count:
                                    vehicle_checker.stop_timeout_counter = 0
                                    vehicle_checker.reset_pedestrian_detect_status()
                                    if vehicle_checker.manual_flag == 0 and vehicle_checker.is_vehicle_avoiding == False:     
                                        vehicle_checker.RemoteStart(self.pub_remote_start)                                                                
                                        print_msg(1, vehicle_checker.VIN + " Pedestrian detection Timeout and Remote Started")
                                        self.publish_diagnose_VIN(vehicle_checker, "000001", "Pedestrian detection Timeout and Remote Started", "Timeout and Remote Started", "dp")
                                        return
                                else:                                                        
                                    vehicle_checker.diff_checker(perception_filtered)
                                    self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrain Abnormal Check Update " + str(vehicle_checker.in_out_diff) + "   " + str(vehicle_checker.in_area_counter),\
                                                                vehicle_checker.pedestrianDetectorList, "dp")
                            else:                                                                    
                                vehicle_checker.normal_checker_counter = vehicle_checker.normal_checker_counter + 1

                    elif vehicle_checker.in_area_counter > 0 and vehicle_checker.in_area_counter < max_pedestrian_check_count:
                        if vehicle_checker.routeId == "line2" and vehicle_checker.pose_center_x > diff_check_route2_x_limit:
                            vehicle_checker.clear_check_conter = 0
                            vehicle_checker.clear_in_area_flag = False
                            vehicle_checker.normal_checker_counter = 0 
                            self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrain Recheck Update " + str(vehicle_checker.in_area_counter) ,vehicle_checker.pedestrianDetectorList, "dp")
                            return
                        if vehicle_checker.clear_in_area_flag == True:
                            vehicle_checker.stop_timeout_counter = 0
                            vehicle_checker.diff_checker(perception_filtered)
                            self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrain Normal Check Update " + str(vehicle_checker.in_out_diff)+ "   " + str(vehicle_checker.in_area_counter) ,vehicle_checker.pedestrianDetectorList, "dp")


                    elif vehicle_checker.in_area_counter >= max_pedestrian_check_count:
                        vehicle_checker.normal_checker_counter = 0
                        vehicle_checker.clear_check_conter = 0
                        vehicle_checker.stop_timeout_counter = 0
                        vehicle_checker.dif_init_flag = True
                        self.clear_in_area_flag = False
                        #vehicle_checker.exceed_number_check_counter = vehicle_checker.exceed_number_check_counter + 1
                        if abs(vehicle_checker.vehicleSpeed) > 0:
                            print_msg(1, vehicle_checker.VIN + " remote stop vehicle by exceed max pedestrian count" )
                            vehicle_checker.RemoteStop(self.pub_remote_stop)
                            vehicle_checker.RemoteStop(self.pub_remote_stop)
                            vehicle_checker.RemoteStop(self.pub_remote_stop)
                            vehicle_checker.RemoteStop(self.pub_remote_stop)
                        self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrian number exceed max pedestrian count" ,str(vehicle_checker.in_area_counter), "dp")

                                                                                    
                    if vehicle_checker.normal_checker_counter > 5:
                        vehicle_checker.normal_checker_counter = 0
                        vehicle_checker.reset_pedestrian_detect_status()                                                
                        if vehicle_checker.manual_flag == 0 and vehicle_checker.is_vehicle_avoiding == False:                                                    
                            vehicle_checker.RemoteStart(self.pub_remote_start)                                                       
                            print_msg(0,  vehicle_checker.VIN + " Pedestrian Clear and Remote Started")
                            self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Pedestrian Clear and Remote Started" ,"----------", "dp")                                                
                    else:
                        vehicle_checker.pedestrian_check_state = "NORMALCHECKING"
                elif vehicle_checker.pedestrian_check_state == "NORMAL" :
                    if vehicle_checker.vehicle_avoiding_status == "VEHICLE_CLEAR_CHECKING":
                        vehicle_checker.normal_checker_counter = vehicle_checker.normal_checker_counter + 1
                        if vehicle_checker.normal_checker_counter > 5:
                            if vehicle_checker.manual_flag == 0 and vehicle_checker.is_vehicle_avoiding == False and vehicle_checker.side_by_side_stop_flag == False:
                                vehicle_checker.RemoteStart(self.pub_remote_start)
                                vehicle_checker.reset_pedestrian_detect_status()
                                print_msg(0, vehicle_checker.VIN + " Vehicle Avioding Clear and Remote Started")
                                self.publish_diagnose_VIN(vehicle_checker, dp_pub_level, "Vehicle Avioding Clear and Remote Started" ,"----------", "dp")
                                vehicle_checker.vehicle_avoiding_status = "NORMAL"
                            else:
                                vehicle_checker.normal_checker_counter = 0
                    else:
                        vehicle_checker.normal_checker_counter = 0                                                                                 
                        # break                                                                
            
    def GetCenterfromRear(self,x,y,r,theta):
        x1 = x + r * math.cos(theta)
        y1 = y + r * math.sin(theta)
        return [x1,y1]

    def vehicle_pose_received_checker(self):
        while True:
            if len(self.vehicle_checker_list) > 0:
                for vehicle_checker in self.vehicle_checker_list:
                    is_exist = False
                    vin = vehicle_checker.VIN
                    counter1 = vehicle_checker.vehiclePoseCounter
                    time.sleep(vehicle_offline_check_time)
                    for vehicle1 in self.vehicle_checker_list:
                        if vin == vehicle1.VIN:
                            is_exist = True
                            break
                    if is_exist == False:
                        break
                    counter2 = vehicle_checker.vehiclePoseCounter
                    if  counter1 == counter2:
                        self.publish_diagnose_VIN(vehicle_checker, "000001", " Pose Not Received!", "", "diagnose" )
                        print_msg (1, vehicle_checker.routeId + " " + vehicle_checker.VIN + " Pose Not Received!")

    #######################掉线检测#########################
    def vehicle_offline_checker(self):
        while True:
            if len(self.vehicle_checker_list) > 0:
                for vehicle_checker in self.vehicle_checker_list:
                    is_exist = False
                    vin = vehicle_checker.VIN
                    counter1 = vehicle_checker.vehicleSeqCounter
                    time.sleep(vehicle_offline_check_time)
                    for vehicle1 in self.vehicle_checker_list:
                        if vin == vehicle1.VIN:
                            is_exist = True
                            break
                    if is_exist == False:
                        break                 
                    counter2 = vehicle_checker.vehicleSeqCounter
                    if counter1 == counter2:
                        self.publish_diagnose_VIN(vehicle_checker, "000001", " is Offline!", "", "diagnose")
                        print_msg (1, vehicle_checker.routeId + " " + vehicle_checker.VIN + " is offline!")
                        vehicle_checker.offline_counter = vehicle_checker.offline_counter +1
                        if vehicle_checker.offline_counter > vehicle_checker.offline_timout_check_counter:
                            print_msg (1, vehicle_checker.routeId + " " + vehicle_checker.VIN + " is offline timeout and task cancelled ")
                            self.publish_diagnose_VIN(vehicle_checker, "000001", "",vehicle_checker.routeId + " " + vehicle_checker.VIN + " is offline timeout and task cancelled ","diagnose")
                            for i in range(10): 
                                vehicle_checker.CancelTask(self.pub_cancel_task)
                            for j in range(10):
                                self.remove_vehicle_from_server(vehicle_checker.VIN, vehicle_checker.routeId)                       
                    else:
                        if vehicle_checker.offline_counter > 0:
                            self.publish_diagnose_VIN(vehicle_checker, "000000", " is reconnected!", "", "diagnose")
                            print_msg (0, vehicle_checker.routeId + " " + vehicle_checker.VIN + " is reconnected!") 
                        vehicle_checker.offline_counter = 0
    ########################################################## 

    def rsu_alive_checker(self):
        rsu_alive_tmp = {}
        while(True):
            if len(self.dic_rsu_heart):
                rsu_alive_tmp = copy.deepcopy(self.dic_rsu_heart)
                time.sleep(5)
                if len(self.dic_rsu_heart):
                    for rsu_no in rsu_alive_tmp.keys():
                        if self.dic_rsu_heart.__contains__(rsu_no):
                            if rsu_alive_tmp[rsu_no] == self.dic_rsu_heart[rsu_no]:
                                self.publish_diagnose_system("000001", "%s is offline!" % rsu_no)
                                print ("%s is offline!" % rsu_no)
                        else:
                            self.publish_diagnose_system("000001", "%s is offline!" % rsu_no)
                            print ("%s is offline!" % rsu_no)

    def publish_diagnose_system(self, errorCode, errorDescription):
        diagnoseInfo = Diagnose()
        diagnoseInfo.errorDescription = str(errorDescription)
        diagnoseInfo.errorCode = str(errorCode)
        if errorDescription != "Normal":
            self.log.write_diagnose_log("COMMON", diagnoseInfo.errorCode, "", diagnoseInfo.errorDescription, "")                    
            # self.pub_diagnose.publish(diagnoseInfo)
            self.publish_diagnose_reducer(diagnoseInfo) 


    def publish_diagnose_VIN(self, vehicleChecker, errorCode, errorDescription, info, logType):
        diagnoseInfo = Diagnose()
        diagnoseInfo.errorDescription = str(vehicleChecker.routeId) + " " + str(vehicleChecker.VIN) + " " + str(errorDescription)
        diagnoseInfo.errorCode = str(errorCode)
        if isinstance(vehicleChecker, Vehicle_checker):
            # self.pub_diagnose.publish(diagnoseInfo) 
            self.publish_diagnose_reducer(diagnoseInfo)         
            if logType == "dp":
                vehicleChecker.write_vehicle_dp_log(str(errorDescription), info)
            elif logType == "diagnose":
                vehicleChecker.write_vehicle_diagnose_log(str(errorCode), str(errorDescription))
              
    def publish_diagnose_reducer(self, diagnoseInfo):
        _diagnoseInfo = diagnoseInfo 
        _errorDescription = _diagnoseInfo.errorDescription  
        if _errorDescription == "Normal":
            self.dic_diagnose.clear()
            return      
        if len(self.dic_diagnose)>0:
            if self.dic_diagnose.__contains__(_errorDescription):
                self.dic_diagnose[_errorDescription] = 1 + self.dic_diagnose[_errorDescription]
            else:
                self.dic_diagnose[_errorDescription] = 1
        else:
            self.dic_diagnose[_errorDescription] = 1
        if self.dic_diagnose[_errorDescription] % 10 == 1:
            if self.dic_diagnose[_errorDescription] > 100:
                self.dic_diagnose[_errorDescription] = 0           
            self.pub_diagnose.publish(_diagnoseInfo)
        
        

    def ros_node_ping(self):
        isok = False
        while True:
            isok = rosnode.rosnode_ping("hmi_adapter")
            print isok


    def remove_vehicle_from_server(self,VIN,routeID):
        self.RemoveVehicle(VIN)
        vi = VehInfo()
        vi.header.frame_id = "diagnose"
        vi.posex = 0
        vi.posey = 0
        vi.poseh = 0
        vi.VIN = VIN
        if routeID == "line1":
            self.pub_remove_route1_vehicle.publish(vi)
        elif routeID == "line2":
            self.pub_remove_route2_vehicle.publish(vi)
        elif routeID == "line3":
            self.pub_remove_route3_vehicle.publish(vi)

    def pad_heart_callback1(self,data):
        if data.alive == 1:
            self.log.write_diagnose_log("COMMON","Line1 pad disconnected by pad sleep","","","")
            print_msg(1, "Line1 Pad disconnected by pad sleep")
        elif data.alive == 2:
            self.log.write_diagnose_log("COMMON","Line1 pad is in the frontpage","","","")
            print_msg(2, "Line1 Pad is in the frontpage")
        elif data.alive == 3:
            self.log.write_diagnose_log("COMMON","Line1 pad is under background","","","")
            print_msg(2, "Line1 Pad is under background")

        if self.pad_route1_heart_counter == 0:
            self.log.write_diagnose_log("COMMON","Line1 pad connected","","","")
            print_msg(2, "Line1 Pad Reconnected")
        self.pad_route1_heart_counter = data.header.seq
        # if self.pad_route1_heart_counter > 0 and self.close_route1_flag == True:
        #     self.close_route1_flag = False
        self.log.write_diagnose_log("COMMON","route1 pad counter", str(self.pad_route1_heart_counter),"","")

    def pad_heart_callback2(self,data):
        if data.alive == 1:
            self.log.write_diagnose_log("COMMON","Line2 pad disconnected by pad sleep","","","")
            print_msg(1, "Line2 Pad disconnected by pad sleep")
        elif data.alive == 2:
            self.log.write_diagnose_log("COMMON","Line2 pad is in the frontpage","","","")
            print_msg(2, "Line2 Pad is in the frontpage")
        elif data.alive == 3:
            self.log.write_diagnose_log("COMMON","Line2 pad is under background","","","")
            print_msg(2, "Line2 Pad is under background")
        if self.pad_route2_heart_counter == 0:
            self.log.write_diagnose_log("COMMON","Line2 pad connected","","","")
            print_msg( 2, "Line2 Pad Reconnected")
        self.pad_route2_heart_counter = data.header.seq
        # if self.pad_route2_heart_counter > 0 and self.close_route2_flag == True:
        #     self.close_route2_flag = False
        self.log.write_diagnose_log("COMMON","route2 pad counter", str(self.pad_route2_heart_counter),"","")
        # print_msg (0, str(self.pad_route2_heart_counter))

    def route_state_callback(self,data):
        # self.routeList = data.routeInfoVec
        # if len(self.routeList)> 0: 
        #     for route in self.routeList:
        #         if route.routeId == "line1":
        #             if route.state == "On" and self.close_route1_flag == True:
        #                 self.close_route1_flag = False
        #                 self.log.write_diagnose_log("COMMON","", "Line1 is opened by manual","","")
        #         elif route.routeId == "line2":
        #             if route.state == "On" and self.close_route2_flag == True:
        #                 self.close_route2_flag = False
        #                 self.log.write_diagnose_log("COMMON","", "Line2 is opened by manual","","")
        pass

    def maunal_stop_req(self, VIN, routeId):
        rospy.wait_for_service('vehicleOperation')
        try:
            maunal_stop_client = rospy.ServiceProxy('vehicleOperation', VehicleOperation)
            response = maunal_stop_client("", "diagnose", VIN, "RemoteStop", routeId)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def close_route_req(self, routeId):
        rospy.wait_for_service('routeOperation')
        try:
            close_route_client = rospy.ServiceProxy('routeOperation', RouteOperation)
            response = close_route_client("", "diagnose", routeId, "CloseLine")
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def stop_whole_vehicle(self, routeId):
        if len(self.vehicle_checker_list):
            for vehicle_checker in self.vehicle_checker_list:
                if vehicle_checker.routeId == routeId:
                    # vehicle_checker.RemoteStop(self.pub_remote_stop)
                    # vehicle_checker.RemoteStop(self.pub_remote_stop)
                    # vehicle_checker.RemoteStop(self.pub_remote_stop)
                    # vehicle_checker.RemoteStop(self.pub_remote_stop)
                    response = self.maunal_stop_req(vehicle_checker.VIN, routeId)
                    self.log.write_diagnose_log("COMMON","",routeId + " " + vehicle_checker.VIN + " remote stop by pad offline " + response.msg,"","")
                    time.sleep(1)

    def pad_route1_offline_checker(self):
        while True:
            counter1 = self.pad_route1_heart_counter
            time.sleep(pad_timeout)
            if len(self.routeList)> 0:
                for route in self.routeList:
                    if route.routeId == "line1" and route.state == "On" :
                        counter2 = self.pad_route1_heart_counter
                        if counter1 == counter2 and counter2 > 0:
                        # if counter1 == counter2:
                            print_msg(1, "Line1 Pad is offline")
                            self.log.write_diagnose_log("COMMON","", "Line1 Pad is offline","","")
                            # response = self.close_route_req("line1")
                            # print (response)
                            # if response.code == "000000":
                            #     self.close_route1_flag = True
                            self.stop_whole_vehicle("line1")
                            self.pad_route1_heart_counter = 0
                            # self.close_route2_flag = True

    def pad_route2_offline_checker(self):
        while True:
            counter1 = self.pad_route2_heart_counter
            time.sleep(pad_timeout)
            if len(self.routeList)> 0:
                for route in self.routeList:
                    if route.routeId == "line2" and route.state == "On":
                        counter2 = self.pad_route2_heart_counter
                        if counter1 == counter2 and counter2 > 0:
                        # if counter1 == counter2:
                            print_msg(1, "Line2 Pad is offline")
                            self.log.write_diagnose_log("COMMON","", "Line2 Pad is offline","","")
                            # response = self.close_route_req("line2")        
                            # print (response)
                            # if response.code == "000000":
                            #     self.close_route2_flag = True
                            self.stop_whole_vehicle("line2")
                            self.pad_route2_heart_counter = 0
                            # self.close_route2_flag = True

    def vehicle_distance_checker(self, vehicle_checker1, vehicle_checker2, distance_threshold):
        abs_xx = abs(vehicle_checker1.pose_center_x - vehicle_checker2.pose_center_x)
        abs_yy = abs(vehicle_checker1.pose_center_y - vehicle_checker2.pose_center_y)
        distance = math.sqrt((abs_xx**2) + (abs_yy**2))
        if distance < distance_threshold: 
            return True
        else:
            return False

    def vehicle_side_by_side_checker(self, vehicle_checker_self):
        for vehicle_checker in self.vehicle_checker_list:                
            if vehicle_checker.VIN != vehicle_checker_self.VIN and vehicle_checker_self.routeId == "line2"\
                and vehicle_checker.routeId == "line2":
                if self.vehicle_distance_checker(vehicle_checker_self, vehicle_checker, side_by_side_distance_limit) == True:
                    if vehicle_checker_self.active_time > vehicle_checker.active_time:
                        # if abs(vehicle_checker_self.vehicleSpeed) > 0:
                        #     print_msg(2, vehicle_checker_self.VIN + " remote stop by 2 vehicle too close" )
                        #     vehicle_checker_self.RemoteStop(self.pub_remote_stop) 
                        #     vehicle_checker_self.RemoteStop(self.pub_remote_stop)
                        #     vehicle_checker_self.RemoteStop(self.pub_remote_stop)         
                        #     vehicle_checker_self.RemoteStop(self.pub_remote_stop)
                        vehicle_checker_self.side_by_side_stop_flag = True
                        vehicle_checker_self.side_by_side_VIN = vehicle_checker.VIN
                        return
                else:
                    if vehicle_checker_self.side_by_side_stop_flag == True and vehicle_checker_self.side_by_side_VIN == vehicle_checker.VIN:
                        vehicle_checker_self.side_by_side_stop_flag = False
                        vehicle_checker_self.side_by_side_VIN = ""
                        vehicle_checker_self.vehicle_avoiding_status = "VEHICLE_CLEAR_CHECKING"
                        return
                           
        
if __name__ == '__main__':
    try:
        hvp_diagnose = HVP_Diagnose()

        rospy.init_node('hvp_diagnose', anonymous=False)
        
        thread_pool = []
        # thread1 = threading.Thread(target = hvp_diagnose.start_lidar_hz_listen,)
        # thread_pool.append(thread1)
        thread2 = threading.Thread(target = hvp_diagnose.vehicle_tbox_checker,)
        thread_pool.append(thread2)
        # thread3 = threading.Thread(target = hvp_diagnose.network_checker,)
        # thread_pool.append(thread3)
        thread4 = threading.Thread(target = hvp_diagnose.hvpserver_mec_checker,)
        thread_pool.append(thread4)
        thread5 = threading.Thread(target = hvp_diagnose.get_lidar_hz,)
        thread_pool.append(thread5)
        thread6 = threading.Thread(target = hvp_diagnose.vehicle_pose_received_checker,)
        thread_pool.append(thread6)
        thread7 = threading.Thread(target = hvp_diagnose.vehicle_offline_checker,)
        thread_pool.append(thread7)
        #thread8 = threading.Thread(target = hvp_diagnose.rsu_alive_checker,)
        #thread_pool.append(thread8)
        # thread9 = threading.Thread(target = hvp_diagnose.remove_vehicle_checker,)
        # thread_pool.append(thread9)
        thread10 = threading.Thread(target = hvp_diagnose.ros_node_ping,)
        thread_pool.append(thread10)
        thread11 = threading.Thread(target = hvp_diagnose.vehicle_pose_checker,)
        thread_pool.append(thread11) 
        thread12 = threading.Thread(target = hvp_diagnose.pad_route1_offline_checker,)
        thread_pool.append(thread12)
        thread13 = threading.Thread(target = hvp_diagnose.pad_route2_offline_checker,)
        thread_pool.append(thread13) 


        for thread in thread_pool:
            thread.start()
        for thread in thread_pool:
            thread.join()
    except rospy.ROSInterruptException:
        pass
