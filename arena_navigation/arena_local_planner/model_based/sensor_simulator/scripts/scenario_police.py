#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from ford_msgs.msg import Clusters
from geometry_msgs.msg import PoseStamped
from pedsim_msgs.msg import AgentState

# message filter
import message_filters

# 
class police():
    def __init__(self):
        self.n_col = 0
        self.n_replan_pm = 0
        self.n_replan_mb = 0
        self.collision_flag = False

        self.odom        = Odometry()
        self.odom_human  = []
        self.cluster     = Clusters()
        self.subgoal     = PoseStamped()
        self.subgoal_wgp = PoseStamped()
        self.global_path = Path()

        self.gp_received = False
        self.sg_received = False
        self.sg_wpg_received = False

        self.update_cluster = True
        self.gp_published = False
        self.num_humans=10
        self.pub_human_list=[]
        self.sub_human_list=[]
        self.human_odom=[]

 
        # sub
        self.scan = rospy.Subscriber('/scan',LaserScan, self.cbScan)

        rospy.Subscriber('/odom',Odometry, self.cb_odom)
        rospy.Subscriber('/subgoal',PoseStamped, self.cb_subgoal)
        rospy.Subscriber('/subgoal_wpg',PoseStamped, self.cb_subgoal_wpg)
        rospy.Subscriber('/vis_global_path',Path, self.cb_global_path)
        ns_prefix='eval_sim/'
        for i in range(self.num_humans):
            self.sub_human_list.append(message_filters.Subscriber(f'{ns_prefix}pedsim_agent_{i+1}/agent_state',AgentState))
        # pub
        self.pub_col = rospy.Publisher('police/collision', Int16, queue_size=10)
        self.pub_odom = rospy.Publisher('police/odom', Odometry, queue_size=10)
        for i in range(self.num_humans):
            self.pub_human_list.append(rospy.Publisher(f'police/pedsim_agent_{i+1}/agent_state', AgentState, queue_size=10))
    
        self.ts = message_filters.ApproximateTimeSynchronizer(self.sub_human_list, 10, slop=0.01) #,allow_headerless=True)
        self.ts.registerCallback(self.cb_agent_state)

        self.pub_subg = rospy.Publisher('police/subgoal', PoseStamped, queue_size=10)
        self.pub_subg_wpg = rospy.Publisher('police/subgoal_wpg', PoseStamped, queue_size=10)
        self.pub_subgp = rospy.Publisher('police/gplan', Path, queue_size=10)


        rospy.Timer(rospy.Duration(0.5),self.publish_state)


    def cb_cluster(self,msg):
        if self.update_cluster:
            self.cluster = Clusters()
            num_clusters = len(msg.mean_points)
            # print(num_clusters)
            for i in range(num_clusters):
                if num_clusters < 24:
                    self.cluster.mean_points.append(msg.mean_points[i])
                    self.cluster.velocities.append(msg.velocities[i])
                    self.cluster.labels.append(msg.labels[i])
                elif msg.labels[i] >= 24:
                    self.cluster.mean_points.append(msg.mean_points[i])
                    self.cluster.velocities.append(msg.velocities[i])
                    self.cluster.labels.append(msg.labels[i])

        #self.cluster = msg

    def cb_global_path(self, msg):
        self.global_path = msg
        self.gp_received = True

    def cb_odom(self, msg):
        self.odom = msg

    def cb_agent_state(self, *msgs):
        self.human_odom=msgs
        # if len() ==
        # if len(self.human_odom) == self.num_humans:
        #     self.receive_all_human = True

    def cb_subgoal(self, msg):
        self.subgoal = msg    
        self.sg_received = True
    
    def cb_subgoal_wpg(self, msg):
        self.subgoal_wgp = msg
        self.sg_wpg_received = True

    def get_pm_path(self,msg):
        self.n_replan_pm += 1

    def get_mb_path(self,msg):
        self.n_replan_mb += 1


    def publish_state(self, event):
        # print(self.odom)
        # self.update_cluster = False
        # self.pub_obst_odom.publish(self.cluster)
        # self.update_cluster = True
        
        self.pub_odom.publish(self.odom)
        # if self.receive_all_human:
        for i in range(self.num_humans):
            self.pub_human_list[i].publish(self.human_odom[i])
        # self.human_odom=[]

        if self.sg_received:
            self.pub_subg.publish(self.subgoal)
            self.sg_received = False

        if self.sg_wpg_received:
            self.pub_subg_wpg.publish(self.subgoal_wgp)
            self.sg_wpg_received = False

        if self.gp_received and not self.gp_published:
            self.pub_subgp.publish(self.global_path)
            self.gp_received = False
            self.gp_published = True

        # print(self.subgoal)

        # self.pub_mb_replan.publish(self.n_replan_mb)
        # self.pub_pb_replan.publish(self.n_replan_pm)

    def cbScan(self,msg):
        scan_array = np.asarray(msg.ranges)
        d_min = np.nanmin(scan_array)
        if np.isnan(d_min):
            d_min = 3.5

        if d_min > 0.5:
            self.collision_flag = False
        if d_min <= 0.35 and not self.collision_flag:
            self.collision_flag = True
            self.n_col += 1 
            self.pub_col.publish(self.n_col)
            print(self.n_col)
            



def run():
    rospy.init_node('scenario_police',anonymous=False)
    print("watching scene")
    police()
    rospy.spin()


if __name__=="__main__":
    run()

