#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import sys
import roslib
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from smabo import tflib
from rokae.data_type import *
from rokae.convert_tools import *
from rokae.error import *
from rokae.xservice import *
from rokae.robot import *

Config={
  'robot_ip':'127.0.0.1',
  'joint_ids':['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'],
  'tcp0_frame_id':'tool0_controller',
}

rospy.init_node('rclient_rokae',anonymous=True)
try:
  Config.update(rospy.get_param('/config/rsocket'))
except Exception as e:
  print("get_param exception:",e.args)

pub_js=rospy.Publisher('/joint_states',JointState,queue_size=1)
pub_tf=rospy.Publisher('/update/config_tf',TransformStamped,queue_size=1)
pub_conn=rospy.Publisher('/rsocket/enable',Bool,queue_size=1)

mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False

def update(q):
  joints=JointState()
  joints.name=Config['joint_ids']
  if len(q)>=len(joints.name):
    joints.header.stamp=rospy.Time.now()
    joints.position=q
    pub_js.publish(joints)
    pub_conn.publish(mTrue)

while True:
  if rospy.is_shutdown(): break
  rospy.sleep(3)

  conf_tf=rospy.get_param('/config_tf')
  tcp0_id=Config['tcp0_frame_id']
  ec={}

  with XMateErProRobot(Config['robot_ip']) as robot:
    robot.connectToRobot(ec)
    print("rclient_rokae::connected",Config['robot_ip'])
    while True:
      try:
        joints= robot.jointPos(ec)
        posture = robot.flangePos(ec)
      except Exception as e:
        print("rclient_rokae::exception")
        break
      update(joints)
      if tcp0_id in conf_tf:
        tf=TransformStamped()
        tf.header.stamp=rospy.Time.now()
        tf.header.frame_id=conf_tf[tcp0_id]['parent_frame_id']
        tf.child_frame_id=tcp0_id
        bTp=np.eye(4)   #base To tcp
        bTp[:3,:3]=R.from_euler('xyz',posture[3:]).as_matrix()
        bTp[:3,3]=np.array(posture[:3])*1000
        fTp=np.eye(4)   #frange To tcp
#        fTp[:3,:3]=R.from_rotvec(comm.state.tcp_offset[3:]).as_matrix()
#        fTp[:3,3]=np.array(comm.state.tcp_offset[:3])*1000
        tf.transform=tflib.fromRT(bTp.dot(tflib.invRT(fTp)))
        pub_tf.publish(tf);
      if rospy.is_shutdown(): break
      rospy.sleep(0.1)
  print('rclient_rokae::connection failed')

