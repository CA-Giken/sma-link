#!/usr/bin/env python3

import numpy as np
import json
import roslib
import rospy
#import pymcprotocol
import mcpdummy

from std_msgs.msg import Bool
from std_msgs.msg import String
import time
import sys

Config={
  'plc_ip':'192.168.3.39',
  'plc_port':5376,
  'inregs':'D100',
  'outregs':'D104',
}

outputRegs=4*[0]
inputRegs=4*[0]
upEdge=0
downEdge=0
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False

def queueOutput(adds,val,bit=None):
  global outputRegs
  if bit is None:
    outputRegs[adds]=val
  else:
    mask=(1 << bit)
    if val:
      outputRegs[adds] = outputRegs[adds]|mask
    else:
      outputRegs[adds] = outputRegs[adds]&(~mask)

####ROS main###########################################
rospy.init_node('mcplink',anonymous=True)
try:
  Config.update(rospy.get_param('/config/plc'))
except Exception as e:
  print("get_param exception:",e.args)

#plc = pymcprotocol.Type3E()
plc=mcpdummy.DummyPLC()

#### User define######################################
def cb_do_capt(msg):
  queueOutput(0,True,4)  #Busy bit
  queueOutput(1,0)   #Error code to 0
def cb_done_capt(msg):
  queueOutput(0,False,4)

def cb_do_solve(msg):
  queueOutput(0,True,5)  #Busy bit
  queueOutput(1,0)   #Error code to 0
def cb_done_solve(msg):
  queueOutput(0,False,5)

def cb_do_recipe(msg):
  queueOutput(0,True,6)  #Busy bit
  queueOutput(1,0)   #Error code to 0
def cb_done_recipe(msg):
  queueOutput(0,False,6)

def cb_error(msg):
  try:
    report=json.loads(msg.data)
  except Exception as e:
    print(e)
    print(msg.data)
    return
  if 'error' in report:
    err=report['error']
    if type(err) == list: err=err[0]
    queueOutput(1,int(err))   #Error code
    queueOutput(0,False,4)    #busy bit to 0
    queueOutput(0,False,5)    #busy bit to 0
    queueOutput(0,False,6)    #busy bit to 0

def checkInRegs():
  if upEdge&1:
    pub_reset.publish(mTrue)
    queueOutput(1,0)   #Error code to 0 

def checkParam():
  queueOutput(0,rospy.get_param('/dashboard/ind/sensors/stat'),1)   #Camera OK
  queueOutput(0,rospy.get_param('/dashboard/ind/rsocket/enable'),2)   #Robot OK

###Pub Sub
rospy.Subscriber("/request/capture",Bool,cb_do_capt)
rospy.Subscriber("/response/capture",Bool,cb_done_capt)
rospy.Subscriber("/request/solve",Bool,cb_do_solve)
rospy.Subscriber("/response/solve",Bool,cb_done_solve)
rospy.Subscriber("/request/recipe",Bool,cb_do_recipe)
rospy.Subscriber("/response/recipe",Bool,cb_done_recipe)
rospy.Subscriber("/report",String,cb_error)
pub_reset=rospy.Publisher("/request/clear",Bool,queue_size=1)

while True:
  if rospy.is_shutdown(): sys.exit()

  plc.connect(Config["plc_ip"],Config["plc_port"])
  if plc._is_connected:
    print("PLC connected")
  else:
    print("PLC failed to connect")
    continue

  loop=True
  while True:
    rospy.sleep(0.33)
    if rospy.is_shutdown():
      loop=False
      break
    try:
      iregs=plc.batchread_wordunits(headdevice=Config['inregs'], readsize=4)
    except Exception as e:
      print('mcplink read failed',e)
      break

    chbit=inputRegs[0]^iregs[0]
    upEdge=chbit&iregs[0]
    downEdge=chbit&inputRegs[0]
    checkInRegs()
    checkParam()
    inputRegs=iregs

    print(outputRegs)

    try:
      plc.batchwrite_wordunits(headdevice=Config['outregs'], values=outputRegs)
    except Exception as e:
      print('mcplink write failed',e)
      break

  plc.close()
  rospy.sleep(3)
  if not loop: sys.exit()

