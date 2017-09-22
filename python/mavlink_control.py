#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2017 Oak Ridge National Laboratory.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from __future__ import print_function
import numpy
from gnuradio import gr
import threading
import datetime as dt
import time

from pymavlink import mavutil
import pmt
from builtins import object

#from pymavlink.dialects.v10 import ardupilotmega as mavlink

class mavlink_control(gr.sync_block):
    """
    This is used to receive messages from a control gui for flight control of a MAVLink device.  This block turns PMT control messages into MAVLink messages.  This block creates two instances of pymavlink connections to handle the data flow.  This currently emulates the basics of a GCS and provides data in such a way that MavProxy can connect to it for debugging processes.  This block also sends 1 second heartbeat messages to maintain the MAVLink link with a pixhawk flight controller.
    """
    def __init__(self, connection_string,baud_rate=57600):
        gr.sync_block.__init__(self,
            name="mavlink_control",
            in_sig=None,
            out_sig=None)


        self.connection_string=connection_string
        self.baud_rate=baud_rate
        #self.message_port_register_in(pmt.intern("command"))
        #self.set_msg_handler(pmt.intern("command"), self.cmd_handler)
        self.message_port_register_in(pmt.intern("MAVLink_IN"))
        self.message_port_register_in(pmt.intern("Command"))
        self.message_port_register_out(pmt.intern("MAVLink_OUT"))
        self.mavlink_connection = mavutil.mavlink_connection('udpout:'+connection_string,baud=baud_rate)
        self.mavlink2=mavutil.mavlink_connection('udpin:'+connection_string,baud=baud_rate)
        self.set_msg_handler(pmt.intern("MAVLink_IN"), self.mavlink_handler)
        self.set_msg_handler(pmt.intern("Command"), self.command_handler)
        print (self.mavlink_connection)
        self.running=True
        self.takeoff=0
        #print self.mavlink_connection
        self.thread = threading.Thread(target=self.check_for_message)
        self.thread.daemon = True
        self.thread.start()
        self.thread2 = threading.Thread(target=self.check_for_message2)
        self.thread2.daemon = True
        self.thread2.start()
        self.data=data=[0]*8
        self.last_heartbeat_time=dt.datetime.now()
        self.thread3=threading.Thread(target=self.send_heartbeat)
        self.thread3.daemon = True
        self.thread3.start()
        
    
    def set_land(self):
        self.takeoff=0
        self.mavlink2.set_mode('LAND')
    
    #def set_servo(self,data):
    #    print(self.data)
    #    print(data)
    #    for i in range (3):
    #        if self.data[i] != data[i]:
    #            self.mavlink2.set_servo(i+1,data[i])
    #            print('servo ' + str(i+1) + 'set to ' +str(data[i]))
        #self.mavlink2.set_servo(channel,pwm)
    def set_channel_overrides(self,data):
         #using set_overrides prevenet the controler of solo from taking command of a out of control UAS
         #self.set_servo(data) #temporary replace for test test set_channel_overrides
         self.mavlink2.mav.rc_channels_override_send(
			self.mavlink2.target_system, self.mavlink2.target_component, *data)
    
    def arm_and_takeoff(self,data):
        #current_location=self.mavlink_connection.location(True)
        #print (current_location)
        if (self.takeoff==1):
            print ('currently in takeoff mode, switch to land mode before attempting again')
            return
        self.last_heartbeat_time=dt.datetime.now()
        self.takeoff=1
        print ("hold")
 
        #data=[0]*8
        #should be 1500 for alt_hold
        self.data[0]=data[0]
        self.data[1]=data[1]
        self.data[2]=data[2]
        
        print('before overide')
        self.set_channel_overrides(self.data)
        time.sleep(0.5)
        #Get the motors working after arming (stablilze requires an input)
        #print('set servos')
        #self.mavlink2.set_relay(1)
        #self.mavlink2.set_relay(2)
        #self.mavlink2.set_relay(3)
        #self.mavlink2.set_servo(1,self.data[0])
        #self.mavlink2.set_servo(2,self.data[1])
        #self.mavlink2.set_servo(3,self.data[2])
        #print(self.data)
        #print('after servo')
        if(data[7]==0):
          self.mavlink2.set_mode('STABILIZE')
        elif(data[7]==1):
          self.mavlink2.set_mode('ALT_HOLD')
        elif(data[7]==2):
          self.mavlink2.set_mode('LOITER')
        time.sleep(0.5)
        print('first set')
        
        if not self.mavlink2.motors_armed(): # Function to check if UAV is armed
          print('in_Armed')
          self.mavlink2.arducopter_arm() # Function to ARM the UAV
          time.sleep(0.3)
          print('Armed called')
          #Do not proceed until the UAV is armed
          #self.mavlink2.motors_armed_wait() # Function to wait till the UAV is armed
          print ('Armed wait')
          
          #give up throttle
          #self.data[2]=self.data[2]+150
          tempdata=[0]*8
          tempdata[0]=data[0]
          tempdata[1]=data[1]
          tempdata[2]=data[2]
          tempdata[2]=tempdata[2]+200
          print('before override set')
          #self.set_servo(tempdata)
          self.set_channel_overrides(tempdata)
          
          print('after override')
          
          
            
    def command_handler(self,msg):
        print ('receive message')
        meta =  pmt.to_python(pmt.car(msg))
        data = pmt.to_python(pmt.cdr(msg))
        print (meta)
        print (data)
        if meta == 'takeoff':
           print ('call takeoff')
           self.arm_and_takeoff(data)
        elif meta == 'rc_override':
           print (data)
           #self.data=data
           print ('rc_override')
           #self.set_servo(data)
           self.set_channel_overrides(data)
           self.data=data
        elif meta == 'land':
           print('land')
           self.set_land()
        elif meta == 'disarm':
           print('disarm')
           self.disarm() 
        elif meta == 'heartbeat':
           print('heartbeat')
           self.receive_hearbeat()
           
    def mavlink_handler(self,msg):
        meta = pmt.car(msg)
        data = pmt.to_python(pmt.cdr(msg))
        binarrymavlink=bytearray(data)
        mavmessage=self.mavlink_connection.mav.decode(binarrymavlink)
        #print(mavmessage)
        self.mavlink_connection.write(binarrymavlink)
       
    def send_heartbeat(self):
        self.mavlink2.wait_heartbeat()
        while(self.running):
          if (self.takeoff!=0):
            currentimem5=dt.datetime.now()-dt.timedelta(seconds=5)
            print('in send_heartbeat')
            print(currentimem5)
            print(self.last_heartbeat_time)
            if(currentimem5>self.last_heartbeat_time): #if we have missed all messages for more than 5 seconds
              print('lost link')
              self.set_land()
              
            if (self.takeoff!=0):
              self.mavlink2.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
              self.set_channel_overrides(self.data)
          time.sleep(1.0)
    def disarm(self):
        self.data[0]=0
        self.data[1]=0
        self.data[2]=0
        self.mavlink2.arducopter_disarm()
    
    def receive_hearbeat(self):
        print('heartbeat received')
        self.last_heartbeat_time=dt.datetime.now()
        print(self.last_heartbeat_time)
    
    def check_for_message(self):
        # Make an empty dictionary
        MAVLink_message = pmt.make_dict()
        key =  pmt.intern('mavlink')
        while(self.running):
           #print ("check_for_message in thread")
           self.message=self.mavlink_connection.recv_match(blocking=True, timeout=10)
           if self.message!=None:
            if self.message.get_type() == 'BAD_DATA':
                self.message=None
           if(self.message!=None):
             #print ("message_1 found")
             #print (self.message)
             buf=self.message.get_msgbuf()
             bufnp=numpy.frombuffer(buf,dtype=numpy.uint8)
             meta='mavlink'
             self.message_port_pub(pmt.intern("MAVLink_OUT"),pmt.cons(pmt.PMT_NIL,pmt.to_pmt(bufnp)))
             self.message=None        
    
    
    def check_for_message2(self):
        # Make an empty dictionary
        MAVLink_message = pmt.make_dict()
        key =  pmt.intern('mavlink')
        while(self.running):
           #print ("check_for_message in thread")
           self.message2=self.mavlink2.recv_match(blocking=True, timeout=10)
           if self.message2!=None:
            if self.message2.get_type() == 'BAD_DATA':
                self.message2=None
           if(self.message2!=None):
             #print ("message_2 found")
             #print (self.message2)
             buf=self.message2.get_msgbuf()
             bufnp=numpy.frombuffer(buf,dtype=numpy.uint8)
             meta='mavlink'
             #self.message_port_pub(pmt.intern("MAVLink_OUT"),pmt.cons(pmt.PMT_NIL,pmt.to_pmt(bufnp)))
             self.message2=None    
    
    def __del__(self):
        self.running=False
        self.mavlink_connection.close()
        self.mavlink2.close()
        self.thread.close()       
        self.thread2.close()
        self.thread3.close()
        
       

    def work(self, input_items, output_items):
        pass

