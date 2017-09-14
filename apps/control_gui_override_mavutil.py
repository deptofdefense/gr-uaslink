#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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
#Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import zmq

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import sys
from gnuradio import qtgui
import pmt


#class pub_msg_sink(object):
    #def __init__(self, addr, timeout, do_bind=True):
        #super(pub_msg_sink, self).__init__()
        #self.timeout = timeout
        #self.ctx = zmq.Context()
        #self.sock = self.ctx.socket(zmq.PUB)
        #if do_bind:
            #self.sock.setsockopt(zmq.LINGER, 0)
            #self.sock.bind(addr)
        #else:
            #self.sock.connect(addr)

    #def __del__(self):
        #self.stop()

    #def stop(self):
        #if getattr(self, 'sock', None):
            #self.sock.close()
            #del self.sock

        #if getattr(self, 'ctx', None):
            #del self.ctx

    #def send(self, data):
        #self.sock.send(data)
        
class control_gui(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Control Gui")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Control Gui")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "control_gui")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())
        

        ##################################################
        # Variables
        ##################################################
        self.up = up = 0
        self.takeoff = takeoff = 0
        self.samp_rate = samp_rate = 32000
        self.right = right = 0
        self.left = left = 0
        self.land = land = 0
        self.forward = forward = 0
        self.down = down = 0
        self.back = back = 0
        self.altitude = altitude = 1
        self.duration = 1
        self.arm = 0
        self.connection_string='udp:127.0.0.1:14552'
        #self.connection_string='/dev/ttyUSB0'

        ##################################################
        # Blocks
        ##################################################
        _up_push_button = Qt.QPushButton('Up')
        self._up_choices = {'Pressed': 1, 'Released': 0}
        _up_push_button.pressed.connect(lambda: self.set_up(self._up_choices['Pressed']))
        _up_push_button.released.connect(lambda: self.set_up(self._up_choices['Released']))
        self.top_grid_layout.addWidget(_up_push_button, 1,3,1,1)
        _takeoff_push_button = Qt.QPushButton('Take Off')
        self._takeoff_choices = {'Pressed': 1, 'Released': 0}
        _takeoff_push_button.pressed.connect(lambda: self.set_takeoff(self._takeoff_choices['Pressed']))
        _takeoff_push_button.released.connect(lambda: self.set_takeoff(self._takeoff_choices['Released']))
        self.top_grid_layout.addWidget(_takeoff_push_button, 1,1,1,1)
        _right_push_button = Qt.QPushButton('Right')
        self._right_choices = {'Pressed': 1, 'Released': 0}
        _right_push_button.pressed.connect(lambda: self.set_right(self._right_choices['Pressed']))
        _right_push_button.released.connect(lambda: self.set_right(self._right_choices['Released']))
        self.top_grid_layout.addWidget(_right_push_button, 2,6,1,1)
        _left_push_button = Qt.QPushButton('Left')
        self._left_choices = {'Pressed': 1, 'Released': 0}
        _left_push_button.pressed.connect(lambda: self.set_left(self._left_choices['Pressed']))
        _left_push_button.released.connect(lambda: self.set_left(self._left_choices['Released']))
        self.top_grid_layout.addWidget(_left_push_button, 2,4,1,1)
        _land_push_button = Qt.QPushButton('Land')
        self._land_choices = {'Pressed': 1, 'Released': 0}
        _land_push_button.pressed.connect(lambda: self.set_land(self._land_choices['Pressed']))
        _land_push_button.released.connect(lambda: self.set_land(self._land_choices['Released']))
        self.top_grid_layout.addWidget(_land_push_button, 2,1,1,1)
        _forward_push_button = Qt.QPushButton('Forward')
        self._forward_choices = {'Pressed': 1, 'Released': 0}
        _forward_push_button.pressed.connect(lambda: self.set_forward(self._forward_choices['Pressed']))
        _forward_push_button.released.connect(lambda: self.set_forward(self._forward_choices['Released']))
        self.top_grid_layout.addWidget(_forward_push_button, 1,5,1,1)
        _down_push_button = Qt.QPushButton('Down')
        self._down_choices = {'Pressed': 1, 'Released': 0}
        _down_push_button.pressed.connect(lambda: self.set_down(self._down_choices['Pressed']))
        _down_push_button.released.connect(lambda: self.set_down(self._down_choices['Released']))
        self.top_grid_layout.addWidget(_down_push_button, 2,3,1,1)
        _back_push_button = Qt.QPushButton('Back')
        self._back_choices = {'Pressed': 1, 'Released': 0}
        _back_push_button.pressed.connect(lambda: self.set_back(self._back_choices['Pressed']))
        _back_push_button.released.connect(lambda: self.set_back(self._back_choices['Released']))
        self.top_grid_layout.addWidget(_back_push_button, 3,5,1,1)
        self._altitude_tool_bar = Qt.QToolBar(self)
        self._altitude_tool_bar.addWidget(Qt.QLabel('altitude'+": "))
        self._altitude_line_edit = Qt.QLineEdit(str(self.altitude))
        self._altitude_tool_bar.addWidget(self._altitude_line_edit)
        self._altitude_line_edit.returnPressed.connect(
        	lambda: self.set_altitude(eng_notation.str_to_num(str(self._altitude_line_edit.text().toAscii()))))
        self.top_grid_layout.addWidget(self._altitude_tool_bar, 2,2,1,1)
        
        
    
        
        # Start simulator
        #print "Start simulator (SITL)"
      
        #self.sitl = dronekit_sitl.start_default()
        #self.connection_string = self.sitl.connection_string()
        #self.connection_string = 
    def zmq_setup(self,zmq_addr):
        #zmq_addr='tcp://127.0.0.1:4003' #port for receiving messages
        self.timeout=100
        self.ctx = zmq.Context()
        self.zmqc = self.ctx.socket(zmq.PUB)
        self.zmqc.setsockopt(zmq.LINGER, 0)
        self.zmqc.bind(zmq_addr)
        #self.zmqc.connect(zmq_addr)
        #zmqc.setsockopt(zmq.SUBSCRIBE, "")
         
    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "control_gui")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()
    
    #def connect_to_vehicle(self,connection_string,baud_rate):
    #    self.connection_string=connection_string
    #    self.baud_rate=baud_rate
    #    print("Connecting to vehicle on: %s" % (self.connection_string,))
        #self.vehicle = connect(self.connection_string,baud=57600, wait_ready='gps_0')
   #     self.mavlink_connection = mavutil.mavlink_connection(self.connection,   baud=self.baud_rate)
   #     print("connected")
    #def send_rc(master, data):
	#    for i in xrange(1):
	#	    self.mavlink_connection.mav.rc_channels_override_encode (
	#		     master.target_system, master.target_component, *data)
    #        print ("sending rc: %s"%data)
    def arm_and_takeoff(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        
        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.vehicle.armed = True
        self.vehicle.channels.overrides = {'3':1650}

        while not self.vehicle.armed:      
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt 
            print " Velocity: %s" % self.vehicle.velocity
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print "Reached target altitude"
                break
            time.sleep(0.5)

    def get_up(self):
        return self.up

    def set_up(self, up):
        self.up = up
        if self.up==1:
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1650
           self.rc_override(data)
        if self.up==0:    
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        
    def get_takeoff(self):
        return self.takeoff
    
    def rc_override(self,data):
        meta = pmt.to_pmt('rc_override')
        pmtdata = pmt.to_pmt(data)
        msg=pmt.cons(meta, pmtdata)
        print('send_zmq')
        self.zmqc.send(pmt.serialize_str(msg))
        
    def set_takeoff(self, takeoff):
        if(takeoff==1):
           meta = pmt.to_pmt('takeoff')
           data = pmt.to_pmt(self.altitude)
           msg=pmt.cons(meta, data)
           print('send_zmq')
           self.zmqc.send(pmt.serialize_str(msg))
            #ARM vechcile
            #self.arm_and_takeoff(self.altitude)
            #self.vehicle.channels.overrides = {'3':1500}
            
    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_right(self):
        return self.right

    def set_right(self, right):
        self.right = right
        if self.right==1: #move
           data=[0]*8
           data[0]=1650
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        if self.right==0:  #stop
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        
    def get_left(self):
        return self.left

    def set_left(self, left):
        self.left = left
        if self.left==1: #move
           data=[0]*8
           data[0]=1350
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        if self.left==0:  #stop
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
       

    def get_land(self):
        return self.land

    def set_land(self, land):
        self.land = land
        meta = pmt.to_pmt('land')
        data = pmt.to_pmt(1)
        msg=pmt.cons(meta, data)
        print('send_zmq')
        self.zmqc.send(pmt.serialize_str(msg))

    def get_forward(self):
        return self.forward
               

    def set_forward(self, forward):
        self.forward = forward
        if self.forward==1: #move
           data=[0]*8
           data[0]=1500
           data[1]=1350
           data[2]=1500
           self.rc_override(data)
        if self.forward==0:  #stop
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        
        
    def get_down(self):
        return self.down

    def set_down(self, down):
        self.down = down
        if self.down==1: #move
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1350
           self.rc_override(data)
        if self.down==0:  #stop
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
        
        
    def get_back(self):
        return self.back

    def set_back(self, back):
        self.back = back
        if self.back==1: #move
           data=[0]*8
           data[0]=1500
           data[1]=1650
           data[2]=1500
           self.rc_override(data)
        if self.back==0:  #stop
           data=[0]*8
           data[0]=1500
           data[1]=1500
           data[2]=1500
           self.rc_override(data)
       
        
    def get_altitude(self):
        return self.altitude
   
    

    def set_altitude(self, altitude):
        self.altitude = altitude
        Qt.QMetaObject.invokeMethod(self._altitude_line_edit, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.altitude)))


def main(top_block_cls=control_gui, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)
    
    tb = top_block_cls()
    #connection_string = tb.sitl.connection_string()
    #print connection_string
    #tb.connect_to_vehicle('udp:127.0.0.1:14552',baud_rate)
    tb.zmq_setup('tcp://127.0.0.1:14000')
    tb.start()
    tb.show()

    # Connect to the Vehicle.
    #print("Connecting to vehicle on: %s" % (connection_string,))
    #vehicle = connect(connection_string, wait_ready=True)
    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
