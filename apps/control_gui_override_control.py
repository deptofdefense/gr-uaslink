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
import numpy
import threading
import datetime as dt
import time



        
class control_gui(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Control Gui")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Control Gui")
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
        self.disarm = disarm = 0
        self.forward = forward = 0
        self.down = down = 0
        self.back = back = 0
        self.speed = speed = 100
        self.overridevalue0 = 1500
        self.overridevalue1 = 1500
        self.overridevalue2 = 1450
        self.data=[0]*8
        self.data[0]=self.overridevalue0
        self.data[1]=self.overridevalue1
        self.data[2]=self.overridevalue2
        self.mode = 'STABILIZE'
        self.duration = 1
        self.arm = 0
        self.connection_string='udp:127.0.0.1:14552'
        #self.connection_string='/dev/ttyUSB0'
         
        #Create 1 sec thread send
        self.running=True
        self.thread = threading.Thread(target=self.send_heartbeat)
        self.thread.daemon = True
        self.flying=0
        
        self.thread.start()
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
        _disarm_push_button = Qt.QPushButton('Disarm')
        self._disarm_choices = {'Pressed': 1, 'Released': 0}
        _disarm_push_button.pressed.connect(lambda: self.set_disarm(self._disarm_choices['Pressed']))
        _disarm_push_button.released.connect(lambda: self.set_disarm(self._disarm_choices['Released']))
        self.top_grid_layout.addWidget(_disarm_push_button, 3,1,1,1)
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
        self._speed_tool_bar = Qt.QToolBar(self)
        self._speed_tool_bar.addWidget(Qt.QLabel('speed'+": "))
        self._speed_line_edit = Qt.QLineEdit(str(self.speed))
        self._speed_tool_bar.addWidget(self._speed_line_edit)
        self._speed_line_edit.returnPressed.connect(
        	lambda: self.set_speed(eng_notation.str_to_num(str(self._speed_line_edit.text().toAscii()))))
        self.top_grid_layout.addWidget(self._speed_tool_bar, 1,2,1,1)
        self._overridevalue0_tool_bar = Qt.QToolBar(self)
        self._overridevalue0_tool_bar.addWidget(Qt.QLabel('RC1:Sideways'+": "))
        self._overridevalue0_line_edit = Qt.QLineEdit(str(self.overridevalue0))
        self._overridevalue0_tool_bar.addWidget(self._overridevalue0_line_edit)
        self._overridevalue0_line_edit.returnPressed.connect(
        	lambda: self.set_overridevalue0(eng_notation.str_to_num(str(self._overridevalue0_line_edit.text().toAscii()))))
        self.top_grid_layout.addWidget(self._overridevalue0_tool_bar, 2,2,1,1)
        self._overridevalue1_tool_bar = Qt.QToolBar(self)
        self._overridevalue1_tool_bar.addWidget(Qt.QLabel('RC2:FrontBack'+": "))
        self._overridevalue1_line_edit = Qt.QLineEdit(str(self.overridevalue1))
        self._overridevalue1_tool_bar.addWidget(self._overridevalue1_line_edit)
        self._overridevalue1_line_edit.returnPressed.connect(
        	lambda: self.set_overridevalue1(eng_notation.str_to_num(str(self._overridevalue1_line_edit.text().toAscii()))))
        self.top_grid_layout.addWidget(self._overridevalue1_tool_bar, 3,2,1,1)
        self._overridevalue2_tool_bar = Qt.QToolBar(self)
        self._overridevalue2_tool_bar.addWidget(Qt.QLabel('RC3:Vertical'+": "))
        self._overridevalue2_line_edit = Qt.QLineEdit(str(self.overridevalue2))
        self._overridevalue2_tool_bar.addWidget(self._overridevalue2_line_edit)
        self._overridevalue2_line_edit.returnPressed.connect(
        	lambda: self.set_overridevalue2(eng_notation.str_to_num(str(self._overridevalue2_line_edit.text().toAscii()))))
        self.top_grid_layout.addWidget(self._overridevalue2_tool_bar, 4,2,1,1)
        self._mode_options = ('STABILIZE', 'ALT_HOLD','LOITER', )
        self._mode_labels = (str(self._mode_options[0]), str(self._mode_options[1]),str(self._mode_options[2]),  )
        self._mode_tool_bar = Qt.QToolBar(self)
        self._mode_tool_bar.addWidget(Qt.QLabel('Mode'+": "))
        self._mode_combo_box = Qt.QComboBox()
        self._mode_tool_bar.addWidget(self._mode_combo_box)
        for label in self._mode_labels: self._mode_combo_box.addItem(label)
        self._mode_callback = lambda i: Qt.QMetaObject.invokeMethod(self._mode_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._mode_options.index(i)))
        self._mode_callback(self.mode)
        self._mode_combo_box.currentIndexChanged.connect(
        	lambda i: self.set_mode(self._mode_options[i]))
        self.top_grid_layout.addWidget(self._mode_tool_bar, 5,2,1,1)
        

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
    
   

    def get_up(self):
        return self.up

    def set_up(self, up):
        self.up = up
        if self.up==1:
           self.data[2]=self.overridevalue2+self.speed
           self.rc_override(self.data)
        if self.up==0:    
           self.data[2]=self.overridevalue2
           self.rc_override(self.data)
        
    def get_takeoff(self):
        return self.takeoff
    
    def rc_override(self,data):
        meta = pmt.to_pmt('rc_override')
        pmtdata = pmt.to_pmt(data)
        msg=pmt.cons(meta, pmtdata)
        print('send_zmq')
        self.zmqc.send(pmt.serialize_str(msg))
        #time.sleep(0.1)
        #self.zmqc.send(pmt.serialize_str(msg))
        
    def set_takeoff(self, takeoff):
        print takeoff
        
        if(takeoff==1):
           meta = pmt.to_pmt('takeoff')
           if (self.mode == 'STABILIZE'):
               self.data[7]=0
           elif (self.mode == 'ALT_HOLD'):
               self.data[7]=1
           elif (self.mode == 'LOITER'):
               self.data[7]=2
           pmtdata = pmt.to_pmt(self.data)
           msg=pmt.cons(meta, pmtdata)
           print('send_zmq')
           self.zmqc.send(pmt.serialize_str(msg))
           self.flying=1
            
    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_right(self):
        return self.right

    def set_right(self, right):
        self.right = right
        if self.right==1: #move
           self.data[0]=self.overridevalue0+self.speed
           self.rc_override(self.data)
        if self.right==0:  #stop
           self.data[0]=self.overridevalue0
           self.rc_override(self.data)
        
    def get_left(self):
        return self.left

    def set_left(self, left):
        self.left = left
        if self.left==1: #move
           self.data[0]=self.overridevalue0-self.speed
           self.rc_override(self.data)
        if self.left==0:  #stop
           self.data[0]=self.overridevalue0
           self.rc_override(self.data)
       

    def get_land(self):
        return self.land
    
    def get_disarm(self):
        return self.disarm

    def set_land(self, land):
        self.land = land
        if  self.land==1:
          meta = pmt.to_pmt('land')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
          print('send_zmq')
          self.zmqc.send(pmt.serialize_str(msg))
          self.flying=0
          #time.sleep(0.1)
          #self.zmqc.send(pmt.serialize_str(msg))


    def set_disarm(self, disarm):
        self.disarm = disarm
        if self.disarm==1:
          meta = pmt.to_pmt('disarm')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
          print('send_zmq disarm')
          self.zmqc.send(pmt.serialize_str(msg))
        
    def get_forward(self):
        return self.forward
               

    def set_forward(self, forward):
        self.forward = forward
        if self.forward==1: #move
           self.data[1]=self.overridevalue1-self.speed
           self.rc_override(self.data)
        if self.forward==0:  #stop
           self.data[1]=self.overridevalue1
           self.rc_override(self.data)
        
        
    def get_down(self):
        return self.down

    def set_down(self, down):
        self.down = down
        if self.down==1: #move
           self.data[2]=self.overridevalue2-self.speed
           self.rc_override(self.data)
        if self.down==0:  #stop
           self.data[2]=self.overridevalue2
           self.rc_override(self.data)
        
        
    def get_back(self):
        return self.back

    def set_back(self, back):
        self.back = back
        if self.back==1: #move
           self.data[1]=self.overridevalue1+self.speed
           self.rc_override(self.data)
        if self.back==0:  #stop
           self.data[1]=self.overridevalue1
           self.rc_override(self.data)
       
        
    def get_speed(self):
        return self.speed
   
    def get_overridevalue0(self):
        return self.overridevalue0
    
    def get_overridevalue1(self):
        return self.overridevalue1
    
    def get_overridevalue2(self):
        return self.overridevalue2
    
    def get_mode(self):
        return self.mode

    def set_mode(self, mode):
        self.mode = mode
        self._mode_callback(self.mode)

    def set_speed(self, speed):
        self.speed = speed
        Qt.QMetaObject.invokeMethod(self._speed_line_edit, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.speed)))
    
    def set_overridevalue0(self, overridevalue0):
        self.overridevalue0 = overridevalue0
        self.data[0]=overridevalue0
        Qt.QMetaObject.invokeMethod(self._overridevalue0_line_edit, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.overridevalue0)))
        
    def set_overridevalue1(self, overridevalue1):
        self.overridevalue1 = overridevalue1
        self.data[1]=overridevalue1
        Qt.QMetaObject.invokeMethod(self._overridevalue1_line_edit, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.overridevalue1)))
        
    def set_overridevalue2(self, overridevalue2):
        self.overridevalue2 = overridevalue2
        self.data[2]=overridevalue2
        Qt.QMetaObject.invokeMethod(self._overridevalue2_line_edit, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.overridevalue2)))
    def send_heartbeat(self):
        while(self.running):
          if self.flying==1:
            meta = pmt.to_pmt('heartbeat')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            print('send_zmq heartbeat')
            self.zmqc.send(pmt.serialize_str(msg))
          time.sleep(1.0)

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
        tb.running=False
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
