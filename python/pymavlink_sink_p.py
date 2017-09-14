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

from pymavlink import mavutil
import pmt
from builtins import object

#from pymavlink.dialects.v10 import ardupilotmega as mavlink
#from pymavlink.dialects.v20 import ardupilotmega as mavlink2


class pymavlink_sink_p(gr.sync_block):
    """
    pymavlink_sink_p accepts pdu version of MAVlink commands and submits they to a MAVLink based flight control or GCS.  A good simple interface block although the 2 way requirements of MAVlink control makes the use of this block limited
    """
    def __init__(self, connection_string,baud_rate=57600):
        gr.sync_block.__init__(self,
            name="pymavlink_sink_p",
            in_sig=None,
            out_sig=None)
        self.connection_string=connection_string
        self.baud_rate=baud_rate
        #self.message_port_register_in(pmt.intern("command"))
        #self.set_msg_handler(pmt.intern("command"), self.cmd_handler)
        self.message_port_register_in(pmt.intern("MAVLink"))
        self.mavlink_connection = mavutil.mavlink_connection(connection_string,baud=baud_rate)
        self.set_msg_handler(pmt.intern("MAVLink"), self.mavlink_handler) 
        print (self.mavlink_connection)
            # we will use a fifo as an encode/decode buffer
                
    def mavlink_handler(self,msg):
        meta = pmt.car(msg);
        data = pmt.to_python(pmt.cdr(msg));
        #turn message back to buf to send through the connection
        #data = pmt.to_python((msg))
        #print (type(data))
        #print (data['mavlink'])
        binarrymavlink=bytearray(data)
        mavmessage=self.mavlink_connection.mav.decode(binarrymavlink)
        #print(mavmessage)
        self.mavlink_connection.write(binarrymavlink)
        #ref = pmt.dict_ref(data, key0, pmt.PMT_NIL)
        #print (ref)
    def __del__(self):
        self.mavlink_connection.close()
        
    def work(self, input_items, output_items):
        pass

