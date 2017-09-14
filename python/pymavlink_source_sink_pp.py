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

class pymavlink_source_sink_pp(gr.sync_block):
    """
    pymavlink_source_sink_pp sends and receives PDU messages which contain MAVLink Messages.  This is useful for connecting to a pixhawk flight controller or SITL module and sending and receiving messages.  The connection settings allow a connection to made over Serial or UDP interfaces.
    """
    def __init__(self, connection_string,baud_rate=57600):
        gr.sync_block.__init__(self,
            name="pymavlink_source_sink_pp",
            in_sig=None,
            out_sig=None)
        self.connection_string=connection_string
        self.baud_rate=baud_rate
        #self.message_port_register_in(pmt.intern("command"))
        #self.set_msg_handler(pmt.intern("command"), self.cmd_handler)
        self.message_port_register_in(pmt.intern("MAVLink_IN"))
        self.message_port_register_out(pmt.intern("MAVLink_OUT"))
        self.mavlink_connection = mavutil.mavlink_connection(connection_string,baud=baud_rate)
        self.set_msg_handler(pmt.intern("MAVLink_IN"), self.mavlink_handler)
        print (self.mavlink_connection)
        self.running=True
        #print self.mavlink_connection
        self.thread = threading.Thread(target=self.check_for_message)
        self.thread.daemon = True
        self.thread.start()

    def mavlink_handler(self,msg):
        meta = pmt.car(msg)
        data = pmt.to_python(pmt.cdr(msg))
        #turn message back to buf to send through the connection
        #data = pmt.to_python((msg))
        #print (type(data))
        #print (data['mavlink'])
        binarrymavlink=bytearray(data)
        mavmessage=self.mavlink_connection.mav.decode(binarrymavlink)
        #uncomment the line below to print actual mavlink messages
        #print(mavmessage)
        self.mavlink_connection.write(binarrymavlink)
        #self.mavlink_connection.write(binarrymavlink)
        #ref = pmt.dict_ref(data, key0, pmt.PMT_NIL)
        #print (ref)

    def check_for_message(self):
        # check_for_message is a thread reading the mavlink connection for messages, the actual device for this block
        MAVLink_message = pmt.make_dict()
        key =  pmt.intern('mavlink')
        while(self.running):
           #print ("check_for_message in thread")
           self.message=self.mavlink_connection.recv_match(blocking=True,timeout=10)
           if self.message!=None:
            if self.message.get_type() == 'BAD_DATA':
                self.message=None
           if(self.message!=None):
             #print ("message found serial")
             #print (self.message)
             buf=self.message.get_msgbuf()
             bufnp=numpy.frombuffer(buf,dtype=numpy.uint8)
             meta='mavlink'
             self.message_port_pub(pmt.intern("MAVLink_OUT"),pmt.cons(pmt.PMT_NIL,pmt.to_pmt(bufnp)))
             self.message=None    
    
    def __del__(self):
        self.running=False
        self.mavlink_connection.close()
        self.thread.close()       

    def general_work(self, input_items, output_items):
        pass
