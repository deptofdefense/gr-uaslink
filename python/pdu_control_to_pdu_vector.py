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

import numpy
from gnuradio import gr
import pmt

class pdu_control_to_pdu_vector(gr.sync_block):
    """
    pdu_control_to_pdu_vector converts a control message from pdu sent over zmq to a vector of 8 bit ints required for packet based processin.  It also incodes message infromation into the data
    """
    def __init__(self):
        gr.sync_block.__init__(self,
            name="pdu_control_to_pdu_vector",
            in_sig=None,
            out_sig=None)
        self.message_port_register_in(pmt.intern("Control_IN"))
        self.message_port_register_out(pmt.intern("Vector_OUT"))
        self.data=[0]*9
        self.set_msg_handler(pmt.intern("Control_IN"), self.control_handler)
        
    def control_handler(self,msg):
        meta =  pmt.to_python(pmt.car(msg))
        data = pmt.to_python(pmt.cdr(msg))
        #print data
        if meta == 'takeoff':
           self.data[8]=1
        elif meta == 'land':
           self.data[8]=2   
        elif meta == 'rc_override':
           self.data[8]=3
           print (data)
        elif meta == 'disarm':
           self.data[8]=4
        elif meta == 'heartbeat':
            self.data[8]=5
        self.data[0]=data[0]
        self.data[1]=data[1]
        self.data[2]=data[2]
        self.data[7]=data[7]
        ndata=numpy.array(self.data,dtype=numpy.int32)
        nbytes=ndata.tobytes()
        bufnp=numpy.frombuffer(nbytes,dtype=numpy.uint8)
        
        self.message_port_pub(pmt.intern("Vector_OUT"),pmt.cons(pmt.PMT_NIL,pmt.to_pmt(bufnp)))
       
    def work(self, input_items, output_items):
        pass

