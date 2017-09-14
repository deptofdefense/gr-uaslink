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

class pdu_vector_to_pdu_control(gr.sync_block):
    """
    pdu_vector_to_pdu_control converts a pdu vector of 8 bit ints into a pdu control message designed for further processing in the system.  It also incodes message information into the data vector
    """
    def __init__(self):
        gr.sync_block.__init__(self,
            name="pdu_vector_to_pdu_control",
            in_sig=None,
            out_sig=None)
        self.message_port_register_in(pmt.intern("Vector_IN"))
        self.message_port_register_out(pmt.intern("Control_OUT"))
        self.data=[0]*8
        self.set_msg_handler(pmt.intern("Vector_IN"), self.vector_handler)

    def vector_handler(self,msg):
        meta =  pmt.to_python(pmt.car(msg))
        databuf = pmt.to_python(pmt.cdr(msg))
        data=numpy.frombuffer(databuf,dtype=numpy.uint32)
        print "vector handler"
        print data
        self.data[0]=int(data[0])
        self.data[1]=int(data[1])
        self.data[2]=int(data[2])
        self.data[7]=int(data[7])
        if(data[8]==1):
          meta = pmt.to_pmt('takeoff')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
        elif(data[8]==2):
          meta = pmt.to_pmt('land')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)  
        elif(data[8]==3):
          meta = pmt.to_pmt('rc_override')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
        elif(data[8]==4):
          meta = pmt.to_pmt('disarm')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
        elif(data[8]==5):
          meta = pmt.to_pmt('heartbeat')
          pmtdata = pmt.to_pmt(self.data)
          msg=pmt.cons(meta, pmtdata)
        self.message_port_pub(pmt.intern("Control_OUT"),msg)
        
    def work(self, input_items, output_items):
        pass

