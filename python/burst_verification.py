#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2017 <+YOU OR YOUR COMPANY+>.
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

class burst_verification(gr.sync_block):
    """
    burst_verification is used to verify that a result from the burst detection meets a limited amount of testing verification as a good block.  It does this by comparing rc rc_override fields.  This block will need to be modified if the control message format changes.
    """
    def __init__(self):
        gr.sync_block.__init__(self,
            name="burst_verification",
            in_sig=None,
            out_sig=None)
        self.message_port_register_in(pmt.intern("PDU_IN"))
        self.message_port_register_out(pmt.intern("Control_OUT"))
        self.data=[0]*8
        self.set_msg_handler(pmt.intern("PDU_IN"), self.pdu_handler)
        
        
    def pdu_handler(self,msg):
        meta = pmt.car(msg);
        databuf = pmt.to_python(pmt.cdr(msg))
        databuf = databuf[0:277]
        data=numpy.frombuffer(databuf,dtype=numpy.uint32)
        good_count=0
        if(data[0]==data[1]):
           good_count +=1
        if(data[0]==data[2]):
           good_count +=1
        if(data[1]==data[2]):
           good_count+=1
        if(int(data[7])==1 or int(data[7])==0):
           good_count+=1
        if(data[8]==1 or data[8]==2 or data[8] ==3):
           good_count+=1
        if(good_count >= 1):
          self.data[0]=int(data[0])
          self.data[1]=int(data[1])
          self.data[2]=int(data[2])
          self.data[7]=int(data[7])
          if(data[8]==1):
            meta = pmt.to_pmt('takeoff')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            self.message_port_pub(pmt.intern("Control_OUT"),msg) 
          elif(data[8]==2):
            meta = pmt.to_pmt('land')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            self.message_port_pub(pmt.intern("Control_OUT"),msg) 
          elif(data[8]==3):
            meta = pmt.to_pmt('rc_override')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            self.message_port_pub(pmt.intern("Control_OUT"),msg) 
          elif(data[8]==4):
            meta = pmt.to_pmt('diasrm')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            self.message_port_pub(pmt.intern("Control_OUT"),msg)
          elif(data[8]==5):
            meta = pmt.to_pmt('heartbeat')
            pmtdata = pmt.to_pmt(self.data)
            msg=pmt.cons(meta, pmtdata)
            self.message_port_pub(pmt.intern("Control_OUT"),msg)
           

    def work(self, input_items, output_items):
        pass

