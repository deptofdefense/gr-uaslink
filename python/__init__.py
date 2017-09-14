#
# Copyright 2008,2009 Free Software Foundation, Inc.
#
# This application is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This application is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

# The presence of this file turns this directory into a Python package

'''
This is the GNU Radio UASLINK module. Place your Python package
description here (python/__init__.py).
'''

# import swig generated symbols into the uaslink namespace
try:
	# this might fail if the module is python-only
	from uaslink_swig import *
except ImportError:
	pass

# import any pure python here
from pymavlink_source_p import pymavlink_source_p
from pymavlink_sink_p import pymavlink_sink_p
from pymavlink_source_sink_pp import pymavlink_source_sink_pp
from mavlink_control import mavlink_control

from pdu_control_to_pdu_vector import pdu_control_to_pdu_vector
from pdu_vector_to_pdu_control import pdu_vector_to_pdu_control
from burst_verification import burst_verification
#
