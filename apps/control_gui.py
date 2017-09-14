#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Control Gui
# Generated: Mon May 15 09:28:39 2017
##################################################

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

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "control_gui")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_up(self):
        return self.up

    def set_up(self, up):
        self.up = up

    def get_takeoff(self):
        return self.takeoff

    def set_takeoff(self, takeoff):
        self.takeoff = takeoff

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_right(self):
        return self.right

    def set_right(self, right):
        self.right = right

    def get_left(self):
        return self.left

    def set_left(self, left):
        self.left = left

    def get_land(self):
        return self.land

    def set_land(self, land):
        self.land = land

    def get_forward(self):
        return self.forward

    def set_forward(self, forward):
        self.forward = forward

    def get_down(self):
        return self.down

    def set_down(self, down):
        self.down = down

    def get_back(self):
        return self.back

    def set_back(self, back):
        self.back = back

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
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
