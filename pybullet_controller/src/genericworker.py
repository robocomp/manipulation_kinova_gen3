#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2024 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior




class GenericWorker(QtCore.QObject):

    kill = QtCore.Signal()

    def __init__(self, mprx):
        """
        initializes an instance of the `GenericWorker` class, setting up a mutex
        and a timer for periodic execution with a period of 30 seconds.

        Args:
            mprx (`object`.): QMutex object used to protect the worker's state.
                
                		- `mutex`: A `QMutex` object for synchronizing access to the
                internal state of the worker. It uses the `Recursive` mutex
                semantics, which allows the timer to interrupt the worker's execution.
                		- `Period`: An integer value representing the period of time
                between timer events in milliseconds.
                		- `timer`: A `QTimer` object for scheduling timer events to
                interrupt the worker's execution at regular intervals.
                

        """
        super(GenericWorker, self).__init__()


        self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
        self.Period = 30
        self.timer = QtCore.QTimer(self)


    @QtCore.Slot()
    def killYourSelf(self):
        """
        emits the `kill` signal, which is caught and leads to an unhandled exception
        being thrown and the program ending its execution.

        """
        rDebug("Killing myself")
        self.kill.emit()

    # \brief Change compute period
    # @param per Period in ms
    @QtCore.Slot(int)
    def setPeriod(self, p):
        """
        sets a new period and updates the ` Period` attribute of the class, before
        starting the timer based on the new period using the `timer.start()` method.

        Args:
            p (int): new period to be set for the timer, which is then assigned
                to the ` Period` attribute of the object and started using the
                `start()` method.

        """
        print("Period changed", p)
        self.Period = p
        self.timer.start(self.Period)
