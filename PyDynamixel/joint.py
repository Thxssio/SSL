# -*- coding: utf-8 -*-

"""
    - Author: Mauricio Moraes Godoy
    - Date: 2025-09-15
"""

from math import pi
from dynamixel_sdk import *
from .base import BaseDynamixel
from .constants import ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION

class Joint(BaseDynamixel):
    """Position-mode Dynamixel actuator.

    Notes
    -----

    - Converts human-friendly angles to control-table units and vice-versa.
    """

    def set_goal_angle(self, angle: float, radian: bool = False):
        """Set desired position angle.

        Parameters
        ----------
        angle : float
            Target angle, in degrees by default.
        radian : bool, optional
            If ``True``, interpret ``angle`` in radians.
        """
        if radian:
            self.goal_value = int(2048.0 * angle / pi) + self.center_value
        else:
            self.goal_value = int(2048.0 * angle / 180.0) + self.center_value
        self.changed = True

    def send_angle(self, angle: float, radian: bool = False):
        """Send a goal angle to the device.

        Parameters
        ----------
        angle : float
            Target angle, in degrees by default.
        radian : bool, optional
            If ``True``, interpret ``angle`` in radians.
        """
        self.set_goal_angle(angle, radian=radian)
        dxl_comm, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.servo_id, ADDR_GOAL_POSITION, self.goal_value
        )
        if dxl_comm != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm))
        elif dxl_error:
            print(self.packet_handler.getRxPacketError(dxl_error))

    def get_angle(self, radian: bool = False):
        """Read present position angle.

        Parameters
        ----------
        radian : bool, optional
            If ``True``, return value in radians; otherwise in degrees.

        Returns
        -------
        float
            Present angle in degrees or radians.
        """
        self.curr_value, dxl_comm, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.servo_id, ADDR_PRESENT_POSITION
        )
        if dxl_comm != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm))
        elif dxl_error:
            print(self.packet_handler.getRxPacketError(dxl_error))

        if radian:
            return pi * float(self.curr_value) / 2048.0
        else:
            return 180.0 * float(self.curr_value) / 2048.0
