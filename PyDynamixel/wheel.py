# -*- coding: utf-8 -*-

"""
    - Author: Mauricio Moraes Godoy
    - Date: 2025-09-15
"""

from dynamixel_sdk import *
from .base import BaseDynamixel
from .constants import ADDR_GOAL_VELOCITY, ADDR_PRESENT_VELOCITY

class Wheel(BaseDynamixel):
    """Velocity-mode Dynamixel actuator.

    Notes
    -----

    - Converts between RPM and internal units for goal and present velocity.
    """

    def set_goal_velocity(self, rpm: float):
        """Set desired wheel velocity in RPM.

        Parameters
        ----------
        rpm : float
            Target rotational speed in revolutions per minute.
        """
        units = int(round(rpm / 0.229))  # rpm -> internal units (signed 32-bit)
        if units < 0:
            # encode as unsigned 32-bit two's complement
            units = (units + (1 << 32)) & 0xFFFFFFFF
        self.goal_value = units
        self.changed = True

    def send_velocity(self, rpm: float):
        """Send velocity command in RPM to the device.

        Parameters
        ----------
        rpm : float
            Target rotational speed in revolutions per minute.
        """
        self.set_goal_velocity(rpm)
        if hasattr(self, 'adapter') and self.adapter is not None:
            dxl_comm, dxl_error = self.adapter.write_goal_velocity_rpm(
                self.port_handler, self.packet_handler, self.servo_id, rpm
            )
        else:
            dxl_comm, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.servo_id, ADDR_GOAL_VELOCITY, self.goal_value
            )
        if dxl_comm != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm))
        elif dxl_error:
            print(self.packet_handler.getRxPacketError(dxl_error))

    def get_velocity(self):
        """Read present velocity in RPM.

        Returns
        -------
        float
            Present rotational speed in RPM.
        """
        if hasattr(self, 'adapter') and self.adapter is not None:
            return self.adapter.read_present_velocity_rpm(
                self.port_handler, self.packet_handler, self.servo_id
            )
        self.curr_value, dxl_comm, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.servo_id, ADDR_PRESENT_VELOCITY
        )
        if dxl_comm != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm))
        elif dxl_error:
            print(self.packet_handler.getRxPacketError(dxl_error))
        # decode signed 32-bit two's complement
        raw = int(self.curr_value)
        if raw & (1 << 31):
            raw -= (1 << 32)
        return float(raw) * 0.229  # internal units -> rpm
