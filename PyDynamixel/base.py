# -*- coding: utf-8 -*-
"""
    - Author: Mauricio Moraes Godoy
    - Date: 2025-09-15
"""

from dynamixel_sdk import *

class BaseDynamixel:
    """Base class for Dynamixel actuators.

    Notes
    -----

    - This class stores shared state (IDs, handlers, cached values) and
      utility operations common to position and velocity modes.
    """

    def __init__(self, servo_id: int, center_value: int = 0):
        """Initialize base actuator state.

        Parameters
        ----------
        servo_id : int
            Device ID of the Dynamixel on the bus.
        center_value : int, optional
            Mechanical/electrical center offset used for position mode mappings.

        Notes
        -----
        Initializes cached goal/current values and placeholders for
        `PacketHandler` and `PortHandler` which are provided by the
        communication manager.
        """
        self.servo_id = servo_id
        self.center_value = center_value
        self.goal_value = 0
        self.curr_value = 0
        self.packet_handler = None
        self.port_handler = None
        self.adapter = None  # protocol adapter (v1 or v2)
        self.changed = False

    def _set_port_and_packet(self, port_handler, packet_handler):
        """Attach communication handlers.

        Parameters
        ----------
        port_handler : PortHandler
            Opened port handler used to communicate with the bus.
        packet_handler : PacketHandler
            Packet handler for the selected protocol.
        """
        self.packet_handler = packet_handler
        self.port_handler = port_handler

    def _set_adapter(self, adapter):
        """Attach a protocol adapter (1.0 or 2.0)."""
        self.adapter = adapter

    def ping(self):
        """Ping the device and return its model number.

        Returns
        -------
        int or None
            Model number on success, otherwise ``None``.
        """
        dxl_model, dxl_comm, dxl_error = self.packet_handler.ping(
            self.port_handler, self.servo_id
        )
        if dxl_comm != COMM_SUCCESS:
            print("Comm Error:", self.packet_handler.getTxRxResult(dxl_comm))
        elif dxl_error:
            print("Packet Error:", self.packet_handler.getRxPacketError(dxl_error))
        else:
            print(f"[ID:{self.servo_id}] Model {dxl_model}")
            return dxl_model

    def enable_torque(self, addr_torque_enable: int):
        """Enable torque for this actuator.

        Parameters
        ----------
        addr_torque_enable : int
            Control-table address for torque enable (e.g., 64 on MX series).
        """
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, addr_torque_enable, 1
        )

    def disable_torque(self, addr_torque_enable: int):
        """Disable torque for this actuator.

        Parameters
        ----------
        addr_torque_enable : int
            Control-table address for torque enable (e.g., 64 on MX series).
        """
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, addr_torque_enable, 0
        )
