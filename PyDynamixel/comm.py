# -*- coding: utf-8 -*-

"""
    - Author: Mauricio Moraes Godoy
    - Date: 2025-09-15
"""

from dynamixel_sdk import *
from math import pi
from .constants import *
from .adapters import ProtocolV2Adapter, ProtocolV1Adapter

class DxlComm:
    """Communication manager for mixed-protocol Dynamixel devices (1.0 and 2.0).

    Notes
    -----

    - Opens a serial port and provides sync-read and sync-write helpers for
      position and velocity.
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=57600):
        """Open the serial port and initialize sync groups.

        Parameters
        ----------
        port : str, optional
            Serial device path.
        baudrate : int, optional
            Serial baudrate to configure on the port.

        Raises
        ------
        RuntimeError
            If the port cannot be opened or baudrate cannot be set.
        """
        self.port_handler = PortHandler(port)
        self.packet_handler_v2 = PacketHandler(2.0)
        self.packet_handler_v1 = PacketHandler(1.0)

        if not self.port_handler.openPort():
            raise RuntimeError("❌ Failed to open port")
        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError("❌ Failed to set baudrate")

        # Adapters
        self.adapter_v2 = ProtocolV2Adapter()
        self.adapter_v1 = ProtocolV1Adapter()

        # registry of devices: id -> (device, adapter, packet_handler)
        self.devices = {}

    def attach(self, dxl):
        """Attach a device; probe protocol 2.0 then 1.0, and wire handlers.

        Parameters
        ----------
        dxl : BaseDynamixel
            Instance of a device such as `Joint` or `Wheel`.
        """
        servo_id = dxl.servo_id
        # Try protocol 2.0
        model, comm, err = self.packet_handler_v2.ping(self.port_handler, servo_id)
        if comm == COMM_SUCCESS and err == 0:
            dxl._set_port_and_packet(self.port_handler, self.packet_handler_v2)
            dxl._set_adapter(self.adapter_v2)
            self.devices[servo_id] = (dxl, self.adapter_v2, self.packet_handler_v2)
            return
        # Try protocol 1.0
        model, comm, err = self.packet_handler_v1.ping(self.port_handler, servo_id)
        if comm == COMM_SUCCESS and err == 0:
            dxl._set_port_and_packet(self.port_handler, self.packet_handler_v1)
            dxl._set_adapter(self.adapter_v1)
            self.devices[servo_id] = (dxl, self.adapter_v1, self.packet_handler_v1)
            return
        raise RuntimeError(f"Failed to detect protocol for ID {servo_id}")

    def enable_all_torque(self):
        """Enable torque for all attached devices (per protocol)."""
        for _, (dxl, adapter, ph) in self.devices.items():
            adapter.write_torque(self.port_handler, ph, dxl.servo_id, True)

    def disable_all_torque(self):
        """Disable torque for all attached devices (per protocol)."""
        for _, (dxl, adapter, ph) in self.devices.items():
            adapter.write_torque(self.port_handler, ph, dxl.servo_id, False)

    def set_torque(self, servo_id: int, enable: bool):
        """Enable/disable torque for a single device."""
        if servo_id not in self.devices:
            raise KeyError(f"Servo ID {servo_id} not attached")
        dxl, adapter, ph = self.devices[servo_id]
        adapter.write_torque(self.port_handler, ph, dxl.servo_id, bool(enable))

    def set_operating_mode(self, servo_id: int, mode: int):
        """Configure operating mode when supported by the device."""
        if servo_id not in self.devices:
            raise KeyError(f"Servo ID {servo_id} not attached")
        dxl, adapter, ph = self.devices[servo_id]
        try:
            return adapter.set_operating_mode(self.port_handler, ph, servo_id, mode)
        except NotImplementedError:
            raise

    def send_positions(self, positions: dict):
        """Send goal positions individually (protocol-aware)."""
        for servo_id, angle in positions.items():
            dxl, adapter, ph = self.devices[servo_id]
            value = int(2048.0 * angle / 180.0)
            adapter.write_goal_position_value(self.port_handler, ph, servo_id, value)

    def read_positions(self):
        """Read present positions individually (protocol-aware)."""
        result = {}
        for servo_id, (_, adapter, ph) in self.devices.items():
            pos = adapter.read_present_position_value(self.port_handler, ph, servo_id)
            result[servo_id] = 180.0 * float(pos) / 2048.0
        return result

    def send_wheel_velocity(self, servo_id: int, rpm: float):
        dxl, adapter, ph = self.devices[servo_id]
        if hasattr(dxl, "set_goal_velocity"):
            dxl.set_goal_velocity(rpm)
        adapter.write_goal_velocity_rpm(self.port_handler, ph, servo_id, rpm)

    def read_wheel_velocity(self, servo_id: int) -> float:
        _, adapter, ph = self.devices[servo_id]
        return adapter.read_present_velocity_rpm(self.port_handler, ph, servo_id)

    def send_wheel_velocities_sync(self, velocities: dict):
        """Broadcast wheel velocities using GroupSyncWrite (per protocol)."""
        if not velocities:
            return

        groups = {}

        for servo_id, rpm in velocities.items():
            if servo_id not in self.devices:
                raise KeyError(f"Servo ID {servo_id} not attached")
            dxl, adapter, ph = self.devices[servo_id]
            key = (ph, adapter.addr_goal_velocity, adapter.len_goal_velocity)
            if key not in groups:
                groups[key] = (GroupSyncWrite(
                    self.port_handler,
                    ph,
                    adapter.addr_goal_velocity,
                    adapter.len_goal_velocity,
                ), ph)

            group, _ = groups[key]
            params = adapter.goal_velocity_to_bytes(rpm)

            if hasattr(dxl, "set_goal_velocity"):
                dxl.set_goal_velocity(rpm)

            if not group.addParam(servo_id, params):
                raise RuntimeError(f"Failed to add sync param for ID {servo_id}")

        for group, ph in groups.values():
            dxl_comm = group.txPacket()
            if dxl_comm != COMM_SUCCESS:
                print(f"[DXL][SyncWrite] txPacket error: {ph.getTxRxResult(dxl_comm)}")
            group.clearParam()
