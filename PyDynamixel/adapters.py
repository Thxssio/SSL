# -*- coding: utf-8 -*-

from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)


class ProtocolAdapterBase:
    def __init__(self, version: float):
        self.version = version

    # ----- Torque -----
    @property
    def addr_torque_enable(self) -> int:
        raise NotImplementedError

    def write_torque(self, port_handler, packet_handler, servo_id: int, enable: bool):
        packet_handler.write1ByteTxRx(
            port_handler, servo_id, self.addr_torque_enable, 1 if enable else 0
        )

    # ----- Velocity (wheel) -----
    def write_goal_velocity_rpm(self, port_handler, packet_handler, servo_id: int, rpm: float):
        raise NotImplementedError

    def read_present_velocity_rpm(self, port_handler, packet_handler, servo_id: int) -> float:
        raise NotImplementedError

    @property
    def addr_goal_velocity(self) -> int:
        raise NotImplementedError

    @property
    def len_goal_velocity(self) -> int:
        raise NotImplementedError

    def goal_velocity_to_bytes(self, rpm: float):
        raise NotImplementedError

    def set_operating_mode(self, port_handler, packet_handler, servo_id: int, mode: int):
        raise NotImplementedError

    # ----- Position (joint) -----
    @property
    def addr_goal_position(self) -> int:
        raise NotImplementedError

    @property
    def addr_present_position(self) -> int:
        raise NotImplementedError

    def write_goal_position_value(self, port_handler, packet_handler, servo_id: int, value: int):
        raise NotImplementedError

    def read_present_position_value(self, port_handler, packet_handler, servo_id: int) -> int:
        raise NotImplementedError


class ProtocolV2Adapter(ProtocolAdapterBase):
    def __init__(self):
        super().__init__(2.0)

    @property
    def addr_torque_enable(self) -> int:
        return 64

    @property
    def addr_goal_position(self) -> int:
        return 116

    @property
    def addr_present_position(self) -> int:
        return 132

    def write_goal_velocity_rpm(self, port_handler, packet_handler, servo_id: int, rpm: float):
        units = int(round(rpm / 0.229))
        if units < 0:
            units = (units + (1 << 32)) & 0xFFFFFFFF
        dxl_comm, dxl_error = packet_handler.write4ByteTxRx(
            port_handler, servo_id, 104, units
        )
        return dxl_comm, dxl_error

    def read_present_velocity_rpm(self, port_handler, packet_handler, servo_id: int) -> float:
        value, dxl_comm, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, servo_id, 128
        )
        if dxl_comm != COMM_SUCCESS or dxl_error:
            return 0.0
        raw = int(value)
        if raw & (1 << 31):
            raw -= (1 << 32)
        return float(raw) * 0.229

    @property
    def addr_goal_velocity(self) -> int:
        return 104

    @property
    def len_goal_velocity(self) -> int:
        return 4

    def goal_velocity_to_bytes(self, rpm: float):
        units = int(round(rpm / 0.229))
        units = max(min(units, (1 << 31) - 1), -(1 << 31))
        if units < 0:
            units = (units + (1 << 32)) & 0xFFFFFFFF
        return [
            DXL_LOBYTE(DXL_LOWORD(units)),
            DXL_HIBYTE(DXL_LOWORD(units)),
            DXL_LOBYTE(DXL_HIWORD(units)),
            DXL_HIBYTE(DXL_HIWORD(units)),
        ]

    def set_operating_mode(self, port_handler, packet_handler, servo_id: int, mode: int):
        return packet_handler.write1ByteTxRx(port_handler, servo_id, 11, int(mode))

    def write_goal_position_value(self, port_handler, packet_handler, servo_id: int, value: int):
        return packet_handler.write4ByteTxRx(port_handler, servo_id, self.addr_goal_position, value)

    def read_present_position_value(self, port_handler, packet_handler, servo_id: int) -> int:
        value, dxl_comm, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, servo_id, self.addr_present_position
        )
        return value


class ProtocolV1Adapter(ProtocolAdapterBase):
    def __init__(self):
        super().__init__(1.0)

    @property
    def addr_torque_enable(self) -> int:
        return 24

    @property
    def addr_goal_position(self) -> int:
        return 30

    @property
    def addr_present_position(self) -> int:
        return 36

    def write_goal_velocity_rpm(self, port_handler, packet_handler, servo_id: int, rpm: float):
        # MX Protocol 1.0 Moving Speed (address 32, 2 bytes)
        # 0-1023 CCW, 1024-2047 CW; unit ~ 0.114 rpm
        mag = int(round(abs(rpm) / 0.114))
        if mag > 1023:
            mag = 1023
        raw = mag if rpm >= 0 else (mag + 1024)
        dxl_comm, dxl_error = packet_handler.write2ByteTxRx(
            port_handler, servo_id, 32, raw
        )
        return dxl_comm, dxl_error

    def read_present_velocity_rpm(self, port_handler, packet_handler, servo_id: int) -> float:
        # Present Speed (address 38, 2 bytes), same sign convention
        value, dxl_comm, dxl_error = packet_handler.read2ByteTxRx(
            port_handler, servo_id, 38
        )
        if dxl_comm != COMM_SUCCESS or dxl_error:
            return 0.0
        raw = int(value)
        if raw > 1023:
            return -float(raw - 1024) * 0.114
        else:
            return float(raw) * 0.114

    @property
    def addr_goal_velocity(self) -> int:
        return 32

    @property
    def len_goal_velocity(self) -> int:
        return 2

    def goal_velocity_to_bytes(self, rpm: float):
        mag = int(round(abs(rpm) / 0.114))
        if mag > 1023:
            mag = 1023
        raw = mag if rpm >= 0 else (mag + 1024)
        return [DXL_LOBYTE(raw), DXL_HIBYTE(raw)]

    def set_operating_mode(self, port_handler, packet_handler, servo_id: int, mode: int):
        raise NotImplementedError("Operating mode control is unavailable on Protocol 1.0 devices")

    def write_goal_position_value(self, port_handler, packet_handler, servo_id: int, value: int):
        # 2 bytes (0-4095)
        value = max(0, min(4095, int(value)))
        return packet_handler.write2ByteTxRx(port_handler, servo_id, self.addr_goal_position, value)

    def read_present_position_value(self, port_handler, packet_handler, servo_id: int) -> int:
        value, dxl_comm, dxl_error = packet_handler.read2ByteTxRx(
            port_handler, servo_id, self.addr_present_position
        )
        return value

