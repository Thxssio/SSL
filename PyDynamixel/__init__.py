# -*- coding: utf-8 -*-

"""PyDynamixel V2 - TauraBots Edition.

This package provides small, pragmatic helpers to communicate with Dynamixel
actuators. It supports both Protocol 1.0 and Protocol 2.0 (e.g., MX-28, MX-64,
MX-106 and similar), and can auto-detect the protocol per device ID.

Notes
-----
- Author: Mauricio Moraes Godoy
- Date: 2025-09-15
"""

from .comm import DxlComm
from .base import BaseDynamixel
from .joint import Joint
from .wheel import Wheel
from . import constants
