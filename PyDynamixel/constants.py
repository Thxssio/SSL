# -*- coding: utf-8 -*-

"""Control table constants for MX series (Protocol 2.0).

Notes
-----
- Author: Mauricio Moraes Godoy
- Date: 2025-09-15
- Compatible with MX-28, MX-64, MX-106 and similar Protocol 2.0 devices.
"""

# Main addresses
ADDR_TORQUE_ENABLE       = 64
ADDR_LED                 = 65
ADDR_OPERATING_MODE      = 11
ADDR_GOAL_VELOCITY       = 104
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_VELOCITY    = 128
ADDR_PRESENT_POSITION    = 132

# Data lengths (bytes)
LEN_GOAL_VELOCITY        = 4
LEN_GOAL_POSITION        = 4
LEN_PRESENT_VELOCITY     = 4
LEN_PRESENT_POSITION     = 4
