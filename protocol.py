# -*- coding: utf-8 -*-
"""Lightweight UDP/Serial command servers for SSL robot control.

Notes
-----
- Author: Mauricio Moraes Godoy
- Date: 2025-09-15 (atualizado)

Protocol (UDP)
--------------
JSON messages over UDP. Supported commands:

- {"cmd": "vel", "vx": float, "vy": float, "vtheta": float, "heading": float|null, "kick": 0|1}
- {"cmd": "drive", "vx": float, "vy": float, "vtheta": float,
   "theta": float|null, "theta_target": float|null, "kick": 0|1}
- {"cmd": "stop"}
- {"cmd": "heading", "theta": float}      # set desired heading target
- {"cmd": "imu", "theta": float}          # update measured heading
- {"cmd": "ping"}

Optional fields:
- "id": inteiro para filtro de destino do robô
- "k": alias para "kick" (0/1, bool, ou float; >= threshold dispara)

Responses (when ack=True):
- {"ok": true, "cmd": "<name>"}
"""

import errno
import json
import re
import select
import socket
import time
from typing import Callable, Optional, Tuple

import serial  # pyserial

from ssl_robot import SSLRobot


class UDPCommandServer:
    """Simple UDP server that decodes JSON commands and drives the robot.

    Parameters
    ----------
    robot : SSLRobot
        Robot controller instance.
    host : str, default "0.0.0.0"
        Bind address.
    port : int, default 20011
        UDP port to listen on.
    ack : bool, default False
        If True, send a small JSON acknowledgment back to the sender.
    idle_timeout : float, default 0.25
        Stop robot if no commands arrive within this time (seconds).
    poll_interval : float, default 0.01
        Select() loop poll interval (seconds).
    kick_handler : Optional[Callable[[bool, float], None]]
        Função chamada com (is_active, raw_value) quando houver campo 'kick'/'k' no pacote.
        Envie 0/1 via rádio. O handler deve tratar borda 0→1 se precisar pulso.
    kick_threshold : float, default 0.5
        Se 'kick' vier como float, valores >= threshold contam como ativo.
    target_id : Optional[int]
        Se definido, ignora pacotes cujo 'id' != target_id.
    log_values : bool, default False
        Logar valores recebidos em 'vel' e 'drive'.
    """

    def __init__(
        self,
        robot: SSLRobot,
        host: str = "0.0.0.0",
        port: int = 20011,
        ack: bool = False,
        idle_timeout: float = 0.25,
        poll_interval: float = 0.01,
        *,
        kick_handler: Optional[Callable[[bool, float], None]] = None,
        kick_threshold: float = 0.5,
        target_id: Optional[int] = None,
        log_values: bool = False,
    ):
        self.robot = robot
        self.host = host
        self.port = port
        self.ack = ack
        self.sock: Optional[socket.socket] = None
        self._running: bool = False
        self.idle_timeout = float(idle_timeout)
        self.poll_interval = max(1e-3, float(poll_interval))
        self._last_command_time = time.monotonic()
        self._last_zero_sent = False
        self._kick_handler = kick_handler
        self._kick_threshold = float(kick_threshold)
        self._target_id = target_id
        self._log_values = bool(log_values)

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)
        self._running = True
        self._last_command_time = time.monotonic()
        self._last_zero_sent = False

    def stop(self):
        self._running = False
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def serve_forever(self):
        if self.sock is None:
            self.start()
        assert self.sock is not None

        while self._running:
            ready, _, _ = select.select([self.sock], [], [], self.poll_interval)

            if ready:
                packet = self._drain_latest_packet()
                if packet is None:
                    continue
                _, handled, zero_command = self._handle_packet(*packet)
                if handled:
                    self._last_command_time = time.monotonic()
                    self._last_zero_sent = zero_command
            else:
                self._handle_idle_timeout()

    def _drain_latest_packet(self) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        assert self.sock is not None
        latest: Optional[Tuple[bytes, Tuple[str, int]]] = None
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                latest = (data, addr)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
        return latest

    def _handle_idle_timeout(self):
        if self.idle_timeout <= 0.0:
            return
        now = time.monotonic()
        if (now - self._last_command_time) < self.idle_timeout:
            return
        if self._last_zero_sent:
            return
        try:
            self.robot.set_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
        self._last_zero_sent = True
        self._last_command_time = now

    def _maybe_kick(self, payload: dict):
        if self._kick_handler is None:
            return
        raw = payload.get("kick", payload.get("k", 0))
        try:
            if isinstance(raw, bool):
                is_active = raw
                val = 1.0 if raw else 0.0
            else:
                val = float(raw)
                # interpreta 0/1 ou float com threshold
                try:
                    is_active = bool(int(round(val)))
                except Exception:
                    is_active = (val >= self._kick_threshold)
        except Exception:
            is_active, val = False, 0.0
        try:
            self._kick_handler(is_active, val)
        except TypeError:
            self._kick_handler(is_active)  # compat
        except Exception:
            pass

    def _handle_packet(self, data: bytes, addr: Tuple[str, int]):
        try:
            payload = json.loads(data.decode("utf-8"))
        except Exception:
            return False, False, self._last_zero_sent

        cmd = payload.get("cmd")
        ok = True
        handled = False
        zero_command = False

        # filtro por id (opcional)
        if self._target_id is not None and payload.get("id") != self._target_id:
            if self.ack and self.sock is not None:
                try:
                    self.sock.sendto(b'{"ok":false,"err":"id"}', addr)
                except Exception:
                    pass
            return False, False, self._last_zero_sent

        if cmd == "vel":
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            vtheta = float(payload.get("vtheta", 0.0))
            heading = payload.get("heading")
            heading = float(heading) if heading is not None else None
            self.robot.set_velocity(vx, vy, vtheta, heading=heading)
            self._maybe_kick(payload)
            handled = True
            zero_command = (abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vtheta) < 1e-6)

        elif cmd == "drive":
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            vtheta = float(payload.get("vtheta", 0.0))
            theta = payload.get("theta")
            theta_target = payload.get("theta_target")
            heading_arg = None
            if theta is not None:
                try:
                    theta = float(theta)
                    self.robot.update_heading(theta)
                    heading_arg = theta
                except Exception:
                    pass
            if theta_target is not None:
                try:
                    self.robot.set_heading_target(float(theta_target))
                except Exception:
                    pass
            # mantém teu ganho atual no vtheta:
            self.robot.set_velocity(vx, vy, vtheta * 5, heading=heading_arg)
            self._maybe_kick(payload)
            handled = True
            zero_command = (abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vtheta) < 1e-6)

        elif cmd == "stop":
            self.robot.stop()
            handled = True
            zero_command = True

        elif cmd == "heading":
            theta = float(payload.get("theta", 0.0))
            self.robot.set_heading_target(theta)
            handled = True

        elif cmd == "imu":
            theta = float(payload.get("theta", 0.0))
            self.robot.update_heading(theta)
            handled = True

        elif cmd == "ping":
            handled = True

        else:
            ok = False

        if self._log_values and handled and cmd in ("vel", "drive"):
            try:
                kv = payload.get("kick", payload.get("k", 0))
                print(
                    f"[UDP {addr[0]}] cmd={cmd} "
                    f"vx={payload.get('vx',0):+.3f} vy={payload.get('vy',0):+.3f} "
                    f"w={payload.get('vtheta',0):+.3f} kick={kv}"
                )
            except Exception:
                pass

        if self.ack and self.sock is not None:
            try:
                resp = json.dumps({"ok": ok, "cmd": cmd}).encode("utf-8")
                self.sock.sendto(resp, addr)
            except Exception:
                pass

        return ok, handled, zero_command


class SerialCommandReader:
    """Read velocity commands from a serial device.

    Expected line format (ASCII):
        id,vx,vy,vtheta[,kick[,theta,theta_target]]
    where the fields are separated by commas and optional whitespace.
    All numeric fields are parsed as floats except ``id`` which is parsed as int.
    The ``theta`` (measured heading) and ``theta_target`` fields are optional; when present
    they update the robot heading/target. The ``kick`` field is also optional and is
    forwarded to an optional handler as a float/bool.
    """

    _LINE_REGEX = re.compile(r"[,\s]+")

    def __init__(
        self,
        robot: SSLRobot,
        port: str,
        baudrate: int = 115200,
        *,
        target_id: Optional[int] = None,
        poll_timeout: float = 0.05,
        idle_timeout: float = 0.25,
        log_invalid: bool = False,
        kick_handler: Optional[Callable[[bool, float], None]] = None,
        kick_threshold: float = 0.5,
        log_values: bool = True,
    ):
        self.robot = robot
        self.port = port
        self.baudrate = int(baudrate)
        self.target_id = target_id
        self.poll_timeout = max(1e-3, float(poll_timeout))
        self.idle_timeout = max(0.0, float(idle_timeout))
        self.log_invalid = bool(log_invalid)
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._last_command_time = time.monotonic()
        self._last_zero_sent = False
        self._buffer = bytearray()
        self._kick_handler = kick_handler
        self._kick_threshold = float(kick_threshold)
        self._log_values = bool(log_values)

    def start(self):
        if self._serial is not None:
            return
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.poll_timeout,
            )
            self._serial.reset_input_buffer()
            self._buffer.clear()
        except serial.SerialException as exc:
            raise RuntimeError(f"Não foi possível abrir a serial '{self.port}': {exc}") from exc
        self._running = True
        self._last_command_time = time.monotonic()
        self._last_zero_sent = False

    def stop(self):
        self._running = False
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def serve_forever(self):
        if self._serial is None:
            self.start()
        assert self._serial is not None

        try:
            while self._running:
                try:
                    to_read = self._serial.in_waiting or 1
                    chunk = self._serial.read(to_read)
                except serial.SerialException as exc:
                    print(f"[SERIAL] Erro de leitura: {exc}")
                    break

                if not chunk:
                    self._handle_idle_timeout()
                    continue

                self._buffer.extend(chunk)

                line = self._extract_latest_line()
                if line is None:
                    continue

                try:
                    cmd_id, vx, vy, vtheta, theta, theta_target, kick, text = self._parse_line(line)
                except ValueError as exc:
                    if self.log_invalid:
                        print(f"[SERIAL] Linha inválida ignorada: {exc}")
                    continue

                if self.target_id is not None and cmd_id != self.target_id:
                    continue

                if theta is not None:
                    try:
                        self.robot.update_heading(theta)
                        heading_arg = theta
                    except Exception:
                        heading_arg = None
                else:
                    heading_arg = None

                if theta_target is not None:
                    try:
                        self.robot.set_heading_target(theta_target)
                    except Exception:
                        pass

                if self._log_values:
                    log_parts = [
                        f"id={cmd_id}",
                        f"vx={vx:.3f}",
                        f"vy={vy:.3f}",
                        f"vtheta={vtheta:.3f}",
                    ]
                    if theta is not None:
                        log_parts.append(f"theta={theta:.3f}")
                    if theta_target is not None:
                        log_parts.append(f"theta_target={theta_target:.3f}")
                    log_parts.append(f"kick={kick:.3f}")
                    print(f"[SERIAL] {' '.join(log_parts)} | raw='{text}'")

                self.robot.set_velocity(vx/1000, vy/1000, -vtheta/1000, heading=heading_arg)
                self._dispatch_kick(kick)

                self._last_command_time = time.monotonic()
                self._last_zero_sent = abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vtheta) < 1e-6
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _handle_idle_timeout(self):
        if self.idle_timeout <= 0.0:
            return
        now = time.monotonic()
        if (now - self._last_command_time) < self.idle_timeout:
            return
        if self._last_zero_sent:
            return
        try:
            self.robot.set_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
        self._last_zero_sent = True
        self._last_command_time = now

    def _extract_latest_line(self) -> Optional[bytes]:
        if b"\n" not in self._buffer:
            return None
        head, tail = self._buffer.rsplit(b"\n", 1)
        self._buffer = bytearray(tail)
        if not head:
            return b""
        # Keep only the most recent complete line and discard older ones.
        return head.split(b"\n")[-1]

    def _dispatch_kick(self, kick_value: float):
        if self._kick_handler is None:
            return
        try:
            is_active = bool(int(round(kick_value)))
        except (TypeError, ValueError):
            is_active = bool(kick_value >= self._kick_threshold)
        try:
            self._kick_handler(is_active, float(kick_value))
        except TypeError:
            self._kick_handler(is_active)
        except Exception as exc:
            if self.log_invalid:
                print(f"[SERIAL] Falha no handler do kick: {exc}")

    def _parse_line(self, raw: bytes):
        try:
            text = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            raise ValueError("falha ao decodificar bytes")

        if not text:
            raise ValueError("linha vazia")

        # Permite espaços ou vírgulas múltiplas entre campos
        parts = [p for p in self._LINE_REGEX.split(text) if p]
        if len(parts) < 4:
            raise ValueError(f"esperados >=4 campos, obtido {len(parts)}: '{text}'")

        try:
            cmd_id = int(float(parts[0]))
            vx = float(parts[1])
            vy = float(parts[2])
            vtheta = float(parts[3])
        except ValueError as exc:
            raise ValueError(f"não foi possível converter campos numéricos: '{text}'") from exc

        # Campos opcionais
        def _maybe_float(value):
            low = value.strip().lower()
            if low in ("", "none", "null", "nan"):
                return None
            try:
                return float(value)
            except ValueError as exc:
                raise ValueError(f"não foi possível converter '{value}' em float: '{text}'") from exc

        theta = None
        theta_target = None
        kick = 0.0

        if len(parts) >= 5:
            # Campo 5 é sempre o kick no novo formato; mantém compatibilidade com formato antigo.
            kick_val = _maybe_float(parts[4])
            kick = 0.0 if kick_val is None else kick_val
        if len(parts) >= 6:
            theta = _maybe_float(parts[5])
        if len(parts) >= 7:
            theta_target = _maybe_float(parts[6])

        return cmd_id, vx, vy, vtheta, theta, theta_target, kick, text
