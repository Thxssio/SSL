import argparse
import re
import subprocess
import threading
import time
import glob
from typing import Optional

# Imports locais (mesma pasta)
from ssl_robot import SSLRobot
from protocol import SerialCommandReader, UDPCommandServer


# ---------------------------
# Argumentos de linha de comando
# ---------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="Run SSL omni robot controller")

    # conexão com MOTORES (Dynamixel)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (motores)")
    parser.add_argument("--baudrate", type=int, default=57600, help="Baudrate (motores)")

    # geometria e parâmetros físicos
    parser.add_argument(
        "--wheel-ids", nargs=4, type=int, default=[1, 2, 3, 4],
        help="Four wheel IDs in the same order as angles lists"
    )
    parser.add_argument(
        "--wheel-angles", nargs=4, type=float,
        default=[30.0, 150.0, 225.0, 315.0],  # φ_i (posições) CCW a partir de +x
        help="Wheel MOUNTING angles φ_i in degrees (CCW from +x)"
    )
    parser.add_argument(
        "--wheel-dir", nargs=4, type=float, default=None,
        help="(Optional) Rolling directions θ_i in degrees (CCW from +x). "
             "If omitted, kinematics assumes θ_i = φ_i + 90°."
    )
    parser.add_argument("--wheel-radius", type=float, default=0.03, help="Wheel radius (m)")
    parser.add_argument("--robot-radius", type=float, default=0.09, help="Robot radius (m)")
    parser.add_argument("--max-rpm", type=float, default=60.0, help="Motor datasheet max RPM")
    parser.add_argument("--max-wheel-rpm", type=float, default=55.0, help="Absolute per-wheel RPM limit")
    parser.add_argument(
        "--motor-signs", nargs=4, type=int, default=[1, 1, 1, 1],
        help="Per-wheel sign (±1) to match motor wiring/order"
    )

    # modo de comando de velocidade
    parser.add_argument("--v-cmd", type=float, default=0.3,
                        help="Target linear speed magnitude for teleop (m/s)")
    try:
        parser.add_argument("--force-constant-vcmd", action=argparse.BooleanOptionalAction, default=True,
                            help="If true, normalize (vx,vy) to have |v|=v_cmd; else only limit by v_max")
    except Exception:
        parser.add_argument("--force-constant-vcmd", action="store_true", default=True,
                            help="If set, normalize (vx,vy) to have |v|=v_cmd; else only limit by v_max")

    # transformação de entrada (UI -> robô)
    parser.add_argument("--in-rotate-deg", type=float, default=180.0,
                        help="Rotate incoming (vx,vy) CCW in degrees before kinematics (UI→robot).")
    try:
        parser.add_argument("--in-flip-x", action=argparse.BooleanOptionalAction, default=False,
                            help="Flip X after rotation (UI→robot). Use --no-in-flip-x to disable.")
    except Exception:
        # mantém o mesmo default (=False) para compatibilidade
        parser.add_argument("--in-flip-x", dest="in_flip_x", action="store_true", default=False,
                            help="Flip X after rotation (UI→robot).")
        parser.add_argument("--no-in-flip-x", dest="in_flip_x", action="store_false",
                            help="Disable X flip after rotation (UI→robot).")

    try:
        parser.add_argument("--in-flip-y", action=argparse.BooleanOptionalAction, default=True,
                            help="Flip Y after rotation (UI→robot). Use --no-in-flip-y to disable.")
    except Exception:
        parser.add_argument("--in-flip-y", dest="in_flip_y", action="store_true", default=True,
                            help="Flip Y after rotation (UI→robot).")
        parser.add_argument("--no-in-flip-y", dest="in_flip_y", action="store_false",
                            help="Disable Y flip after rotation (UI→robot).")

    try:
        parser.add_argument("--in-swap-xy", action=argparse.BooleanOptionalAction, default=True,
                            help="Swap incoming vx↔vy before rotation (UI→robot).")
    except Exception:
        parser.add_argument("--in-swap-xy", dest="in_swap_xy", action="store_true", default=True,
                            help="Swap incoming vx↔vy before rotation (UI→robot).")
        parser.add_argument("--no-in-swap-xy", dest="in_swap_xy", action="store_false",
                            help="Disable vx↔vy swap before rotation (UI→robot).")

    # heading/field alignment
    parser.add_argument("--heading-offset-deg", type=float, default=0.0,
                        help="Offset (deg) added to any heading/theta from vision before transforms.")

    # protocolo de comandos (rádio/PC)
    parser.add_argument("--protocol", choices=["none", "udp", "serial"], default="serial", help="Command protocol")
    # UDP
    parser.add_argument("--udp-host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--udp-port", type=int, default=20011, help="UDP port")
    parser.add_argument("--udp-ack", action="store_true", help="Send ACK responses")
    parser.add_argument("--udp-log-values", action="store_true", default=False, help="Log UDP received values")
    # Serial (comandos)
    parser.add_argument("--serial-cmd-port", default="/dev/ttyACM0", help="Serial port for command input")
    parser.add_argument("--serial-cmd-baudrate", type=int, default=115200, help="Baudrate for command serial input")
    parser.add_argument("--serial-cmd-target-id", type=int, default=None, help="Only accept commands with this ID (optional)")
    parser.add_argument("--serial-poll-timeout", type=float, default=0.05, help="Serial read timeout (s)")
    parser.add_argument("--serial-idle-timeout", type=float, default=0.3, help="Zero command timeout when serial is idle (s)")
    parser.add_argument("--serial-log-invalid", action="store_true", help="Log invalid serial lines")
    try:
        parser.add_argument("--serial-log-values", action=argparse.BooleanOptionalAction, default=True,
                            help="Print parsed serial command lines")
    except Exception:
        parser.add_argument("--serial-log-values", dest="serial_log_values", action="store_true", default=True,
                            help="Print parsed serial command lines")
        parser.add_argument("--no-serial-log-values", dest="serial_log_values", action="store_false",
                            help="Disable serial command logging")

    # Kicker (GPIO por libgpiod)
    try:
        parser.add_argument("--kick-enable", action=argparse.BooleanOptionalAction, default=True,
                            help="Enable GPIO kick pulse output triggered by command 'kick' field.")
    except Exception:
        parser.add_argument("--kick-enable", dest="kick_enable", action="store_true", default=True,
                            help="Enable GPIO kick pulse output.")
        parser.add_argument("--no-kick-enable", dest="kick_enable", action="store_false",
                            help="Disable GPIO kick pulse output.")

    parser.add_argument("--kick-pin", type=int, default=26, help="BCM pin used for kick pulse output")
    parser.add_argument("--kick-pulse-ms", type=float, default=300.0, help="Kick pulse duration in milliseconds")
    parser.add_argument("--kick-threshold", type=float, default=0.5, help="Minimum kick value that triggers the pulse")

    # comando direto (protocol=none)
    parser.add_argument("--vx", type=float, default=0.0, help="vx (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="vy (m/s)")
    parser.add_argument("--vtheta", type=float, default=0.0, help="angular vel (rad/s)")
    parser.add_argument("--duration", type=float, default=3.0, help="run time (s)")
    parser.add_argument("--kick", type=float, default=0.0, help="kick value (0/1) for protocol=none quick test")

    # logs do robô
    try:
        parser.add_argument("--robot-verbose", action=argparse.BooleanOptionalAction, default=False,
                            help="Print robot command/feedback debug info")
    except Exception:
        parser.add_argument("--robot-verbose", dest="robot_verbose", action="store_true", default=False,
                            help="Print robot command/feedback debug info")
        parser.add_argument("--no-robot-verbose", dest="robot_verbose", action="store_false",
                            help="Silence robot command/feedback debug info")

    return parser.parse_args()


# ---------------------------
# Funções auxiliares GPIO (libgpiod) — KickPulseController
# ---------------------------
def find_bcm_chip_path() -> str:
    """
    Descobre o caminho absoluto do gpiochip Broadcom (Pi 3/4/5).
    Retorna sempre algo como '/dev/gpiochip0'.
    """
    try:
        out = subprocess.check_output(["gpiodetect"], text=True)
        for line in out.splitlines():
            # casa pi4 (pinctrl-bcm2835), pi5 (pinctrl-bcm2712), etc.
            if "pinctrl-bcm" in line:
                m = re.match(r"^\s*(gpiochip\d+)\b", line.strip())
                if m:
                    return f"/dev/{m.group(1)}"
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass

    # Fallback: primeiro /dev/gpiochip* existente; se não houver, /dev/gpiochip0
    chips = sorted(glob.glob("/dev/gpiochip*"))
    if chips:
        return chips[0]
    return "/dev/gpiochip0"


class KickPulseController:
    """Edge-triggered GPIO pulse generator for the kicker output (libgpiod v1/v2)."""

    def __init__(self, pin: int, pulse_ms: float):
        self.pin = int(pin)
        self.pulse_s = max(1e-4, float(pulse_ms) / 1000.0)
        self._lock = threading.Lock()
        self._last_active = False
        self._pulse_thread = None
        self._chip = None
        self._release = None
        self._set_high = None
        self._set_low = None
        self._init_gpio()

    def _init_gpio(self):
        try:
            import gpiod
        except ImportError as exc:
            raise RuntimeError("Módulo 'gpiod' não está instalado; não é possível acionar o kick.") from exc

        chip_path = find_bcm_chip_path()  # <-- agora sempre '/dev/gpiochipX'
        try:
            chip = gpiod.Chip(chip_path)
        except Exception as exc:
            raise RuntimeError(f"Falha ao abrir {chip_path}: {exc}") from exc

        is_v2 = hasattr(chip, "request_lines")
        try:
            if is_v2:
                # ---------- libgpiod v2 ----------
                line_mod = getattr(gpiod, "line", gpiod)  # enums no v2 ficam em gpiod.line
                LineSettings = getattr(gpiod, "LineSettings")
                settings = LineSettings(direction=line_mod.Direction.OUTPUT,
                                        output_value=line_mod.Value.INACTIVE)

                if hasattr(gpiod, "LineConfig"):
                    lc = gpiod.LineConfig()
                    lc.add_line_settings([self.pin], settings)
                    req = chip.request_lines(consumer="kick-pulse", config=lc)
                else:
                    # variante portátil: dict {offset: LineSettings}
                    req = chip.request_lines(consumer="kick-pulse", config={self.pin: settings})

                def set_high():
                    req.set_value(self.pin, line_mod.Value.ACTIVE)

                def set_low():
                    req.set_value(self.pin, line_mod.Value.INACTIVE)

                def release():
                    try:
                        req.release()
                    except Exception:
                        pass

            else:
                # ---------- libgpiod v1 ----------
                request_type = getattr(gpiod, "LINE_REQ_DIR_OUT", None) \
                               or getattr(gpiod, "LINE_REQUEST_DIRECTION_OUTPUT", None)
                if request_type is None:
                    raise RuntimeError("libgpiod v1 sem constante LINE_REQ_DIR_OUT.")
                line = chip.get_line(self.pin)
                line.request(consumer="kick-pulse", type=request_type, default_vals=[0])

                def set_high():
                    line.set_value(1)

                def set_low():
                    line.set_value(0)

                def release():
                    try:
                        line.set_value(0)
                    except Exception:
                        pass
                    try:
                        line.release()
                    except Exception:
                        pass

            set_low()
        except Exception:
            try:
                chip.close()
            except Exception:
                pass
            raise

        self._chip = chip
        self._set_high = set_high
        self._set_low = set_low
        self._release = release

    def handle(self, is_active: bool, raw: float = 0.0):
        """Chame com is_active=True para iniciar pulso; False é ignorado aqui.
        A lógica de borda 0→1 pode ser feita no servidor; aqui garantimos 1 pulso/thread."""
        with self._lock:
            if is_active:
                if not self._last_active:
                    self._last_active = True
                    self._start_pulse_locked()
            else:
                self._last_active = False

    def _start_pulse_locked(self):
        if self._pulse_thread is not None and self._pulse_thread.is_alive():
            return
        self._pulse_thread = threading.Thread(target=self._pulse_task, daemon=True)
        self._pulse_thread.start()

    def _pulse_task(self):
        try:
            if self._set_high is not None:
                self._set_high()
            time.sleep(self.pulse_s)
        except Exception as exc:
            print(f"[KICK] Erro ao gerar pulso: {exc}")
        finally:
            try:
                if self._set_low is not None:
                    self._set_low()
            except Exception:
                pass
            with self._lock:
                self._pulse_thread = None
                self._last_active = False

    def close(self):
        thread = None
        with self._lock:
            thread = self._pulse_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=self.pulse_s + 0.1)
        try:
            if self._set_low is not None:
                self._set_low()
        except Exception:
            pass
        if self._release is not None:
            try:
                self._release()
            except Exception:
                pass
        if self._chip is not None:
            try:
                self._chip.close()
            except Exception:
                pass


# ---------------------------
# Main
# ---------------------------
def main():
    args = parse_args()

    # Inicializa o robô (motores)
    robot = SSLRobot(
        wheel_ids=args.wheel_ids,
        wheel_angles_deg=args.wheel_angles,    # φ_i (posições)
        wheel_radius=args.wheel_radius,
        robot_radius=args.robot_radius,
        port=args.port,
        baudrate=args.baudrate,
        max_rpm=args.max_rpm,
        command_linear_speed=args.v_cmd,
        max_wheel_rpm=args.max_wheel_rpm,
        motor_signs=args.motor_signs,
        wheel_dir_deg=args.wheel_dir,          # θ_i (direções de rolagem)
        force_constant_vcmd=args.force_constant_vcmd,
        in_rotate_deg=args.in_rotate_deg,
        in_flip_x=args.in_flip_x,
        in_flip_y=args.in_flip_y,
        in_swap_xy=args.in_swap_xy,
        heading_offset_deg=args.heading_offset_deg,
        verbose=args.robot_verbose,
    )

    # Kick (GPIO) — só via comando do rádio
    kick_controller = None
    if args.kick_enable:
        try:
            kick_controller = KickPulseController(args.kick_pin, args.kick_pulse_ms)
            print(f"[KICK] GPIO{args.kick_pin} pronto ({args.kick_pulse_ms:.0f} ms).")
        except RuntimeError as exc:
            print(f"[KICK] Desativado: {exc}")
            kick_controller = None

    # garante torque ligado
    try:
        robot.comm.enable_all_torque()
    except Exception as e:
        print(f"[WARN] enable_all_torque falhou: {e}")

    # --- modos de operação ---
    try:
        if args.protocol == "udp":
            server = UDPCommandServer(
                robot,
                host=args.udp_host,
                port=args.udp_port,
                ack=args.udp_ack,
                idle_timeout=0.25,
                poll_interval=0.01,
                kick_handler=kick_controller.handle if kick_controller else None,
                kick_threshold=args.kick_threshold,
                target_id=args.serial_cmd_target_id,  # reuso: filtra 'id' se quiser
                log_values=args.udp_log_values,
            )
            try:
                server.start()
                print(f"[UDP] listening on {args.udp_host}:{args.udp_port}")
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                server.stop()

        elif args.protocol == "serial":
            reader = SerialCommandReader(
                robot,
                port=args.serial_cmd_port,
                baudrate=args.serial_cmd_baudrate,
                target_id=args.serial_cmd_target_id,
                poll_timeout=args.serial_poll_timeout,
                idle_timeout=args.serial_idle_timeout,
                log_invalid=args.serial_log_invalid,
                kick_handler=kick_controller.handle if kick_controller else None,
                kick_threshold=args.kick_threshold,
                log_values=args.serial_log_values,
            )
            try:
                print(f"[SERIAL] Lendo de {args.serial_cmd_port} @ {args.serial_cmd_baudrate} baud")
                reader.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                reader.stop()

        else:  # protocol == "none" — teste rápido
            try:
                robot.set_velocity(args.vx, args.vy, args.vtheta)
                # aciona kick se pedido
                if kick_controller and args.kick >= args.kick_threshold:
                    kick_controller.handle(True, raw=args.kick)
                time.sleep(args.duration)
            finally:
                robot.stop()

    finally:
        # shutdown ordenado
        robot.stop()
        if kick_controller is not None:
            kick_controller.close()


if __name__ == "__main__":
    main()
