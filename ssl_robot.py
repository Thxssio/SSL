# -*- coding: utf-8 -*-
"""SSL omni-directional robot controller (com auto-calibrate + persistência JSON)."""
import json
import time
from pathlib import Path
import numpy as np
from PyDynamixel import DxlComm, Wheel
from kinematics import OmniKinematics


class SSLRobot:
    def __init__(
        self,
        wheel_ids,
        wheel_angles_deg,              # φ_i (posições) em graus
        wheel_radius=0.03,
        robot_radius=0.09,
        port="/dev/ttyUSB0",
        baudrate=57600,
        max_rpm=60.0,
        command_linear_speed=0.3,
        max_wheel_rpm=55.0,
        motor_signs=None,              # ±1 por roda, ordem coerente com wheel_ids
        wheel_dir_deg=None,            # θ_i (rolagem) em graus; se None assume tangencial
        force_constant_vcmd=True,
        # Transformação de entrada (UI -> robô)
        in_rotate_deg=-90.0,
        in_flip_x=False,
        in_flip_y=True,
        in_swap_xy=True,
        heading_offset_deg=0.0,
        verbose=True,
    ):
        assert len(wheel_ids) == 4, "São esperadas 4 rodas."
        self.comm = DxlComm(port, baudrate)
        self.kin = OmniKinematics(
            wheel_pos_deg=wheel_angles_deg,
            wheel_radius=wheel_radius,
            robot_radius=robot_radius,
            wheel_dir_deg=wheel_dir_deg
        )

        # limites físicos
        self.r = float(wheel_radius)
        self.max_rps = float(max_rpm) / 60.0
        self.max_w = 2.0 * np.pi * self.max_rps
        self.v_max = self.max_w * self.r
        self.v_cmd = float(command_linear_speed)
        self.max_wheel_rpm = float(max_wheel_rpm)
        self.force_constant_vcmd = bool(force_constant_vcmd)

        # sinais/ganhos por roda e viés de rotação
        self.motor_signs = np.asarray(motor_signs if motor_signs is not None else [1, 1, 1, 1], dtype=float)
        assert self.motor_signs.shape == (4,), "motor_signs deve ter 4 elementos (±1)."
        self.motor_gains = np.ones(4, dtype=float)   # ganho ≈1.0 (corrige roda “forte/fraca”)
        self.omega_bias = 0.0                         # viés de rotação (rad/s) para ir reto

        self.verbose = bool(verbose)

        # rodas
        self.wheels = []
        for wid in wheel_ids:
            w = Wheel(wid)
            self.comm.attach(w)
            self.wheels.append(w)

        self._configure_wheel_operating_mode()

        # heading
        self.theta = 0.0
        self.theta_target = None
        self.k_theta = 2.0

        # transformação de entrada (UI -> robô)
        self.in_rotate_rad = float(in_rotate_deg) * np.pi / 180.0
        self.in_flip_x = bool(in_flip_x)
        self.in_flip_y = bool(in_flip_y)
        self.in_swap_xy = bool(in_swap_xy)
        self.heading_offset = float(heading_offset_deg) * np.pi / 180.0

    # -------- utils
    @staticmethod
    def _radps_to_rpm(radps): return radps * (60.0 / (2.0 * np.pi))
    @staticmethod
    def _rpm_to_radps(rpm):   return rpm   * (2.0 * np.pi / 60.0)

    def _apply_input_transform(self, vx, vy):
        vx, vy = float(vx), float(vy)
        # swap (antes da rotação)
        if self.in_swap_xy:
            vx, vy = vy, vx
        # rotação CCW
        if abs(self.in_rotate_rad) > 1e-12:
            c, s = np.cos(self.in_rotate_rad), np.sin(self.in_rotate_rad)
            vx, vy = (c*vx - s*vy), (s*vx + c*vy)
        # flips
        if self.in_flip_x:
            vx = -vx
        if self.in_flip_y:
            vy = -vy
        return vx, vy

    # -------- persistência (JSON) --------
    def _geom_degrees(self):
        # devolve φ, θ em graus (listas)
        phi_deg = (np.degrees(self.kin.phi) % 360.0).tolist()
        theta_deg = (np.degrees(self.kin.theta) % 360.0).tolist()
        return [float(x) for x in phi_deg], [float(x) for x in theta_deg]

    def to_state_dict(self):
        phi_deg, theta_deg = self._geom_degrees()
        return {
            "version": 1,
            "geometry": {
                "wheel_pos_deg": phi_deg,
                "wheel_dir_deg": theta_deg,
            },
            "calibration": {
                "motor_signs": self.motor_signs.astype(int).tolist(),
                "motor_gains": self.motor_gains.tolist(),
                "omega_bias": float(self.omega_bias),
            },
            "input_transform": {
                "in_rotate_deg": float(np.degrees(self.in_rotate_rad)),
                "in_flip_x": bool(self.in_flip_x),
                "in_flip_y": bool(self.in_flip_y),
                "in_swap_xy": bool(self.in_swap_xy),
                "heading_offset_deg": float(np.degrees(self.heading_offset)),
            },
        }

    def _configure_wheel_operating_mode(self):
        """Ensure wheels operate in velocity mode when supported."""
        for w in self.wheels:
            try:
                self.comm.set_torque(w.servo_id, False)
            except Exception as e:
                print(f"[WARN] id={getattr(w,'servo_id','?')} disable_torque falhou antes do set_mode: {e}")

        for w in self.wheels:
            try:
                self.comm.set_operating_mode(w.servo_id, 1)
            except NotImplementedError:
                print(f"[INFO] id={getattr(w,'servo_id','?')} não suporta troca de operating mode; mantendo atual")
            except Exception as e:
                print(f"[WARN] id={getattr(w,'servo_id','?')} set_operating_mode falhou: {e}")

        for w in self.wheels:
            try:
                self.comm.set_torque(w.servo_id, True)
            except Exception as e:
                print(f"[WARN] id={getattr(w,'servo_id','?')} enable_torque falhou após set_mode: {e}")

    def apply_state_dict(self, state: dict, strict_geometry_check: bool = False):
        try:
            geom = state.get("geometry", {})
            cal = state.get("calibration", {})
            trf = state.get("input_transform", {})

            # Checagem de geometria (avisa se não bate)
            cur_phi, cur_theta = self._geom_degrees()
            saved_phi = geom.get("wheel_pos_deg")
            saved_theta = geom.get("wheel_dir_deg")
            def _almost_equal(a, b, tol=1e-2):
                return all(abs(((ai - bi + 180) % 360) - 180) <= tol for ai, bi in zip(a, b))
            if saved_phi and not _almost_equal(saved_phi, cur_phi):
                msg = f"[STATE] Aviso: wheel_pos_deg do estado difere do atual.\n  atual={cur_phi}\n  salvo={saved_phi}"
                if strict_geometry_check:
                    raise ValueError(msg)
                else:
                    print(msg)
            if saved_theta and not _almost_equal(saved_theta, cur_theta):
                msg = f"[STATE] Aviso: wheel_dir_deg do estado difere do atual.\n  atual={cur_theta}\n  salvo={saved_theta}"
                if strict_geometry_check:
                    raise ValueError(msg)
                else:
                    print(msg)

            # Aplica calibração (se existir)
            if "motor_signs" in cal:
                ms = np.asarray(cal["motor_signs"], dtype=float)
                if ms.shape == (4,):
                    self.motor_signs = ms
            if "motor_gains" in cal:
                mg = np.asarray(cal["motor_gains"], dtype=float)
                if mg.shape == (4,):
                    self.motor_gains = mg
            if "omega_bias" in cal:
                self.omega_bias = float(cal["omega_bias"])

            # Aplica transformação de entrada
            if "in_rotate_deg" in trf:
                self.in_rotate_rad = float(trf["in_rotate_deg"]) * np.pi / 180.0
            if "in_flip_x" in trf:
                self.in_flip_x = bool(trf["in_flip_x"])
            if "in_flip_y" in trf:
                self.in_flip_y = bool(trf["in_flip_y"])
            if "in_swap_xy" in trf:
                self.in_swap_xy = bool(trf["in_swap_xy"])
            if "heading_offset_deg" in trf:
                self.heading_offset = float(trf["heading_offset_deg"]) * np.pi / 180.0

            print("[STATE] Estado aplicado.")
        except Exception as e:
            print(f"[STATE] Falha ao aplicar estado: {e}")

    def save_state(self, path: str | Path):
        try:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as f:
                json.dump(self.to_state_dict(), f, ensure_ascii=False, indent=2)
            print(f"[STATE] Salvo em: {path}")
        except Exception as e:
            print(f"[STATE] Erro ao salvar estado: {e}")

    def load_state(self, path: str | Path, strict_geometry_check: bool = False):
        path = Path(path)
        if not path.exists():
            print(f"[STATE] Arquivo não encontrado, ignorando: {path}")
            return False
        try:
            with path.open("r", encoding="utf-8") as f:
                state = json.load(f)
            self.apply_state_dict(state, strict_geometry_check=strict_geometry_check)
            print(f"[STATE] Carregado de: {path}")
            return True
        except Exception as e:
            print(f"[STATE] Erro ao carregar estado: {e}")
            return False

    # -------- api
    def set_velocity(self, vx, vy, vtheta=0.0, heading=None):
        # transforma UI -> robô (corrige mapeamento de eixos)
        vx, vy = self._apply_input_transform(vx, vy)

        try:
            self.comm.enable_all_torque()
        except Exception as e:
            print(f"[WARN] enable_all_torque falhou: {e}")

        # normalização/limite de (vx,vy)
        norm = float(np.hypot(vx, vy))
        if self.force_constant_vcmd:
            if norm > 1e-6:
                speed = min(self.v_cmd, self.v_max)
                vx, vy = (vx / norm) * speed, (vy / norm) * speed
            else:
                vx, vy = 0.0, 0.0
        else:
            if norm > self.v_max:
                scale = self.v_max / norm
                vx, vy = vx * scale, vy * scale

        # campo -> robô (se heading fornecido)
        if heading is not None:
            heading = float(heading) + float(self.heading_offset)
            R = np.array([[np.cos(heading),  np.sin(heading)],
                          [-np.sin(heading), np.cos(heading)]], dtype=float)
            vx, vy = (R @ np.array([vx, vy], dtype=float)).tolist()

        # controle de heading (P)
        if self.theta_target is not None:
            err = (self.theta_target - self.theta + np.pi) % (2.0 * np.pi) - np.pi
            vtheta = self.k_theta * err

        # compensa viés de rotação residual (para ir reto)
        vtheta = float(vtheta) + float(self.omega_bias)

        # cinemática: (vx, vy, ω) -> ω rodas (rad/s)
        wheel_radps = self.kin.robot_to_wheels(vx, vy, vtheta)

        # modelo -> RPM físico (aplica sinais e ganhos por roda)
        wheel_rpm_model = self._radps_to_rpm(wheel_radps)
        wheel_rpm_cmd = (wheel_rpm_model * self.motor_signs) / self.motor_gains

        # saturação proporcional e clip
        max_abs = float(np.max(np.abs(wheel_rpm_cmd))) if wheel_rpm_cmd.size else 0.0
        if max_abs > self.max_wheel_rpm:
            scale = self.max_wheel_rpm / max_abs
            wheel_rpm_cmd *= scale
            if self.verbose:
                print(f"[SAT] max|rpm|={max_abs:.2f}>{self.max_wheel_rpm:.2f} => scale={scale:.3f}")

        clipped = np.clip(wheel_rpm_cmd, -self.max_wheel_rpm, self.max_wheel_rpm)
        if np.any(clipped != wheel_rpm_cmd):
            if self.verbose:
                print(f"[CLIP] aplicando limite rígido [±{self.max_wheel_rpm:.2f}]")
            wheel_rpm_cmd = clipped

        # debug
        try:
            ids = [w.servo_id for w in self.wheels]
        except Exception:
            ids = list(range(1, len(self.wheels) + 1))
        if self.verbose:
            print("[CMD] vx={:.3f} vy={:.3f} vth={:.3f} | {}".format(
                vx, vy, vtheta,
                ", ".join(f"id={sid}:cmd={float(rpm):.2f}rpm" for sid, rpm in zip(ids, wheel_rpm_cmd))
            ))

        # envia
        commands = {w.servo_id: float(rpm) for w, rpm in zip(self.wheels, wheel_rpm_cmd)}
        try:
            self.comm.send_wheel_velocities_sync(commands)
        except Exception as e:
            print(f"[WARN] SyncWrite falhou ({e}), usando fallback sequencial")
            for w, rpm in zip(self.wheels, wheel_rpm_cmd):
                try:
                    w.send_velocity(float(rpm))
                except Exception as e2:
                    print(f"[ERR] id={getattr(w,'servo_id','?')} send_velocity falhou: {e2}")

        # pequeno intervalo
        try:
            time.sleep(0.02)
        except Exception:
            pass

        # feedback
        if self.verbose:
            lines = []
            for i, (w, cmd_rpm, sign) in enumerate(zip(self.wheels, wheel_rpm_cmd, self.motor_signs)):
                try:
                    meas_phys = float(w.get_velocity())     # RPM físico
                    meas_model = meas_phys / float(sign)     # volta ao frame do modelo (não dividir por gain!)
                except Exception as e:
                    lines.append(f"id={getattr(w,'servo_id','?')}:meas=ERR({e})")
                    continue
                responded = (abs(meas_model) > max(1.0, 0.2 * abs(float(cmd_rpm)))) if abs(float(cmd_rpm)) >= 5.0 else abs(meas_model) < 3.0
                status = "OK" if responded else "NO-RESP"
                lines.append(f"id={getattr(w,'servo_id','?')}:cmd={float(cmd_rpm):.2f}rpm meas={meas_phys:.2f}rpm(model={meas_model:.2f}) [{status}]")
            print("[FBK] " + " | ".join(lines))

    def set_heading_target(self, theta_d):
        self.theta_target = None if theta_d is None else float(theta_d) + float(self.heading_offset)

    def update_heading(self, theta_meas):
        self.theta = float(theta_meas) + float(self.heading_offset)

    def stop(self):
        try:
            self.comm.enable_all_torque()
        except Exception as e:
            print(f"[WARN] enable_all_torque falhou no stop: {e}")
        zero_cmd = {w.servo_id: 0.0 for w in self.wheels}
        try:
            self.comm.send_wheel_velocities_sync(zero_cmd)
        except Exception as e:
            print(f"[WARN] SyncWrite stop falhou ({e}), usando fallback sequencial")
            for w in self.wheels:
                try:
                    w.send_velocity(0.0)
                except Exception as e2:
                    print(f"[ERR] id={getattr(w,'servo_id','?')} stop falhou: {e2}")

    def read_feedback(self):
        meas_rpm_phys = np.array([w.get_velocity() for w in self.wheels], dtype=float)
        meas_rpm_model = meas_rpm_phys / self.motor_signs
        meas_radps = self._rpm_to_radps(meas_rpm_model)
        return self.kin.wheels_to_robot(meas_radps)

    # -------------------- Calibrações --------------------

    def auto_calibrate(self, rpm_test=20.0, settle=0.4, avg_time=0.6):
        """
        Calibra sinais (±1) e ganhos por roda.
        - Gira cada roda sozinha a 'rpm_test' por 'settle' + 'avg_time' segundos.
        - Ajusta sinal se leitura vier invertida.
        - Define ganho = (|RPM_medida| / |rpm_test|). Ganho >1 indica roda “mais forte”.
        """
        print("[CAL] iniciando calibração por roda...")
        new_signs = self.motor_signs.copy()
        new_gains = np.ones(4, dtype=float)

        for i in range(4):
            # zera todas
            for w in self.wheels:
                try: w.send_velocity(0.0)
                except Exception: pass

            # comanda somente a roda i (no sinal atual)
            cmds = np.zeros(4, dtype=float)
            cmds[i] = float(rpm_test) * float(self.motor_signs[i])
            for w, rpm in zip(self.wheels, cmds):
                try: w.send_velocity(float(rpm))
                except Exception: pass

            time.sleep(float(settle))

            # média de leitura
            samples = []
            t0 = time.time()
            while time.time() - t0 < float(avg_time):
                try:
                    meas = float(self.wheels[i].get_velocity())
                    samples.append(meas)
                except Exception:
                    pass
                time.sleep(0.05)

            # zera todas novamente
            for w in self.wheels:
                try: w.send_velocity(0.0)
                except Exception: pass
            time.sleep(0.1)

            if not samples:
                print(f"[CAL] roda {i}: sem leitura, mantendo sinais/ganhos atuais.")
                continue

            m = float(np.mean(samples))
            # traz leitura para o frame do modelo removendo o sinal atual
            m_model = m / float(self.motor_signs[i])

            # se for negativa, sinal está invertido
            if m_model < 0:
                new_signs[i] *= -1.0
                m_model = -m_model

            gain = m_model / abs(float(rpm_test))
            new_gains[i] = gain if gain > 1e-3 else 1.0
            print(f"[CAL] roda {i}: sign={int(new_signs[i])}  gain≈{new_gains[i]:.3f}  (meas={m:.1f} rpm)")

        self.motor_signs = new_signs
        self.motor_gains = new_gains
        print(f"[CAL] concluída. signs={self.motor_signs.astype(int).tolist()} gains={self.motor_gains.round(3).tolist()}")

    def estimate_omega_bias(self, vx=0.25, vy=0.0, t_run=1.5):
        """
        Mede vtheta reconstruído enquanto comanda translação pura e define omega_bias para cancelá-lo.
        """
        print("[CAL] estimando omega_bias durante translação pura...")
        self.set_velocity(float(vx), float(vy), vtheta=0.0)
        time.sleep(0.3)
        vals = []
        t0 = time.time()
        while time.time() - t0 < float(t_run):
            try:
                _, _, w_m = self.read_feedback()
                vals.append(w_m)
            except Exception:
                pass
            time.sleep(0.05)
        self.stop()
        if vals:
            self.omega_bias = -float(np.median(vals))  # cancela o viés mediano
            print(f"[CAL] omega_bias ajustado para {self.omega_bias:.4f} rad/s")
        else:
            print("[CAL] não foi possível estimar omega_bias (sem leituras).")
