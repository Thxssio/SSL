# -*- coding: utf-8 -*-
import numpy as np


class OmniKinematics:
    """
    Cinemática para base omni de 4 rodas (forma geral).

    Parâmetros
    ----------
    wheel_pos_deg : list[float]
        Ângulos de MONTAGEM (posição) φ_i em graus (CCW a partir de +x).
    wheel_radius : float
        Raio da roda (m).
    robot_radius : float
        Distância do centro do robô ao eixo da roda (m).
    wheel_dir_deg : list[float] | None
        Direções de rolagem θ_i (em graus). Se None, assume rodas tangenciais: θ_i = φ_i + 90°.

    Modelo (i-ésima roda)
    ----------------------
      ω_i = (1/r) * ( cos θ_i * v_x + sin θ_i * v_y + L * sin(θ_i - φ_i) * ω )
    """

    def __init__(self, wheel_pos_deg, wheel_radius=0.03, robot_radius=0.09, wheel_dir_deg=None):
        self.r = float(wheel_radius)
        self.L = float(robot_radius)

        self.phi = np.radians(np.asarray(wheel_pos_deg, dtype=float))  # posições φ
        assert self.phi.shape == (4,), "Este módulo espera exatamente 4 rodas."

        if wheel_dir_deg is None:
            # Assume tangential rolling direction (θ_i = φ_i + 90°)
            self.theta = self.phi + np.pi / 2.0
        else:
            self.theta = np.radians(np.asarray(wheel_dir_deg, dtype=float))
            assert self.theta.shape == (4,), "wheel_dir_deg deve ter 4 ângulos."

        # Matriz T (Robot -> Wheels)
        self.T = np.zeros((4, 3), dtype=float)
        for i, (th, ph) in enumerate(zip(self.theta, self.phi)):
            self.T[i, :] = [np.cos(th), np.sin(th), self.L * np.sin(th - ph)]

        # Posto deve ser 3 (controlável em vx, vy, ω)
        rank = np.linalg.matrix_rank(self.T)
        if rank != 3:
            raise ValueError(f"Posto(T) = {rank} (esperado 3). Revise φ/θ.")

    def robot_to_wheels(self, vx, vy, omega):
        v = np.array([vx, vy, omega], dtype=float)
        return (1.0 / self.r) * (self.T @ v)  # rad/s

    def wheels_to_robot(self, wheel_speeds):
        T_pinv = np.linalg.pinv(self.T)
        v = self.r * (T_pinv @ np.asarray(wheel_speeds, dtype=float))
        return float(v[0]), float(v[1]), float(v[2])
