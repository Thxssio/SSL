#!/usr/bin/env python3
# ESPAÇO -> pulso PULSE_S no GPIO BCM 26; 'q' sai.
# Funciona com libgpiod v2 (com ou sem LineConfig) e com fallback para v1.
import time, curses, re, subprocess, sys

PIN = 26        # número BCM (offset no RPi)
PULSE_S = 0.300 # duração do pulso em segundos

def find_bcm_chip_path() -> str:
    try:
        out = subprocess.check_output(["gpiodetect"], text=True)
        for line in out.splitlines():
            if "pinctrl-bcm" in line:
                m = re.match(r'^(gpiochip\d+)\b', line.strip())
                if m: return f"/dev/{m.group(1)}"
    except Exception:
        pass
    return "/dev/gpiochip0"

def main(stdscr):
    import gpiod  # carrega a binding real instalada
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, f"ESPAÇO => pulso {int(PULSE_S*1000)} ms no GPIO{PIN} (BCM) | 'q' sai")
    stdscr.refresh()

    chip_path = find_bcm_chip_path()
    try:
        chip = gpiod.Chip(chip_path)
    except FileNotFoundError:
        stdscr.addstr(2, 0, f"ERRO: chip não encontrado em {chip_path}.")
        stdscr.refresh(); time.sleep(2); return
    except PermissionError:
        stdscr.addstr(2, 0, "ERRO: permissão negada em /dev/gpiochip*. Adicione-se ao grupo 'gpio' e relogue.")
        stdscr.refresh(); time.sleep(2); return

    is_v2 = hasattr(chip, "request_lines")  # v2 tem request_lines; v1 usa get_line

    try:
        if is_v2:
            # ---------- libgpiod v2 ----------
            line_mod = getattr(gpiod, "line", gpiod)  # enums moram em gpiod.line no v2
            LineSettings = getattr(gpiod, "LineSettings")  # deve existir no v2
            settings = LineSettings(direction=line_mod.Direction.OUTPUT,
                                    output_value=line_mod.Value.INACTIVE)

            if hasattr(gpiod, "LineConfig"):
                # Variante que usa LineConfig (se disponível)
                lc = gpiod.LineConfig()
                lc.add_line_settings([PIN], settings)
                req = chip.request_lines(consumer="gpio-pulse", config=lc)
            else:
                # Variante portátil: dict {offset: LineSettings}
                req = chip.request_lines(consumer="gpio-pulse", config={PIN: settings})

            def set_high(): req.set_value(PIN, line_mod.Value.ACTIVE)
            def set_low():  req.set_value(PIN, line_mod.Value.INACTIVE)
            def release():  req.release()

        else:
            # ---------- libgpiod v1 ----------
            line = chip.get_line(PIN)
            line.request(consumer="gpio-pulse", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            def set_high(): line.set_value(1)
            def set_low():  line.set_value(0)
            def release():  line.release()

        pulses = 0
        try:
            while True:
                ch = stdscr.getch()
                if ch == ord('q'):
                    break
                if ch == ord(' '):
                    set_high()
                    time.sleep(PULSE_S)
                    set_low()
                    pulses += 1
                    stdscr.addstr(1, 0, f"Pulsos enviados: {pulses}        ")
                    stdscr.refresh()
                time.sleep(0.01)
        finally:
            try: set_low()
            except Exception: pass
            try: release()
            except Exception: pass
            try: chip.close()
            except Exception: pass

    except PermissionError:
        stdscr.addstr(2, 0, "ERRO: permissão negada ao controlar a linha. Grupo 'gpio' e re-login ajudam.")
        stdscr.refresh(); time.sleep(2)
    except OSError as e:
        stdscr.addstr(2, 0, f"ERRO de OS: {e}")
        stdscr.refresh(); time.sleep(2)

if __name__ == "__main__":
    curses.wrapper(main)
