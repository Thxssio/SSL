# -*- coding: utf-8 -*-

import argparse
import json
import socket
import sys
from datetime import datetime


def parse_args():
    parser = argparse.ArgumentParser(description="Servidor UDP simples para teste de recepção")
    parser.add_argument("--host", default="0.0.0.0", help="Endereço de bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=20011, help="Porta UDP (default: 20011)")
    parser.add_argument("--ack", action="store_true", help="Enviar ACK ao remetente")
    parser.add_argument("--json", action="store_true", help="Tentar decodificar como JSON e imprimir formatado")
    return parser.parse_args()


def format_payload(data: bytes, try_json: bool) -> str:
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        text = str(data)
    if try_json:
        try:
            obj = json.loads(text)
            return json.dumps(obj, ensure_ascii=False, indent=2)
        except Exception:
            pass
    return text


def main():
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.host, args.port))
    except OSError as e:
        print(f"Falha ao bindar em {args.host}:{args.port} -> {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[UDP] Ouvindo em {args.host}:{args.port} (ACK={'on' if args.ack else 'off'}, JSON={'on' if args.json else 'off'})")

    try:
        while True:
            data, addr = sock.recvfrom(65535)
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            payload = format_payload(data, try_json=args.json)
            print(f"[{ts}] De {addr[0]}:{addr[1]} ({len(data)} bytes):\n{payload}")

            if args.ack:
                try:
                    ack_payload = json.dumps({
                        "ok": True,
                        "len": len(data),
                        "ts": ts,
                    }).encode("utf-8")
                except Exception:
                    ack_payload = b"{\"ok\":true}"
                try:
                    sock.sendto(ack_payload, addr)
                except Exception as e:
                    print(f"Erro ao enviar ACK: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()


