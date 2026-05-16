"""
Local PC radio bridge — Windows version.
Serial port: COM* (set in farm_config.yaml → radio.serial_port)
Run: python radio_bridge_win.py [--config ../config/farm_config.yaml]
"""
import argparse
import os

from radio_bridge_linux import RadioBridge, load_config

_SCRIPT_DIR        = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CFG       = os.path.join(_SCRIPT_DIR, "../config/farm_config.yaml")
_DEFAULT_CFG_LOCAL = os.path.join(_SCRIPT_DIR, "../config/local_config.yaml")

_WIN_RADIO = {
    "serial_port": "COM8",
    "baud_rate":   57600,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=_DEFAULT_CFG)
    parser.add_argument("--local-config", default=_DEFAULT_CFG_LOCAL)
    args = parser.parse_args()
    cfg = load_config(args.config, args.local_config)
    cfg["radio"].update(_WIN_RADIO)
    RadioBridge(cfg).run()


if __name__ == "__main__":
    main()