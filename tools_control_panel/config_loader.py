import os
import yaml


def load_config(path: str) -> dict:
    path = os.path.abspath(os.path.expanduser(path))
    with open(path) as f:
        cfg = yaml.safe_load(f)
    _resolve(cfg, os.path.dirname(path))
    return cfg


def _resolve(cfg: dict, base: str):
    def r(p: str) -> str:
        p = os.path.expanduser(str(p))
        return p if os.path.isabs(p) else os.path.abspath(os.path.join(base, p))

    for key in ('map_dir', 'data_dir'):
        if key in cfg.get('paths', {}):
            cfg['paths'][key] = r(cfg['paths'][key])

    if 'sources' in cfg.get('upload', {}):
        cfg['upload']['sources'] = [r(s) for s in cfg['upload']['sources']]