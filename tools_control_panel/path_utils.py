import os


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_configured_path(configured_path, fallback_relative_path):
    """Resolve a configured path, falling back to this repo when the config is stale.

    Many configs in this project were authored under ~/box/vln-large-scale-farm,
    but the runtime workspace may now live elsewhere. If the configured path does
    not exist, use a path anchored at the current repo root instead.
    """
    configured = os.path.expanduser(str(configured_path or "").strip())
    if configured and os.path.exists(configured):
        return configured
    return os.path.join(REPO_ROOT, fallback_relative_path)
