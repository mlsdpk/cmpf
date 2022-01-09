# Dictionary with information about all registered plugins
_PLUGINS = {}


def register(func):
    """Decorator for registering a new plugin"""
    _PLUGINS[func.__module__] = func
    return func
