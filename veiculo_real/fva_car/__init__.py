from .pid import PID, MANUAL, AUTOMATIC, DIRECT, REVERSE, P_ON_M, P_ON_E

__all__ = ["Car", "PID", "MANUAL", "AUTOMATIC", "DIRECT", "REVERSE", "P_ON_M", "P_ON_E"]


def __getattr__(name):
    # Permite usar o PID sem carregar dependências de simulador/hardware.
    if name == "Car":
        from .car import Car
        globals()[name] = Car
        return Car
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
