"""Factory for selecting the active balance controller from config."""

from LegoBalance.NonLinearController import NonLinearController
from LegoBalance.PidController import PidController


def BuildBalanceController(config):
    """Construct the configured balance controller.

    Supported values:
    - ``tanh`` / ``nonlinear``: composite-variable tanh controller
    - ``pid``: discrete PID controller
    """
    algorithm = str(getattr(config.controller, "algorithm", "tanh")).strip().lower()
    if algorithm in ("tanh", "nonlinear"):
        return NonLinearController(config)
    if algorithm == "pid":
        return PidController(config)
    raise ValueError(f"unsupported controller.algorithm: {algorithm}")
