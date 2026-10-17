from dataclasses import dataclass
from enum import IntEnum


class Mode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    FORCE_POS = 4
    ROBSTRIDE_POS_VEL_CSP = 5


@dataclass(frozen=True)
class MotorState:
    can_id: int
    arbitration_id: int
    status_code: int
    pos: float
    vel: float
    torq: float
    t_mos: float
    t_rotor: float
    # HighTorque: 表3 故障码(0=无故障); 其余厂商恒 0。
    fault_code: int = 0
