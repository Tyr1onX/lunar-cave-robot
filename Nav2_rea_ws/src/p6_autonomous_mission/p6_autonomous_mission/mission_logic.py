"""Small, ROS-independent mission state model used by the coordinator and tests."""

from enum import Enum


class MissionState(str, Enum):
    IDLE = 'IDLE'
    WAITING_FOR_STACK = 'WAITING_FOR_STACK'
    NAVIGATING_TO_TARGET = 'NAVIGATING_TO_TARGET'
    CONFIRMING_TARGET_STOP = 'CONFIRMING_TARGET_STOP'
    DETECTING_TARGET = 'DETECTING_TARGET'
    ARM_PICK = 'ARM_PICK'
    NAVIGATING_HOME = 'NAVIGATING_HOME'
    CONFIRMING_HOME_STOP = 'CONFIRMING_HOME_STOP'
    ARM_UNLOAD = 'ARM_UNLOAD'
    SUCCEEDED = 'SUCCEEDED'
    ABORTED = 'ABORTED'


TRANSITIONS = {
    MissionState.IDLE: {MissionState.WAITING_FOR_STACK},
    MissionState.WAITING_FOR_STACK: {MissionState.NAVIGATING_TO_TARGET},
    MissionState.NAVIGATING_TO_TARGET: {MissionState.CONFIRMING_TARGET_STOP},
    MissionState.CONFIRMING_TARGET_STOP: {MissionState.DETECTING_TARGET},
    MissionState.DETECTING_TARGET: {MissionState.ARM_PICK},
    MissionState.ARM_PICK: {MissionState.NAVIGATING_HOME},
    MissionState.NAVIGATING_HOME: {MissionState.CONFIRMING_HOME_STOP},
    MissionState.CONFIRMING_HOME_STOP: {MissionState.ARM_UNLOAD},
    MissionState.ARM_UNLOAD: {MissionState.SUCCEEDED},
    MissionState.SUCCEEDED: set(),
    MissionState.ABORTED: set(),
}


class MissionModel:
    """Enforce one-shot triggers and terminal failure/success states."""

    def __init__(self):
        self.state = MissionState.IDLE
        self.started = False
        self.history = [self.state]

    def accept_trigger(self):
        if self.started or self.state in (MissionState.SUCCEEDED, MissionState.ABORTED):
            return False
        self.started = True
        self.transition(MissionState.WAITING_FOR_STACK)
        return True

    def transition(self, next_state):
        if next_state == MissionState.ABORTED:
            if self.state in (MissionState.SUCCEEDED, MissionState.ABORTED):
                return False
        elif next_state not in TRANSITIONS[self.state]:
            raise ValueError('illegal mission transition: %s -> %s' % (self.state, next_state))
        self.state = next_state
        self.history.append(next_state)
        return True

