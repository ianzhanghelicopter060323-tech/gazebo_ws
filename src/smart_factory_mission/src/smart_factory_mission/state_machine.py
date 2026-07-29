"""Small state transition helper for the incremental mission pipeline."""

from smart_factory_mission import states


class MissionStateMachine:
    def __init__(self, context, state_callback):
        self.context = context
        self._state_callback = state_callback

    def transition(self, new_state, detail):
        if new_state not in states.NAMES:
            raise ValueError("unknown mission state: {}".format(new_state))
        self.context.current_stage = new_state
        self._state_callback(self.context, detail)
