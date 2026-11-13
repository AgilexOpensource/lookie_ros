"""Button edge helpers (design §5): long / short press, grip hysteresis and grip double-tap-hold."""


class LongPress:
    """Fires once ``hold_s`` after the press edge; re-arms on release."""

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self._since = None
        self._fired = False

    def update(self, pressed: bool, now: float) -> bool:
        if not pressed:
            self._since, self._fired = None, False
            return False
        if self._since is None:
            self._since = now
        if not self._fired and now - self._since >= self.hold_s:
            self._fired = True
            return True
        return False

    def reset(self) -> None:
        # Swallow an in-progress hold so it cannot fire after a state change
        self._fired = True if self._since is not None else False


class ShortLongPress:
    """``"long"`` once ``hold_s`` after the press edge; ``"short"`` on release before that."""

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self._since = None
        self._fired = False

    def update(self, pressed: bool, now: float):
        if not pressed:
            short = self._since is not None and not self._fired
            self._since, self._fired = None, False
            return "short" if short else None
        if self._since is None:
            self._since = now
        if not self._fired and now - self._since >= self.hold_s:
            self._fired = True
            return "long"
        return None


class Hysteresis:
    def __init__(self, on: float, off: float):
        self.on, self.off = on, off
        self.state = False

    def update(self, value: float) -> bool:
        if self.state and value < self.off:
            self.state = False
        elif not self.state and value > self.on:
            self.state = True
        return self.state


class DoubleTapHold:
    """Press, release, then press again within ``window_s`` of the first press and keep holding.

    ``update`` returns ``"engage"`` on the second press edge and ``"release"`` when an engaged
    hold ends; otherwise None.
    """

    def __init__(self, window_s: float):
        self.window_s = window_s
        self._last_press = None
        self._prev = False
        self.engaged = False

    def update(self, pressed: bool, now: float):
        event = None
        if pressed and not self._prev:
            if self._last_press is not None and now - self._last_press <= self.window_s:
                self.engaged = True
                self._last_press = None
                event = "engage"
            else:
                self._last_press = now
        elif not pressed and self._prev and self.engaged:
            self.engaged = False
            event = "release"
        self._prev = pressed
        return event

    def reset(self) -> None:
        self._last_press = None
        self.engaged = False
