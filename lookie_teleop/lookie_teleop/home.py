"""Home completion tracking from kstone ``~/state`` (design §11). The robot does the motion."""

import re

_REQUEST_ID = re.compile(r"request_id=(\d+)")


def parse_request_id(message: str) -> int | None:
    m = _REQUEST_ID.search(message or "")
    return int(m.group(1)) if m else None


class HomeTracker:
    """Per module: the outcome must appear with applied=true within ``outcome_timeout_s``, then
    ``home_state`` (only from snapshots that already contain that outcome) must reach succeeded
    with ``held_by`` null.
    """

    def __init__(self, modules: list, started: float, timeout_s: float, outcome_timeout_s: float):
        self.modules = list(modules)
        self.started = started
        self.timeout_s = timeout_s
        self.outcome_timeout_s = outcome_timeout_s
        self.request = {}        # module -> (request_id, sent time)
        self.outcome_seen = set()
        self.done = set()
        self.failure = None

    def accepted(self, module: str, request_id: int | None, now: float) -> None:
        if request_id is None:
            self.fail(f"{module} Home 回复无 request_id")
        else:
            self.request[module] = (request_id, now)

    def fail(self, reason: str) -> None:
        if self.failure is None:
            self.failure = reason

    def update(self, state: dict | None, now: float) -> str | None:
        """Returns ``"succeeded"``, ``"failed"`` or None while in progress."""
        if self.failure:
            return "failed"
        if state is not None:
            outcomes = {o.get("request_id"): o for o in state.get("outcomes", [])}
            mods = state.get("modules", {})
            for module, (rid, _) in self.request.items():
                if module in self.done:
                    continue
                outcome = outcomes.get(rid) or outcomes.get(float(rid))
                if module not in self.outcome_seen:
                    if outcome is None:
                        continue
                    if not outcome.get("applied", False):
                        self.fail(f"{module} Home 未受理: {outcome.get('error', '')}")
                        return "failed"
                    self.outcome_seen.add(module)
                m = mods.get(module, {})
                home_state = m.get("home_state")
                if home_state in ("timed_out", "aborted"):
                    self.fail(f"{module} Home {home_state}")
                    return "failed"
                if home_state == "succeeded" and not m.get("held_by"):
                    self.done.add(module)
        for module, (_, sent) in self.request.items():
            if module not in self.outcome_seen and now - sent > self.outcome_timeout_s:
                self.fail(f"{module} Home 结果查询不到")
                return "failed"
        if now - self.started > self.timeout_s:
            self.fail("Home 超时")
            return "failed"
        if len(self.done) == len(self.modules):
            return "succeeded"
        return None
