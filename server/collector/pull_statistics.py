"""Count actual requests/outcomes, independently of packet identifiers."""
import threading
import time


class PullStatistics:
    def __init__(self):
        self.lock = threading.RLock()
        self.total = self._new()
        self.capture = None
        self.active = False
        self.epoch = 0
        self.outstanding = None

    def _new(self):
        return dict(attempts=0, successes=0, timeouts=0, errors=0, started=time.monotonic(), ended=None)

    def begin(self):
        with self.lock:
            self.epoch += 1
            self.capture = self._new()
            self.active = True

    def finish(self):
        with self.lock:
            if self.capture and self.active:
                self.capture['ended'] = time.monotonic()
            self.active = False
            return self.recording()

    def request(self):
        with self.lock:
            self.total['attempts'] += 1
            epoch = self.epoch if self.active else None
            if epoch is not None:
                self.capture['attempts'] += 1
            self.outstanding = epoch

    def result(self, outcome):
        if outcome not in ('successes', 'timeouts', 'errors'):
            raise ValueError(outcome)
        with self.lock:
            self.total[outcome] += 1
            if self.active and self.outstanding == self.epoch:
                self.capture[outcome] += 1
            self.outstanding = None

    def _snapshot(self, counter):
        if counter is None:
            return None
        elapsed = max(0., (counter['ended'] or time.monotonic()) - counter['started'])
        result = {k: counter[k] for k in ('attempts', 'successes', 'timeouts', 'errors')}
        result.update(pending=max(0, result['attempts']-sum(result[k] for k in ('successes','timeouts','errors'))),
                      success_rate=100*result['successes']/result['attempts'] if result['attempts'] else None,
                      effective_hz=result['successes']/elapsed if elapsed > 0 else None,
                      elapsed_seconds=elapsed,
                      definition='valid_on_time_responses / attempted_requests; in_flight_in_denominator',
                      freshness_basis='host_response_time; device_measurement_sequence_unavailable')
        return result

    def recording(self):
        with self.lock:
            return self._snapshot(self.capture)

    def lifetime(self):
        with self.lock:
            return self._snapshot(self.total)
