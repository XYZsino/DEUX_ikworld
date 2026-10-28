"""Reserve planning/validation/dispatch time on the same clock as interception.

Wall-time compute measurements are converted using the observed ROS clock rate.
The initial budget is a prior, never permission to execute a missed schedule.
"""
from collections import deque
import math


DEFAULTS = dict(initial_search_wall_s=3.0, initial_revalidation_wall_s=.5,
                safety_factor=1.25, dispatch_lead_s=.25, max_research_attempts=2,
                measurement_window=8, minimum_compute_wall_s=.1)


def settings(config):
    cfg = dict(DEFAULTS)
    cfg.update(config.get('conveyor_tracking', {}).get('candidate_search', {})
               .get('time_search', {}).get('planning_budget', {}))
    if set(cfg) != set(DEFAULTS):
        raise ValueError('unknown planning budget setting')
    for key in DEFAULTS.keys() - {'max_research_attempts','measurement_window'}:
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid planning budget ' + key)
    if cfg['safety_factor'] < 1 or cfg['dispatch_lead_s'] < .1:
        raise ValueError('insufficient planning budget margin')
    if type(cfg['max_research_attempts']) is not int or not 0 <= cfg['max_research_attempts'] <= 5:
        raise ValueError('invalid planning retry bound')
    if type(cfg['measurement_window']) is not int or not 1<=cfg['measurement_window']<=100:
        raise ValueError('invalid planning measurement window')
    return cfg


class PlanningLatency:
    def __init__(self, config, use_sim_time=False):
        self.config = settings(config)
        self.use_sim_time = use_sim_time
        self.samples = deque(maxlen=100)
        self.wall_budget = dict(search=self.config['initial_search_wall_s'],
                                revalidation=self.config['initial_revalidation_wall_s'])
        self.measurements = {kind:deque(maxlen=self.config['measurement_window']) for kind in self.wall_budget}

    def sample(self, ros_now, wall_now):
        if self.samples and (ros_now < self.samples[-1][0] or wall_now < self.samples[-1][1]):
            self.samples.clear()
        if not self.samples or wall_now > self.samples[-1][1]:
            self.samples.append((ros_now, wall_now))
        while len(self.samples) > 2 and wall_now - self.samples[0][1] > 3.:
            self.samples.popleft()

    @property
    def clock_rate(self):
        if not self.use_sim_time or len(self.samples) < 2:
            return 1.
        first, last = self.samples[0], self.samples[-1]
        elapsed = last[1] - first[1]
        if elapsed < .25 or last[0] <= first[0]:
            return 1.
        return max(.01, (last[0] - first[0]) / elapsed)

    def observe(self, kind, elapsed_wall):
        if not math.isfinite(elapsed_wall) or elapsed_wall < 0:
            raise ValueError('invalid planning duration')
        # Initial priors apply only until the first measurement. Keep the recent
        # maximum, then apply the safety factor when reserving a future start.
        self.measurements[kind].append(elapsed_wall)
        self.wall_budget[kind] = max(self.config['minimum_compute_wall_s'],max(self.measurements[kind]))

    def compute_lead(self, kind):
        return self.wall_budget[kind] * self.clock_rate * self.config['safety_factor']

    def reserve_start(self, now):
        return now + self.compute_lead('search') + self.compute_lead('revalidation') + self.config['dispatch_lead_s']

    def can_validate(self, scheduled, now):
        return scheduled - now >= self.compute_lead('revalidation') + self.config['dispatch_lead_s']

    def can_dispatch(self, scheduled, now):
        return scheduled - now >= self.config['dispatch_lead_s']

    def diagnostics(self):
        return dict(clock_rate=self.clock_rate, search_wall_budget_s=self.wall_budget['search'],
                    revalidation_wall_budget_s=self.wall_budget['revalidation'],
                    search_clock_budget_s=self.compute_lead('search'),
                    revalidation_clock_budget_s=self.compute_lead('revalidation'),
                    dispatch_lead_s=self.config['dispatch_lead_s'])
