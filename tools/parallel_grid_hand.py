"""Hand coroutine for a grid replay caller; no node or publisher is started here.

Start concurrently with the arm action using the SAME scheduled ROS start_at.
The caller supplies existing goal/state guards, feedback qualification and pause.
Returning records command timing, never proof of physical closure or grasp.
"""
import math


async def close_during_descent(plan, start_at, hand, *, now, pause, guard, qualified,
                               max_start_lateness_s=.02, strict_timing=True):
    schedule = plan['hand_schedule']; events = plan['events']
    begin = float(schedule['close_start_s']); duration = float(schedule['close_duration_s'])
    finish = float(events['contact_s']); period = float(schedule['repeat_period_s'])
    if (not all(math.isfinite(v) for v in (start_at, begin, duration, finish, period, max_start_lateness_s))
            or begin < 0 or duration <= 0 or not 0 < period <= .02
            or not 0 <= max_start_lateness_s <= period
            or abs(begin+duration-finish) > 1e-9
            or events['close_complete_s'] != finish or events['lift_start_s'] != finish):
        raise ValueError('invalid parallel closure schedule')
    first = None; last = None; count = 0; attempts = 0; previous = -float('inf'); timing_warnings = []
    def timing_issue(reason, elapsed):
        if strict_timing: raise ValueError(reason)
        timing_warnings.append(dict(reason=reason, elapsed_s=elapsed))
    while True:
        guard()
        elapsed = now()-start_at
        if elapsed < previous-1e-9: timing_issue('trajectory clock moved backwards', elapsed)
        previous = elapsed
        if elapsed < begin:
            await pause(min(period, begin-elapsed)); continue
        if first is None:
            if elapsed-begin > max_start_lateness_s+1e-9:
                timing_issue('missed hand closure start; nominal contact deadline unchanged', elapsed)
        elif elapsed-last > period+max_start_lateness_s+1e-9:
            timing_issue('hand command scheduling gap', elapsed)
        # Qualification compares live arm/object feedback at matching timestamps.
        # It must allow the planned remaining descent height, not require contact Z.
        try:
            if not qualified(elapsed): raise ValueError('parallel closure feedback not qualified')
        except InterruptedError:
            raise
        except Exception as error:
            timing_issue('hand feedback error: '+str(error), elapsed)
        try:
            hand.send(float(schedule['close_fraction']))
            if first is None: first = elapsed
            last = elapsed; count += 1
        except InterruptedError:
            raise
        except Exception as error:
            # A driver may refuse commands; report failed attempts without canceling the arm.
            timing_issue('hand command error: '+str(error), elapsed)
        attempts += 1
        if elapsed >= finish-1e-9:
            return dict(first_command_s=first, last_command_s=last, commands=count, attempts=attempts,
                        scheduled_contact_s=finish, physical_closure_verified=False, timing_warnings=timing_warnings)
        await pause(min(period, finish-elapsed))
