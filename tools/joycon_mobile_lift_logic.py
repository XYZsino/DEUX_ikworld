"""Only mobile/lift control; mapping follows Thor deux_leader_joycon.

No dependency on deux_leader. Times are monotonic seconds.
"""
import math


class Control:
    def __init__(self):
        self.enabled = dict(left=False, right=False)
        self.wait = dict(left=True, right=True)
        self.seen = dict(left=None, right=None)
        self.buttons = dict(left=False, right=False)
        self.axes = dict(left=[0.]*6, right=[0.]*6)
        self.turn_wait = True
        self.estop = False
        self.lift = 0

    def stop(self):
        self.enabled = dict(left=False, right=False)
        self.wait = dict(left=True, right=True)
        self.lift = 0
        self.turn_wait = True

    def emergency(self, engaged):
        self.estop = engaged
        if engaged:
            self.stop()

    def feed(self, side, axes, buttons, now):
        index = 4 if side == 'left' else 6
        if len(axes) < 6 or len(buttons) <= index or not all(math.isfinite(x) and abs(x) <= 1.001 for x in axes):
            self.stop()
            self.seen[side] = None
            return
        rejoin = self.seen[side] is None or now-self.seen[side] > .5
        if rejoin:
            self.turn_wait = True
            self.enabled[side] = False
            self.wait[side] = True
        pressed = bool(buttons[index])
        if pressed and not self.buttons[side] and not rejoin and not self.estop:
            self.enabled[side] = not self.enabled[side]
            if side == "right":
                self.turn_wait = True
            self.wait[side] = True
        self.buttons[side] = pressed
        self.axes[side] = list(axes)
        self.seen[side] = now
        x,y = self.stick(side)
        if abs(x) <= .15 and abs(y) <= .15:
            self.wait[side] = False
        # Rotation belongs to +, but uses the left stick. Require fresh neutral
        # on that stick after enabling + or reconnecting the left controller.
        lx, ly = self.stick("left")
        if (self.enabled["right"] and self.seen["left"] is not None
                and now-self.seen["left"] <= .5 and abs(lx) <= .15 and abs(ly) <= .15):
            self.turn_wait = False

    def stick(self, side):
        a = self.axes[side]
        return (a[0],a[1]) if side == 'left' else (a[2],a[3])

    @staticmethod
    def shape(x):
        return math.copysign(min(1.,max(0.,(abs(x)-.15)/.85)),x)

    def output(self, now):
        fresh = {s:self.seen[s] is not None and now-self.seen[s] <= .5 for s in self.seen}
        for side in fresh:
            if not fresh[side] or self.estop:
                self.enabled[side] = False
        if not fresh["left"]:
            self.turn_wait = True
        active = {s:self.enabled[s] and not self.wait[s] for s in fresh}
        turbo = all(fresh.values()) and self.axes['left'][4] < 0 and self.axes['right'][5] < 0
        v,w = (1.,1.) if turbo else (.4,.8)
        rx,ry = self.stick('right')
        lx,ly = self.stick('left')
        lx = lx if abs(lx) > .15 else 0.
        ly = ly if abs(ly) > .15 else 0.
        if min(abs(lx),abs(ly)) > max(abs(lx),abs(ly))*math.tan(math.radians(30)):
            lx=ly=0.
        elif abs(ly) >= abs(lx):
            lx=0.
        else:
            ly=0.
        if not active['left']:
            self.lift=0
        elif self.lift == 0:
            self.lift = 1 if ly > .6 else 2 if ly < -.6 else 0
        elif (self.lift == 1 and ly < .4) or (self.lift == 2 and ly > -.4):
            self.lift=0
        return (self.shape(ry)*v if active['right'] else 0.,
                self.shape(rx)*v if active['right'] else 0.,
                self.shape(lx)*w if self.enabled['right'] and fresh['left'] and not self.turn_wait else 0.),self.lift
