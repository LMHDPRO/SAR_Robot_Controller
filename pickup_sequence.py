"""Supervised arm pickup: advance only after the firmware reports each target."""

GRIPPER_HOLD_DEG = 15
PICKUP_APPROACH_ANGLES = (111.7, 111.1, 160.6, 125.1)
PICKUP_PICK_ANGLES = (115.2, 14.3, 95.8, 162.9)
PICKUP_DEPART_ANGLES = (115.2, 55.2, 81.5, 123.6)
SAFE_RESCUE_ANGLES = (115.2, 110.5, 121.2, 99.7)
PICKUP_SETTLE_SECONDS = {'PICK': 1, 'Cerrar pinza': 2, 'Abrir pinza': 2}


def pickup_servo_target(joints, gripper):
    base, shoulder, elbow, wrist = map(round, joints)
    return (base, shoulder, elbow, 90, wrist, gripper)


class PickupSequence:
    def __init__(self, now, release=False):
        self.release = release
        self.steps = [
            ('Approach', PICKUP_APPROACH_ANGLES, 75),
            ('PICK', PICKUP_PICK_ANGLES, 75),
            ('Cerrar pinza', PICKUP_PICK_ANGLES, GRIPPER_HOLD_DEG),
            ('DEPART', PICKUP_DEPART_ANGLES, GRIPPER_HOLD_DEG),
            ('SAFE_Rescue', SAFE_RESCUE_ANGLES, GRIPPER_HOLD_DEG),
        ] if not release else [
            ('DEPART', PICKUP_DEPART_ANGLES, GRIPPER_HOLD_DEG),
            ('PICK', PICKUP_PICK_ANGLES, GRIPPER_HOLD_DEG),
            ('Abrir pinza', PICKUP_PICK_ANGLES, 75),
            ('Approach', PICKUP_APPROACH_ANGLES, 75),
        ]
        self.index = -1
        self.started = now
        self.seen_busy = False
        self.wait_until = None
        self.done = False

    def next_step(self, now):
        self.index += 1
        if self.index == len(self.steps):
            self.done = True
            return None
        self.started = now
        self.seen_busy = False
        return self.steps[self.index]

    def observe(self, channels, busy, now):
        name, joints, grip = self.steps[self.index]
        target = pickup_servo_target(joints, grip)
        if self.wait_until is not None:
            if busy or any(abs(a-b)>1 for a,b in zip(channels,target)):
                raise RuntimeError('La pose estimada cambió durante la espera')
            if now < self.wait_until:return None
            self.wait_until = None
            return self.next_step(now)
        if now-self.started > 6:raise RuntimeError('Brazo sin confirmación de la etapa')
        if busy:self.seen_busy = True
        if self.seen_busy and not busy and now-self.started >= 3 and all(abs(a-b)<=1 for a,b in zip(channels,target)):
            if name in PICKUP_SETTLE_SECONDS:
                self.wait_until = now+PICKUP_SETTLE_SECONDS[name]
                return None
            return self.next_step(now)
        return None
