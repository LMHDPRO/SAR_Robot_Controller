"""Supervised arm pickup: advance only after the firmware reports each target."""

GRIPPER_HOLD_DEG = 15
ALIGNMENT_BASE_DEG = 117
PICKUP_BASE_DEG = 115
RELEASE_APPROACH_ANGLES = (120, 111.1, 160.6, 125.1)
RELEASE_PICK_ANGLES = (120, 14.3, 95.8, 162.9)
RELEASE_DEPART_ANGLES = (120, 55.2, 81.5, 123.6)
PICKUP_PICK_ANGLES = (PICKUP_BASE_DEG, *RELEASE_PICK_ANGLES[1:])
PICKUP_DEPART_ANGLES = (PICKUP_BASE_DEG, *RELEASE_DEPART_ANGLES[1:])
SAFE_RESCUE_ANGLES = (PICKUP_BASE_DEG, 110.5, 121.2, 99.7)
# Servo waypoints for the folded descent and low insertion. The last two
# insertion poses level the tool at the taught grasp height before seating it.
PICKUP_ENTRY = (
    ('Orientar base', (PICKUP_BASE_DEG, 149.4, 180, 6.8), 75),
    ('Bajar con pinza recogida', (PICKUP_BASE_DEG, 110, 180, 7), 75),
    ('Bajar a altura de recogida', (PICKUP_BASE_DEG, 98, 180, 61), 75),
    ('Extender pinza', (PICKUP_BASE_DEG, 95, 180, 83), 75),
    ('Extender a baja altura', (PICKUP_BASE_DEG, 86, 180, 114), 75),
    ('Orientar muñeca', (PICKUP_BASE_DEG, 74, 180, 144), 75),
    ('Nivelar pinza', (PICKUP_BASE_DEG, 43, 151, 177), 75),
    ('Entrar bajo la pieza', (PICKUP_BASE_DEG, 34, 124, 158), 75),
)
PICKUP_SETTLE_SECONDS = {'PICK': 1, 'Cerrar pinza': 2, 'Abrir pinza': 2}


def pickup_servo_target(joints, gripper):
    base, shoulder, elbow, wrist = map(round, joints)
    return (base, shoulder, elbow, 90, wrist, gripper)


class PickupSequence:
    def __init__(self, now, release=False):
        self.release = release
        self.steps = [
            *PICKUP_ENTRY,
            ('PICK', PICKUP_PICK_ANGLES, 75),
            ('Cerrar pinza', PICKUP_PICK_ANGLES, GRIPPER_HOLD_DEG),
            ('DEPART', PICKUP_DEPART_ANGLES, GRIPPER_HOLD_DEG),
            ('SAFE_Rescue', SAFE_RESCUE_ANGLES, GRIPPER_HOLD_DEG),
        ] if not release else [
            ('DEPART', RELEASE_DEPART_ANGLES, GRIPPER_HOLD_DEG),
            ('PICK', RELEASE_PICK_ANGLES, GRIPPER_HOLD_DEG),
            ('Abrir pinza', RELEASE_PICK_ANGLES, 75),
            ('Approach', RELEASE_APPROACH_ANGLES, 75),
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
