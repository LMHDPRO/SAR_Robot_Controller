"""Supervised arm pickup: advance only after the firmware reports each target."""


class PickupSequence:
    def __init__(self, safe_pose, now, release=False):
        self.release = release
        self.steps = [
            ('Approach', [110.9, 112.5, 160.5, 106.0], 75),
            ('Pick', [110.9, 71.1, 142.8, 84.7], 75),
            ('Cerrar pinza', [110.9, 71.1, 142.8, 84.7], 10),
            ('Volver a Approach', [110.9, 112.5, 160.5, 106.0], 10),
            ('Volver a posición segura', safe_pose, 10),
        ] if not release else [
            ('Approach con pieza', [110.9, 112.5, 160.5, 106.0], 10),
            ('Posición para dejar', [110.9, 71.1, 142.8, 84.7], 10),
            ('Abrir pinza', [110.9, 71.1, 142.8, 84.7], 75),
            ('Retirar a Approach', [110.9, 112.5, 160.5, 106.0], 75),
            ('Volver a posición segura', safe_pose, 75),
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
        if self.wait_until is not None:
            _, joints, grip = self.steps[self.index]
            target = [round(joints[0]),round(joints[1]),round(joints[2]),90,round(joints[3]),grip]
            if busy or any(abs(a-b)>1 for a,b in zip(channels,target)):
                raise RuntimeError('La pose estimada cambió durante la espera')
            if now < self.wait_until:return None
            self.wait_until = None
            return self.next_step(now)
        if now-self.started > 6:raise RuntimeError('Brazo sin confirmación de la etapa')
        if busy:self.seen_busy = True
        _, joints, grip = self.steps[self.index]
        target = [round(joints[0]), round(joints[1]), round(joints[2]), 90, round(joints[3]), grip]
        if self.seen_busy and not busy and now-self.started >= 3 and all(abs(a-b)<=1 for a,b in zip(channels,target)):
            if self.index in (1,2):
                self.wait_until = now+(1 if self.index==1 else 2)
                return None
            return self.next_step(now)
        return None
