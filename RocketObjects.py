eng_lut = {'A': [1.26, 2.50],
           'B': [2.51, 5.00],
           'C': [5.01, 10.0],
           'D': [10.01, 20.0],
           'E': [20.01, 40.0],
           'F': [40.01, 80.0],
           'G': [80.01, 160],
           'H': [160.01, 320]}  # add range force


class Engine:
    def __init__(self, parram):
        self.pos = (0, 0, 60)
        self.mass = 0.010

        self.inertia = (1, 1, 1)

        self.letter = parram[0]
        self.force, self.delay = parram[1:].split('-')
        self.force, self.delay = int(self.force), int(self.delay)
        self.impulse = self.impulse_calc()
        self.shutdown_t = self.impulse / self.force

    def impulse_calc(self):
        impulse_rang = eng_lut[self.letter]  # todo change so range is betwen max min, is delay before or after shutdown
        return impulse_rang[0] + self.force/10*impulse_rang[1]-impulse_rang[0]


class Rocket:
    def __init__(self):
        self.mass = 0.151
        self.inertia = (1, 1, 1)
        self.COM = (0, 0, 30)
        self.nose = (0, 0, 60)


class Motor:
    def __init__(self):
        self.Torque = 1
        self.rad = 0.04


class Parachute:
    def __init__(self, delay):
        self.delay = delay
        self.force = 1
