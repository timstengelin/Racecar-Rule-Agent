import random
from math import atan2, sin, cos, pi


class Racecar:
    def __init__(self, track):
        self._track = track
        self._actions = actions = [
            ('left', 'accelerate'), ('left', 'coast'), ('left', 'brake'),
            ('straight', 'accelerate'), ('straight', 'coast'),
            ('straight', 'brake'),
            ('right', 'accelerate'), ('right', 'coast'), ('right', 'brake')]

        t = random.uniform(.25, .75)

        self._x = t * track._startFinish[0][0] + (1 - t) * \
                  track._startFinish[1][0]
        self._y = t * track._startFinish[0][1] + (1 - t) * \
                  track._startFinish[1][1]
        self._v = 0.
        self._d = atan2(track._startFinish[0][0] - track._startFinish[1][0],
                        track._startFinish[0][1] - track._startFinish[1][
                            1]) + random.uniform(3 * pi / 4, 5 * pi / 4)

        self._oldPosition = 0.
        self._mostlyThere = False
        self._numSteps = 0
        self._totalReward = 0

        self._accel = .05
        self._maxV = 1

        self._done = False

        lidar = self._track.getLidar((self._x, self._y), self._d)
        self._obs = {'lidar': [x[0] for x in lidar], 'velocity': self._v,
                     'done': False}

    def actions(self):
        return tuple(self._actions)

    def step(self, action):
        self._numSteps += 1
        self._x += .5 * self._v * cos(self._d)
        self._y += .5 * self._v * sin(self._d)

        if action[0] == 'right' and self._v > 0:
            self._d += 6 * pi / 180
        elif action[0] == 'left' and self._v > 0:
            self._d -= 6 * pi / 180

        if action[1] == 'accelerate':
            self._v += min(self._accel, .1 * (self._maxV - self._v))
        elif action[1] == 'brake':
            self._v = max(0, self._v - self._accel)

        while self._d > 2 * pi:
            self._d -= 2 * pi
        while self._d < -2 * pi:
            self._d += 2 * pi

        self._x += .5 * self._v * cos(self._d)
        self._y += .5 * self._v * sin(self._d)

        newPosition = self._track.distanceTravelled(self._x, self._y)
        reward = (newPosition - self._oldPosition) * 100 / self._track._length
        self._oldPosition = newPosition

        lidar = self._track.getLidar((self._x, self._y), self._d)
        obs = {'lidar': [x[0] for x in lidar], 'velocity': self._v,
               'done': False}

        d = self._track.distance((self._x, self._y))
        if d >= 1:
            obs['done'] = True
            return obs, reward - 50, True

        done = False
        if self._mostlyThere:
            if newPosition < 1:
                done = True
                reward += 100
                reward += 120 * self._track._length / self._numSteps
        else:
            if newPosition > self._track._length - 5:
                self._mostlyThere = True

        if max(obs['lidar']) > 1e6:
            done = True
            obs['done'] = True
            reward -= 50

        self._done = done
        self._obs = obs

        return obs, reward, done

    def done(self):
        return self._done

    def observe(self):
        return self._obs

    def __str__(self):
        return f'<({self._x:.2f}, {self._y:.2f}), {180 / pi * self._d:.1f}, {self._v:.2f}>'
