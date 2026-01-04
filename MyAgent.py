# VERSION 03
# - Mit Startup aa

import math
from math import pi

class Agent:
    def __init__(self):
        self.step = -1
        self.history = [[None] * 4, [None] * 4]

    def round_half(self, x):
        return round(x * 2) / 2

    def round_to_005(self, x):
        return round(math.ceil(x / 0.05) * 0.05, 2)

    def get_outer_circle(self,
                        r1: float, theta1_deg: float,
                        r2: float, theta2_deg: float,
                        r3: float, theta3_deg: float):

        # convert to cartesian
        def to_xy(r, deg):
            th = math.radians(deg)
            return r * math.cos(th), r * math.sin(th)

        x1, y1 = to_xy(r1, theta1_deg)
        x2, y2 = to_xy(r2, theta2_deg)
        x3, y3 = to_xy(r3, theta3_deg)

        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))

        A1 = x1 * x1 + y1 * y1
        A2 = x2 * x2 + y2 * y2
        A3 = x3 * x3 + y3 * y3

        Ux = (A1 * (y2 - y3) + A2 * (y3 - y1) + A3 * (y1 - y2)) / (D + 1e-10)
            # Prevent division by zero by 1e-10
        Uy = (A1 * (x3 - x2) + A2 * (x1 - x3) + A3 * (x2 - x1)) / (D + 1e-10)
            # Prevent division by zero by 1e-10

        R = math.hypot(Ux - x1, Uy - y1)
        return Ux, Uy, R

    def get_actual_radius_path(self, Ux, Uy):
        return (Ux**2 + Uy**2)**0.5

    def chooseAction(self, observations, possibleActions):
        # -------- COUNTER AND HISTORY --------
        # Increment counter
        self.step += 1

        # Initialize history for one step
        history_step = [self.step,
                        None, # direction_control
                        None, # vel_control
                        None, # dir_track_detected
                        None]

        # -------- OBSERVATIONS --------
        # Distances
        dist_l45 = observations['lidar'][0]
        dist_l10 = observations['lidar'][1]
        dist_s   = observations['lidar'][2]
        dist_r10 = observations['lidar'][3]
        dist_r45 = observations['lidar'][4]

        # Velocity
        vel = observations['velocity']

        # -------- NON-TUNABLE/LEARNABLE PARAMETERS (Environment specific) --------
        TRACK_WIDTH = 2
        MAX_BRAKE_PER_STEP = 0.05
            # max. velocity change per step
        MAX_DELTA_DIR_PER_STEP = 6 * pi / 180
            # max. change in direction in rad/step

        # -------- PARAMETERS --------
        # index
        IDX_HISTORY_STEP = 0
        IDX_HISTORY_DIRECTION_CONTROL = 1
        IDX_HISTORY_VEL_CONTROL = 2
        IDX_HISTORY_DIR_TRACK_DETECTED = 3

        # speed-related
        EMERGENCY_MARGIN_STOP_DIST = 0.2
            # emergency-margin of velocity-dependent stopping distance
            # regarding head-on collision
        TOLERANCE_VEL_CONTROL = 1e-4

        # direction-related
        WEIGHT_DIST_10 = 0.6
        WEIGHT_DIST_45 = 0.4
        TOLERANCE_DIR_TRACK = 0.05
        TOLERANCE_DIR_CONTROL_STRAIGHT = 0.05
        TOLERANCE_DIR_CONTROL_TURN = 0.1

        # -------- DERIVED VALUES --------
        # Minimal distances
        min_dist_l = min(dist_l10, dist_l45)
        min_dist_r = min(dist_r10, dist_r45)
        min_dist = min(dist_s, min_dist_l, min_dist_r)

        # Weighted and balanced distances
        weighSum_dist_l = WEIGHT_DIST_10 * dist_l10 + WEIGHT_DIST_45 * dist_l45
        weighSum_dist_r = WEIGHT_DIST_10 * dist_r10 + WEIGHT_DIST_45 * dist_r45
        balance_dist = (weighSum_dist_l - weighSum_dist_r)

        # Detected direction of track
        if abs(balance_dist) < TOLERANCE_DIR_TRACK:
            dir_track_detected = 'straight ahead'
        elif balance_dist > 0:
            dir_track_detected = 'left turn'
        else:
            dir_track_detected = 'right turn'
        history_step[IDX_HISTORY_DIR_TRACK_DETECTED] = dir_track_detected

        # Decide on direction of track
        if dir_track_detected == 'straight ahead':
            dir_track = 'straight ahead'
        elif dir_track_detected == 'left turn':
            dir_track = 'left turn'
        elif dir_track_detected == 'right turn':
            dir_track = 'right turn'
        else:
            raise ValueError('Error (01)!')

        # Stopping distance regarding head-on collision
        stop_dist = (vel * vel) / (2.0 * MAX_BRAKE_PER_STEP)
            # velocity-dependent stopping distance:
            # stopping_distance = vel^2 / (2 * MAX_BRAKE_PER_STEP)
        stop_dist_with_margin = stop_dist * (1 + EMERGENCY_MARGIN_STOP_DIST)

        # -------- 1. Rule: Default operation --------
        # CALCULATIONS
        if dir_track == 'straight ahead': # 'straight ahead'
            radius_outer = -1 # Helper
            radius_inner = -1 # Helper
            desir_max_vel = 1
            desir_radius_path = -1 # Helper
            actual_radius_path = -1 # Helper

        else: # 'left turn' or 'right turn'
            if dir_track == 'left turn': # 'left' turn
                Ux, Uy, radius_outer = self.get_outer_circle(dist_r45, -45.0, dist_r10, -10.0, dist_s, -0.0)
            else: # 'right turn'
                Ux, Uy, radius_outer = self.get_outer_circle(dist_l45, +45.0, dist_l10, +10.0, dist_s, +0.0)

            radius_outer = self.round_half(radius_outer) # round radius to, e.g., 1.5, 2, 2.5, 3, ...

            radius_inner = radius_outer - TRACK_WIDTH
            desir_radius_path = radius_outer - 1.5
            actual_radius_path = self.get_actual_radius_path(Ux, Uy)

            #desir_rel_path_pos = (desir_radius_path - radius_inner) / (radius_outer - radius_inner + 1e-10) # TODO Not used
                # with prevention from dividing by zero
            actual_rel_path_pos = (actual_radius_path - radius_inner) / (radius_outer - radius_inner + 1e-10)
                # with prevention from dividing by zero
            #balance_rel_path_pos = actual_rel_path_pos - desir_rel_path_pos

            desir_max_vel = max(0.0, min(1.0, actual_radius_path * MAX_DELTA_DIR_PER_STEP)) #* 0.7 #* 0.95 # TODO: Change desir_radius_path?; faktor 0.7 DRASTICALLY REDUCED
            if desir_max_vel < 0.6:
                desir_max_vel = desir_max_vel * 0.6

            if self.step > 130:
                desir_max_vel = desir_max_vel * 0.3

            if desir_max_vel < 0.5: # round desired velocity to, e.g., 0.15, 0.20, 0.25, ...
                desir_max_vel = self.round_to_005(desir_max_vel)

            balance_vel = vel - desir_max_vel

        # VELOCITY CONTROL
        if dir_track == 'straight ahead':  # 'straight ahead'
            vel_control = 'accelerate'
        else:  # 'left turn' or 'right turn'
            if abs(balance_vel) < TOLERANCE_VEL_CONTROL:
                vel_control = 'coast'
            elif balance_vel > 0:
                vel_control = 'brake'
            else: # balance_vel < 0
                vel_control = 'accelerate'

        # DIRECTION CONTROL
        if dir_track == 'straight ahead':  # 'straight ahead'
            if abs(balance_dist) < TOLERANCE_DIR_CONTROL_STRAIGHT:
                direction_control = 'straight'
            elif balance_dist > 0:
                direction_control = 'left'
            else:
                direction_control = 'right'
        else:  # 'left turn' or 'right turn'
            if dir_track == 'left turn': # 'left' turn
                if actual_rel_path_pos > 0.05:
                    direction_control = 'left'
                else: # actual_rel_path_pos <= 0.5:
                    direction_control = 'straight'
            else: # 'right turn'
                if actual_rel_path_pos > 0.05:
                    direction_control = 'right'
                else: # actual_rel_path_pos <= 0.5:
                    direction_control = 'straight'

        # -------- 2. Rule: Emergency stopping regarding head-on collision --------
        emergency_brake = False
        if vel > 0 and dist_s <= stop_dist_with_margin and self.step > 130:
            emergency_brake = True
            vel_control = 'brake'
            if balance_dist > 0:
               direction_control = 'left'
            else: # balance_dist <= 0
               direction_control = 'right'

        # -------- 3. Rule: Start-up --------
        if self.step < 9 + 1:
            # SPEED CONTROL
            if self.step % 5 == 0:
                vel_control = 'accelerate'
            else:
                vel_control = 'coast'

            # DIRECTION CONTROL
            if abs(balance_dist) < TOLERANCE_DIR_CONTROL_STRAIGHT:
                direction_control = 'straight'
            elif balance_dist > 0:
                direction_control = 'left'
            else:
                direction_control = 'right'

        # Append history for one step to history
        history_step[IDX_HISTORY_DIRECTION_CONTROL] = direction_control
        history_step[IDX_HISTORY_VEL_CONTROL] = vel_control
        self.history.append(history_step)

        # Helper
        #print('dir_track: ' + dir_track)
        #print('dir_track_detected: ' + dir_track_detected)
        #print('-----')
        #print('radius_outer: ' + str(radius_outer))
        #print('desir_radius_path: ' + str(desir_radius_path))
        #print('radius_inner: ' + str(radius_inner))
        #print('-----')
        #print('desir_max_vel: ' + str(desir_max_vel))
        #print('actual_vel: ' + str(vel))
        #print('-----')
        #print('emergency: ' + str(emergency_brake))
        #print('-----')
        #print('step: ' + str(self.step))

        return direction_control, vel_control
