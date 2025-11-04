# Robotics and Intelligent Systems - IE4060
# Differential Drive Robot Simulation (P, PD, PID controllers)
# Author: Dayarathne S.H.N.R.
# ID: IT22581198
# Run: python assignment_ddr.py
# Requires: pygame, math, collections

import pygame
import math
import sys
from collections import deque

# Simulation Parameters
SCREEN_WIDTH = 1000       # Window width in pixels
SCREEN_HEIGHT = 700       # Window height in pixels
FPS = 60                  # Frames per second

# Robot physical parameters
WHEEL_RADIUS = 0.03       # Radius of wheels in meters
WHEEL_BASE = 0.18         # Distance between wheels (L) in meters
SCALE = 300               # Pixels per meter for display scaling

# Controller default gains
KP_DIST = 2.0             # Proportional gain for distance
KD_DIST = 0.8             # Derivative gain for distance (for PD/PID)
KI_DIST = 0.1             # Integral gain for distance (for PID)
KP_HEADING = 5.0          # Proportional gain for heading control
KI_HEADING = 0.05         # Integral gain for heading control (PID)
KD_HEADING = 0.8          # Derivative gain for heading control (PD/PID)

# Anti-windup limits for integral controllers
MAX_INTEGRAL_DIST = 1.0   # Maximum integral value for distance error
MAX_INTEGRAL_HEADING = 1.0 # Maximum integral value for heading error

# Stopping thresholds
DIST_THRESHOLD = 0.02     # Minimum distance to target (meters)
HEADING_THRESHOLD = 0.03  # Minimum heading error (radians)

# Trajectory history length
MAX_TRAIL = 10000         # Maximum number of points to store in trajectory

# Colors used in simulation
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (220, 20, 60)
GREEN = (34, 139, 34)
BLUE = (30, 144, 255)
GRAY = (200, 200, 200)
YELLOW = (255, 215, 0)

# Utility function to wrap angles to [-pi, pi]
def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

# Utility function to compute Euclidean distance
def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

# Robot Class
class DifferentialDriveRobot:
    # Class representing a differential drive robot with wheel velocities,
    # pose updates, and trajectory tracking
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x                      # Robot x-position (meters)
        self.y = y                      # Robot y-position (meters)
        self.theta = theta              # Robot orientation (radians)
        
        self.omega_r = 0.0              # Right wheel angular velocity (rad/s)
        self.omega_l = 0.0              # Left wheel angular velocity (rad/s)

        # Controller state variables
        self.prev_dist_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_dist_error = 0.0
        self.integral_heading_error = 0.0

        # Trajectory storage for visualization
        self.traj = deque(maxlen=MAX_TRAIL)

    def forward_kinematics(self, omega_l, omega_r):
        # Compute robot linear and angular velocity from wheel velocities
        v = WHEEL_RADIUS * 0.5 * (omega_r + omega_l)
        omega = WHEEL_RADIUS / WHEEL_BASE * (omega_r - omega_l)
        return v, omega

    def inverse_kinematics(self, v, omega):
        # Compute wheel velocities required for given linear and angular velocity
        omega_r = (2.0 * v + omega * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
        omega_l = (2.0 * v - omega * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
        return omega_l, omega_r

    def set_wheel_omegas(self, omega_l, omega_r):
        # Set robot wheel velocities
        self.omega_l = omega_l
        self.omega_r = omega_r

    def update(self, dt):
        # Update robot pose using forward kinematics and integration
        v, omega = self.forward_kinematics(self.omega_l, self.omega_r)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = wrap_angle(self.theta)

        # Append new pose to trajectory with current control mode
        self.traj.append((self.x, self.y, controllers.mode))

    def reset_controller_state(self):
        # Reset integral and previous errors (useful when changing modes/targets)
        self.prev_dist_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_dist_error = 0.0
        self.integral_heading_error = 0.0

# Controllers Class
class Controllers:
    # Implements P, PD, and PID controllers for distance and heading
    def __init__(self):
        # Controller gains
        self.kp_dist = KP_DIST
        self.kd_dist = KD_DIST
        self.ki_dist = KI_DIST
        self.kp_head = KP_HEADING
        self.ki_head = KI_HEADING
        self.kd_head = KD_HEADING

        # Default controller mode
        self.mode = 'P'

    def compute_control(self, robot, target, dt):
        # Compute control outputs: linear velocity v and angular velocity omega
        rx, ry, rtheta = robot.x, robot.y, robot.theta
        tx, ty = target

        # Distance and heading errors
        dx = tx - rx
        dy = ty - ry
        dist_err = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_err = wrap_angle(desired_heading - rtheta)

        # Distance control
        v = 0.0
        if self.mode == 'P':
            v = self.kp_dist * dist_err
            robot.integral_dist_error = 0.0
        elif self.mode == 'PD':
            d_err = (dist_err - robot.prev_dist_error) / dt if dt > 0 else 0.0
            v = self.kp_dist * dist_err + self.kd_dist * d_err
            robot.integral_dist_error = 0.0
        elif self.mode == 'PID':
            robot.integral_dist_error += dist_err * dt
            robot.integral_dist_error = max(-MAX_INTEGRAL_DIST, min(MAX_INTEGRAL_DIST, robot.integral_dist_error))
            d_err = (dist_err - robot.prev_dist_error) / dt if dt > 0 else 0.0
            v = self.kp_dist * dist_err + self.ki_dist * robot.integral_dist_error + self.kd_dist * d_err

        # Limit linear velocity
        max_v = 0.6
        v = max(-max_v, min(max_v, v))

        # Heading control
        omega = 0.0
        de_heading = (heading_err - robot.prev_heading_error) / dt if dt > 0 else 0.0
        if self.mode == 'P':
            omega = self.kp_head * heading_err
            robot.integral_heading_error = 0.0
        elif self.mode == 'PD':
            omega = self.kp_head * heading_err + self.kd_head * de_heading
            robot.integral_heading_error = 0.0
        elif self.mode == 'PID':
            robot.integral_heading_error += heading_err * dt
            robot.integral_heading_error = max(-MAX_INTEGRAL_HEADING, min(MAX_INTEGRAL_HEADING, robot.integral_heading_error))
            omega = self.kp_head * heading_err + self.ki_head * robot.integral_heading_error + self.kd_head * de_heading

        # Limit angular velocity
        max_omega = 4.0
        omega = max(-max_omega, min(max_omega, omega))

        # Store current errors for derivative calculations
        robot.prev_dist_error = dist_err
        robot.prev_heading_error = heading_err

        # Stop robot if close enough to target
        if dist_err < DIST_THRESHOLD:
            v = 0.0
            omega = 0.0
            robot.integral_dist_error = 0.0
            robot.integral_heading_error = 0.0

        return v, omega, dist_err, heading_err

# Drawing helpers for Pygame
def meters_to_pixels(x_m, y_m):
    # Convert meters to pixels for screen display
    px = int(100 + x_m * SCALE)
    py = int(SCREEN_HEIGHT/2 - y_m * SCALE)
    return px, py

def draw_robot(screen, robot):
    # Draw robot body and heading
    rx, ry = meters_to_pixels(robot.x, robot.y)
    radius = int(0.12 * SCALE)
    pygame.draw.circle(screen, BLUE, (rx, ry), radius, 2)
    hx = rx + int(math.cos(robot.theta) * radius)
    hy = ry - int(math.sin(robot.theta) * radius)
    pygame.draw.line(screen, RED, (rx, ry), (hx, hy), 3)
    pygame.draw.circle(screen, BLACK, (rx, ry), 4)

def draw_target(screen, target):
    # Draw target as green cross
    tx, ty = meters_to_pixels(target[0], target[1])
    pygame.draw.circle(screen, GREEN, (tx, ty), 7)
    pygame.draw.line(screen, GREEN, (tx-10, ty), (tx+10, ty), 1)
    pygame.draw.line(screen, GREEN, (tx, ty-10), (tx, ty+10), 1)

def draw_trajectory(screen, traj):
    # Draw trajectory using colors for different controller modes
    if len(traj) < 2:
        return
    for i in range(1, len(traj)):
        x1, y1, mode1 = traj[i-1]
        x2, y2, mode2 = traj[i]
        px1, py1 = meters_to_pixels(x1, y1)
        px2, py2 = meters_to_pixels(x2, y2)
        if mode1 == 'P':
            color = (255, 100, 100)
        elif mode1 == 'PD':
            color = (100, 255, 100)
        else:
            color = (100, 100, 255)
        pygame.draw.line(screen, color, (px1, py1), (px2, py2), 3)

def draw_text(screen, text, x, y, size=18, color=BLACK):
    # Helper function to draw text on screen
    font = pygame.font.SysFont('Arial', size, bold=(size >= 20))
    surf = font.render(text, True, color)
    screen.blit(surf, (x, y))

# Main simulation loop
def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Differential Drive Robot Simulation - P / PD / PID (Full)")
    clock = pygame.time.Clock()

    global controllers
    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
    controllers = Controllers()
    target = (1.2, 0.3)  # Default target
    running = True
    time_elapsed = 0.0

    # Instructions displayed on screen
    instructions = [
        "Left-click to set target",
        "Keys: 1=P, 2=PD, 3=PID (full)",
        "Up/Down: KP_DIST ±0.1",
        "Left/Right: KP_HEAD ±0.1",
        "W/S: KI_DIST ±0.01 (PID)",
        "I/K: KI_HEAD ±0.01 (PID)",
        "C: clear trajectory, R: reset",
    ]

    while running:
        dt = clock.tick(FPS) / 1000.0
        time_elapsed += dt

        # Handle user input events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mx, my = event.pos
                    tx = (mx - 100) / SCALE
                    ty = (SCREEN_HEIGHT/2 - my) / SCALE
                    target = (tx, ty)
                    robot.reset_controller_state()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    controllers.mode = 'P'
                    robot.reset_controller_state()
                elif event.key == pygame.K_2:
                    controllers.mode = 'PD'
                    robot.reset_controller_state()
                elif event.key == pygame.K_3:
                    controllers.mode = 'PID'
                    robot.reset_controller_state()
                elif event.key == pygame.K_c:
                    robot.traj.clear()
                elif event.key == pygame.K_r:
                    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
                    controllers = Controllers()
                elif event.key == pygame.K_UP:
                    controllers.kp_dist += 0.1
                elif event.key == pygame.K_DOWN:
                    controllers.kp_dist = max(0.0, controllers.kp_dist - 0.1)
                elif event.key == pygame.K_w:
                    controllers.ki_dist += 0.01
                elif event.key == pygame.K_s:
                    controllers.ki_dist = max(0.0, controllers.ki_dist - 0.01)
                elif event.key == pygame.K_LEFT:
                    controllers.kp_head = max(0.0, controllers.kp_head - 0.1)
                elif event.key == pygame.K_RIGHT:
                    controllers.kp_head += 0.1
                elif event.key == pygame.K_i:
                    controllers.ki_head += 0.01
                elif event.key == pygame.K_k:
                    controllers.ki_head = max(0.0, controllers.ki_head - 0.01)

        # Compute control commands
        v, omega, dist_err, heading_err = controllers.compute_control(robot, target, dt)
        omega_l, omega_r = robot.inverse_kinematics(v, omega)
        robot.set_wheel_omegas(omega_l, omega_r)
        robot.update(dt)

        # Draw background and grid
        screen.fill(WHITE)
        for gx in range(0, SCREEN_WIDTH, 50):
            pygame.draw.line(screen, GRAY, (gx, 0), (gx, SCREEN_HEIGHT), 1)
        for gy in range(0, SCREEN_HEIGHT, 50):
            pygame.draw.line(screen, GRAY, (0, gy), (SCREEN_WIDTH, gy), 1)

        # Draw simulation elements
        draw_trajectory(screen, robot.traj)
        draw_target(screen, target)
        draw_robot(screen, robot)

        # Display controller parameters and errors
        mode_color = RED if controllers.mode == 'P' else (GREEN if controllers.mode == 'PD' else BLUE)
        draw_text(screen, f"Mode: {controllers.mode} (1=P, 2=PD, 3=PID Full)", 650, 10, color=mode_color, size=20)
        draw_text(screen, "Press C to clear trajectory after mode change!", 650, 30, size=14, color=(255, 140, 0))
        draw_text(screen, f"Distance Control:", 650, 55, size=16)
        draw_text(screen, f"KP: {controllers.kp_dist:.2f}  KI: {controllers.ki_dist:.3f}  KD: {controllers.kd_dist:.2f}", 650, 75, size=16)
        draw_text(screen, f"Integral: {robot.integral_dist_error:.3f}", 650, 95, size=16)
        draw_text(screen, f"Heading Control:", 650, 125, size=16)
        draw_text(screen, f"KP: {controllers.kp_head:.2f}  KI: {controllers.ki_head:.3f}  KD: {controllers.kd_head:.2f}", 650, 145, size=16)
        draw_text(screen, f"Integral: {robot.integral_heading_error:.3f}", 650, 165, size=16)
        draw_text(screen, f"Errors:", 650, 195, size=16)
        draw_text(screen, f"Distance: {dist_err:.3f} m", 650, 215, size=16)
        draw_text(screen, f"Heading: {math.degrees(heading_err):.2f}°", 650, 235, size=16)
        draw_text(screen, f"Velocities:", 650, 265, size=16)
        draw_text(screen, f"Linear (v): {v:.3f} m/s", 650, 285, size=16)
        draw_text(screen, f"Angular (ω): {omega:.3f} rad/s", 650, 305, size=16)

        yline = 345
        draw_text(screen, "Trajectory Color: Red=P, Green=PD, Blue=PID", 650, 335, size=14, color=(128, 0, 128))
        for ins in instructions:
            draw_text(screen, ins, 650, yline, size=14, color=BLACK)
            yline += 20

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    run_simulation()