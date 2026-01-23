import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as mcolors
import ipywidgets as widgets
from IPython.display import display, clear_output

# --- CONSTANTS (MKS) ---
G = 9.81
FIELD_X_LEN = 16.46
FIELD_Y_LEN = 8.23
HUB_CENTER_X = 4.6
HUB_CENTER_Y = 4.0

# Hub Geometry
HUB_TOP_H = 72.0 * 0.0254      # 1.83 m
HUB_BOT_H = 50.0 * 0.0254      # 1.27 m
HUB_TOP_R = (41.3 / 2) * 0.0254
HUB_BOT_R = (28.0 / 2) * 0.0254
HUB_DEPTH_TOTAL = HUB_TOP_H - HUB_BOT_H

# Aliases for compatibility
R_TOP = HUB_TOP_R
R_BOT = HUB_BOT_R

# Ball Physics
BALL_MASS = 0.5 * 0.453592  # kg
BALL_DIAM = 5.9 * 0.0254
BALL_RADIUS = BALL_DIAM / 2.0
BALL_AREA = np.pi * (BALL_RADIUS**2)
AIR_DENSITY = 1.225
Cd = 0.5

# Robot/Shooter
WHEEL_DIAM_IN = 4.0
WHEEL_CIRCUM_M = (WHEEL_DIAM_IN * 0.0254) * np.pi
WHEEL_CIRCUM = WHEEL_CIRCUM_M

# --- PHYSICS FUNCTIONS ---

def calculate_trajectory(v0, angle_deg, h0, dist, use_drag=True):
    angle_rad = np.radians(angle_deg)
    vx = v0 * np.cos(angle_rad)
    vy = v0 * np.sin(angle_rad)
    
    dt = 0.005
    x, y = 0.0, h0
    xs, ys = [x], [y]
    
    # Sim loop
    while y > 0 and x < (dist + 2.0):
        v = np.sqrt(vx**2 + vy**2)
        if use_drag:
            f_drag = 0.5 * AIR_DENSITY * (v**2) * Cd * BALL_AREA
            ax = -(f_drag * (vx/v)) / BALL_MASS
            ay = -G - (f_drag * (vy/v)) / BALL_MASS
        else:
            ax, ay = 0, -G
            
        x += vx * dt
        y += vy * dt
        vx += ax * dt
        vy += ay * dt
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)

def analyze_shot(xs, ys, dist):
    # Find point closest to target distance
    idx = (np.abs(xs - dist)).argmin()
    if idx >= len(ys) - 1: return "SHORT", "#F44336" # Red
    
    z = ys[idx]
    
    # 1. Check geometric clearance
    if z < HUB_BOT_H: return "TOO LOW", "#F44336"
    if z > HUB_TOP_H + 1.2: return "TOO HIGH", "#F44336"
    
    # 2. Check if falling
    is_falling = (ys[idx] - ys[idx-1]) < 0 if idx > 0 else False
    if not is_falling: return "FLAT/RISING", "#FF9800" # Orange
    
    # 3. Precision Check
    error = abs(z - HUB_TOP_H)
    if error < 0.20: return "GOOD", "#4CAF50"   # Green
    elif error < 0.50: return "MARGINAL", "#FFEB3B" # Yellow
    return "POOR", "#F44336" # Red

def solve_angle_batch(angle_deg, distances, shot_height_m, check_h, max_safe_r, efficiency, max_rpm):
    """
    Simulates a full sweep of velocities for a single angle.
    Returns best RPM and Shot Quality for each distance.
    Quality 1.0 = Dead Center. Quality 0.0 = Grazing Limit.
    """
    # 1. Create a range of test velocities (2m/s to 30m/s)
    v0_tests = np.linspace(2, 30, 1000)
    
    # 2. Setup initial state for ALL velocities at once
    angle_rad = np.radians(angle_deg)
    vx = v0_tests * np.cos(angle_rad)
    vy = v0_tests * np.sin(angle_rad)
    
    x = np.zeros_like(v0_tests)
    y = np.full_like(v0_tests, shot_height_m)
    
    dt = 0.01
    steps = 400 # 4 seconds max flight
    
    x_at_top = np.full_like(v0_tests, np.nan)
    x_at_check = np.full_like(v0_tests, np.nan)
    
    k = 0.5 * AIR_DENSITY * Cd * BALL_AREA / BALL_MASS
    
    # --- TIME STEPPING (Vectorized) ---
    for _ in range(steps):
        # Drag Calc
        v = np.sqrt(vx**2 + vy**2)
        ax = -k * v * vx
        ay = -G - (k * v * vy)
        
        x_prev = x.copy()
        y_prev = y.copy()
        
        # Euler Step
        x += vx * dt
        y += vy * dt
        vx += ax * dt
        vy += ay * dt
        
        # Check Top Crossing (downwards)
        cross_top_mask = (y_prev >= HUB_TOP_H) & (y < HUB_TOP_H)
        if np.any(cross_top_mask):
            frac = (HUB_TOP_H - y[cross_top_mask]) / (y_prev[cross_top_mask] - y[cross_top_mask])
            x_at_top[cross_top_mask] = x[cross_top_mask] + (x_prev[cross_top_mask] - x[cross_top_mask]) * frac
            
        # Check Depth Crossing (downwards)
        cross_check_mask = (y_prev >= check_h) & (y < check_h)
        if np.any(cross_check_mask):
            frac = (check_h - y[cross_check_mask]) / (y_prev[cross_check_mask] - y[cross_check_mask])
            x_at_check[cross_check_mask] = x[cross_check_mask] + (x_prev[cross_check_mask] - x[cross_check_mask]) * frac

    # --- MATCHING TO DISTANCES ---
    results_rpm = []
    results_valid = []
    results_qual = []
    
    for target_dist in distances:
        # 1. Calculate horizontal error from center axis
        err_top = np.abs(x_at_top - target_dist)
        err_check = np.abs(x_at_check - target_dist)
        
        # 2. Check Constraints
        valid_top = err_top < (R_TOP - BALL_RADIUS)
        valid_check = err_check < max_safe_r
        
        valid_shots = valid_top & valid_check
        
        if np.any(valid_shots):
            # If multiple velocities work, pick the median valid one (safest shot)
            valid_indices = np.where(valid_shots)[0]
            best_idx = valid_indices[len(valid_indices)//2] 
            
            best_v0 = v0_tests[best_idx]
            
            # --- QUALITY CALCULATION ---
            # How close is the best shot to the center?
            # 0 error = 1.0 (Green), Max error = 0.0 (Red)
            best_err = err_check[best_idx]
            qual = 1.0 - (best_err / max_safe_r)
            qual = np.clip(qual, 0.0, 1.0)
            
            # Convert to RPM
            v_wheel = best_v0 / efficiency
            rpm = (v_wheel * 60) / WHEEL_CIRCUM
            
            if rpm <= max_rpm:
                results_rpm.append(rpm)
                results_valid.append(True)
                results_qual.append(qual)
            else:
                results_rpm.append(np.nan)
                results_valid.append(False)
                results_qual.append(np.nan)
        else:
            results_rpm.append(np.nan)
            results_valid.append(False)
            results_qual.append(np.nan)
            
    return results_rpm, results_valid, results_qual