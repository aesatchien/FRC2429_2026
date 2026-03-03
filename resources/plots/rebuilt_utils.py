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

            # Penalize near-side shots (short) more than far-side shots
            best_x = x_at_check[best_idx]
            signed_err = best_x - target_dist

            if signed_err < 0:
                weighted_err = abs(signed_err) * 1.5
            else:
                weighted_err = abs(signed_err)

            qual = 1.0 - (weighted_err / max_safe_r)
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

# simulations
# --- CONFIGURATION ---
default_config = {
    "SLIP": 0.25,
    "SHOT_HEIGHT_IN": 20,
    "MAX_RPM": 4500,
    "BOUNCE_DEPTH_IN": 10.0,
    "ANGLES": np.arange(50, 86, 1),
    "DISTANCES": np.arange(0.5, 6.1, 0.1)
}


# --- VECTORIZED SIMULATION ---
def vector_sim(config=None):
    if config is None:
        config = default_config

    SLIP = config["SLIP"]
    SHOT_HEIGHT_IN = config["SHOT_HEIGHT_IN"]
    MAX_RPM = config["MAX_RPM"]
    BOUNCE_DEPTH_IN = config["BOUNCE_DEPTH_IN"]
    ANGLES = config["ANGLES"]
    DISTANCES = config["DISTANCES"]

    # Derived values
    CHECK_DEPTH_M = BOUNCE_DEPTH_IN * 0.0254
    CHECK_H = HUB_TOP_H - CHECK_DEPTH_M
    slope = (R_TOP - R_BOT) / (HUB_TOP_H - HUB_BOT_H)
    CHECK_R = R_TOP - (slope * CHECK_DEPTH_M)
    EFFICIENCY = 1.0 - SLIP
    MAX_SAFE_R = CHECK_R - BALL_RADIUS

    print(f"Config: Depth {BOUNCE_DEPTH_IN}\" ({CHECK_DEPTH_M:.2f}m) | Target Radius {CHECK_R/0.0254:.2f}\"")

    # --- RUN ANALYSIS ---
    grid_rpm = np.zeros((len(ANGLES), len(DISTANCES)))
    grid_qual = np.zeros((len(ANGLES), len(DISTANCES)))
    mask = np.zeros((len(ANGLES), len(DISTANCES)), dtype=bool)

    print(f"Calculating {len(ANGLES)} angles...")

    for i, ang in enumerate(ANGLES):
        rpms, valids, quals = solve_angle_batch(ang, DISTANCES, SHOT_HEIGHT_IN * 0.0254, CHECK_H, MAX_SAFE_R, EFFICIENCY, MAX_RPM)
        grid_rpm[i, :] = rpms
        grid_qual[i, :] = quals
        mask[i, :] = valids

        print(f"\rAngle {ang}° done.", end="")

    print("\nCalculation Complete.")

    # --- PLOTTING ---
    fig = plt.figure(figsize=(14, 12))
    plt.suptitle(
        f"Shooter Analysis (Vectorized Solver)\nDepth Check: {BOUNCE_DEPTH_IN}\" | Slip: {SLIP} | Max RPM: {MAX_RPM}",
        fontsize=16)

    # 1. Heatmap (RPM)
    ax1 = fig.add_subplot(2, 1, 1)
    X, Y = np.meshgrid(DISTANCES, ANGLES)
    mesh = ax1.pcolormesh(X, Y, np.ma.masked_where(~mask, grid_rpm), cmap='viridis', shading='auto', vmin=1000,
                          vmax=MAX_RPM)
    cbar = fig.colorbar(mesh, ax=ax1)
    cbar.set_label("Required RPM")
    ax1.set_title("Required RPM (White = Miss)")
    ax1.set_ylabel("Shooter Angle (deg)")
    ax1.set_xlabel("Distance (m)")

    # 2. Valid Range by Angle (PROPER HEATMAP GRID)
    ax2 = fig.add_subplot(2, 1, 2)

    # Custom Colormap: Red (Grazing) -> Yellow -> Green (Centered)
    cmap_qual = mcolors.LinearSegmentedColormap.from_list("ShotQuality", ["#FF3333", "#FFCC00", "#00CC00"])

    # We swap axes for this plot: X=Angle, Y=Distance
    # Create meshgrid for Transposed view: X=ANGLES, Y=DISTANCES
    X2, Y2 = np.meshgrid(ANGLES, DISTANCES)

    # Transpose the data grids to match (Distances x Angles) -> (Angles as X, Distances as Y)
    grid_qual_T = grid_qual.T
    mask_T = mask.T

    # Render using pcolormesh instead of scatter
    mesh2 = ax2.pcolormesh(X2, Y2, np.ma.masked_where(~mask_T, grid_qual_T),
                           # cmap=cmap_qual,
                           cmap='RdYlGn',
                           shading='auto',
                           vmin=0, vmax=1.0)

    cbar2 = fig.colorbar(mesh2, ax=ax2)
    cbar2.set_label("Shot Quality (Centering)")
    cbar2.set_ticks([0, 0.5, 1.0])
    cbar2.set_ticklabels(['Grazing', 'Okay', 'Perfect'])

    # Overlay Min/Max lines for clarity
    mins, maxs = [], []
    for i in range(len(ANGLES)):
        dists = DISTANCES[mask[i, :]]
        if len(dists) > 0:
            mins.append(dists.min())
            maxs.append(dists.max())
        else:
            mins.append(np.nan)
            maxs.append(np.nan)

    ax2.plot(ANGLES, mins, 'k-', lw=1.5, alpha=0.5, label='Min/Max Range')
    ax2.plot(maxs, ANGLES, 'k-', lw=1.5, alpha=0.5)

    # Find Best Angle
    valid_widths = np.array(maxs) - np.array(mins)
    valid_widths[np.isnan(valid_widths)] = -1
    best_idx = np.argmax(valid_widths)
    best_ang = ANGLES[best_idx]
    ax2.axvline(best_ang, color='blue', ls='--', lw=2, label=f"Best Angle: {best_ang}°")

    ax2.set_title("Valid Range vs. Angle (Color = Centering Quality)")
    ax2.set_xlabel("Shooter Angle (deg)")
    ax2.set_ylabel("Distance (m)")
    ax2.set_ylim(0, 7)
    ax2.set_xlim(50, 86)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def generate_quality_heatmap(config=None):
    if config is None:
        config = default_config

    SLIP = config["SLIP"]
    SHOT_HEIGHT_IN = config["SHOT_HEIGHT_IN"]
    MAX_RPM = config["MAX_RPM"]
    BOUNCE_DEPTH_IN = config["BOUNCE_DEPTH_IN"]
    ANGLES = config["ANGLES"]
    DISTANCES = config["DISTANCES"]

    # Derived values
    CHECK_DEPTH_M = BOUNCE_DEPTH_IN * 0.0254
    CHECK_H = HUB_TOP_H - CHECK_DEPTH_M
    slope = (R_TOP - R_BOT) / (HUB_TOP_H - HUB_BOT_H)
    CHECK_R = R_TOP - (slope * CHECK_DEPTH_M)
    EFFICIENCY = 1.0 - SLIP
    MAX_SAFE_R = CHECK_R - BALL_RADIUS

    # --- 1. GENERATE THE 2D GRID DATA ---
    # We need a 2D matrix (Angles x Distances) for the heatmap.
    # Initialize with NaNs so invalid areas are transparent.
    quality_grid = np.full((len(ANGLES), len(DISTANCES)), np.nan)
    rpm_grid = np.full((len(ANGLES), len(DISTANCES)), np.nan)

    print(f"Calculating sweep for {len(ANGLES)} angles...")

    for i, angle in enumerate(ANGLES):
        # Capture the return values from your function
        # Assuming solve_angle_batch returns: (rpms, valid_mask, qualities)
        r, v, q = solve_angle_batch(angle, DISTANCES, SHOT_HEIGHT_IN * 0.0254, CHECK_H, MAX_SAFE_R, EFFICIENCY, MAX_RPM)

        # Fill the row in our 2D grid
        # We only want to fill in the Valid shots. 
        # If your function returns NaNs for invalid shots in 'q', you can assign directly.
        # Otherwise, we use the 'v' (valid_mask) to filter.

        # Convert lists to numpy arrays for indexing
        row_q = np.array(q)
        row_r = np.array(r)
        row_v = np.array(v, dtype=bool)

        # Assign valid data to the grid
        quality_grid[i, row_v] = row_q[row_v]
        rpm_grid[i, row_v] = row_r[row_v]

    print("Calculation done. Plotting...")

    # --- 2. PLOT WITH PCOLORMESH ---
    fig, ax = plt.subplots(figsize=(10, 6))

    # Create the Meshgrid for the axes
    # X = Distances (Columns), Y = Angles (Rows)
    X, Y = np.meshgrid(DISTANCES, ANGLES)

    # Render the Heatmap
    # vmin=0, vmax=1 locks the color scale to your 0-1 quality metric
    # shading='nearest' ensures each grid cell is colored solidly
    mesh = ax.pcolormesh(X, Y, quality_grid,
                         cmap='RdYlGn',
                         vmin=0, vmax=1.0,
                         shading='nearest')

    # Add Colorbar
    cbar = fig.colorbar(mesh, ax=ax)
    cbar.set_label("Shot Centering Quality (1.0 = Perfect)")
    cbar.set_ticks([0, 0.5, 1.0])
    cbar.set_ticklabels(['Rim Grazer', 'Okay', 'Dead Center'])

    # Overlays (Optional: Draw the min/max range lines)
    # We can calculate the min/max distance for each angle from the mask
    valid_mask = ~np.isnan(quality_grid)
    min_dists = []
    max_dists = []

    for i in range(len(ANGLES)):
        row_dists = DISTANCES[valid_mask[i, :]]
        if len(row_dists) > 0:
            min_dists.append(row_dists.min())
            max_dists.append(row_dists.max())
        else:
            min_dists.append(np.nan)
            max_dists.append(np.nan)

    ax.plot(min_dists, ANGLES, 'k-', lw=1.5, alpha=0.5, label='Min/Max Range')
    ax.plot(max_dists, ANGLES, 'k-', lw=1.5, alpha=0.5)

    # Formatting
    ax.set_title(f"Shot Quality Heatmap (Depth: {BOUNCE_DEPTH_IN}\")")
    ax.set_xlabel("Distance to Target (m)")
    ax.set_ylabel("Shooter Angle (deg)")
    ax.set_xlim(0.5, 6.0)
    ax.set_ylim(50, 86)
    ax.grid(True, alpha=0.2)

    plt.show()
