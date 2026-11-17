"""Two-car PI following on a flat road or a hill.
Run: python motorway_slope.py
Requires matplotlib. No input data or other project files are needed.
This is an isolated elevation extension of the team's PI following rules;
straight-road lane-changing and obstacle planning are not simulated here.
"""
import csv
import json
import math
from pathlib import Path
import matplotlib.pyplot as plt

# Same vehicle, following-distance and PI settings as the team's 2D model.
MASS = 1500.0
GRAVITY = 9.81
AIR_DENSITY = 1.225
DRAG_COEFFICIENT = 0.3
FRONTAL_AREA = 2.2
ROLLING_COEFFICIENT = 0.015
CAR_LENGTH = 4.5
MINIMUM_GAP = 5.0
TIME_HEADWAY = 2.5
GAP_GAIN = 0.5
KP = 0.4
KI = 0.05
SPEED_LIMIT = 70 * 0.44704
LEAD_SPEED = SPEED_LIMIT  # 70 mph = 31.2928 m/s throughout the run
MAX_DRIVE_FORCE = 4000.0
MAX_BRAKE_FORCE = 8000.0
ACTUATOR_TIME_CONSTANT = 0.3
DURATION = 100.0
OUTPUT = Path(__file__).resolve().parent / 'motorway_slope_results'


def road_angle(distance, grade_amplitude):
    # A smooth uphill then downhill, with flat road before and afterwards.
    # Positive grade is uphill, negative grade is downhill; 0.06 means 6%.
    grade = 0.0
    if 200.0 < distance < 800.0:
        grade = grade_amplitude * math.sin(2 * math.pi * (distance - 200) / 600)
    return math.atan(grade)


def simulate(grade_amplitude=0.06, dt=0.1):
    # s measures distance ALONG the road, not horizontal distance.
    follower_s = 0.0
    follower_speed = LEAD_SPEED
    desired_gap = MINIMUM_GAP + TIME_HEADWAY * follower_speed
    lead_s = follower_s + CAR_LENGTH + desired_gap
    drive_work = 0.0
    brake_work = 0.0
    resistance_work = 0.0
    gravity_work = 0.0
    history = []
    previous_source = 'free' if LEAD_SPEED >= SPEED_LIMIT else 'traffic'

    # Coordinate lookup for the single lane: y=0; x and z follow the hill.
    # These 1 m geometry samples do not change the along-road dynamics.
    road_x = [0.0]
    road_z = [0.0]
    for i in range(1, math.ceil(lead_s + SPEED_LIMIT * DURATION + 100) + 1):
        angle = road_angle(i - 0.5, grade_amplitude)
        road_x.append(road_x[-1] + math.cos(angle))
        road_z.append(road_z[-1] + math.sin(angle))


    # Start at a balanced flat-road cruising state, avoiding a start-up transient.
    flat_resistance = (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA
                       * follower_speed**2 + ROLLING_COEFFICIENT * MASS * GRAVITY)
    actuator_command = flat_resistance / MASS
    integral_error = actuator_command / KI

    for step in range(round(DURATION / dt)):
        time = step * dt
        follower_angle = road_angle(follower_s, grade_amplitude)
        lead_angle = road_angle(lead_s, grade_amplitude)
        gap = lead_s - follower_s - CAR_LENGTH
        desired_gap = MINIMUM_GAP + TIME_HEADWAY * follower_speed
        gap_error = gap - desired_gap

        # Existing outer following rule: request extra speed if the gap is too big.
        traffic_reference = max(0.0, LEAD_SPEED + GAP_GAIN * gap_error)
        target_speed = min(SPEED_LIMIT, traffic_reference)
        # Tolerance avoids switching modes because of floating-point round-off.
        source = 'free' if traffic_reference >= SPEED_LIMIT - 1e-9 else 'traffic'
        if source != previous_source:
            integral_error = 0.0
        previous_source = source
        speed_error = target_speed - follower_speed

        # PI controller with the same conditional-integration rule as the 2D model.
        minimum_command = max(-8.0, -MAX_BRAKE_FORCE / MASS)
        maximum_command = min(2.0, MAX_DRIVE_FORCE / MASS)
        trial_integral = integral_error + speed_error * dt
        trial_command = KP * speed_error + KI * trial_integral
        close_to_target = abs(speed_error) <= 1.0
        unwinding = speed_error * integral_error < 0
        saturated_high = trial_command >= maximum_command and speed_error > 0
        saturated_low = trial_command <= minimum_command and speed_error < 0
        if (close_to_target or unwinding) and not saturated_high and not saturated_low:
            integral_error = trial_integral
        command = KP * speed_error + KI * integral_error
        command = max(minimum_command, min(maximum_command, command))

        # Preserve the existing emergency-braking threshold.
        closing_speed = follower_speed - LEAD_SPEED
        emergency = follower_speed > 0.1 and (
            gap <= MINIMUM_GAP or (closing_speed > 0 and gap / closing_speed < 1))
        if emergency:
            command = minimum_command
            integral_error = 0.0

        # Actuator lag and physical forces; there is NO slope feedforward here.
        response_fraction = 1 - math.exp(-dt / ACTUATOR_TIME_CONSTANT)
        actuator_command += response_fraction * (command - actuator_command)
        drive_force = MASS * max(actuator_command, 0.0)
        brake_force = MASS * max(-actuator_command, 0.0)
        drag_force = (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA
                      * follower_speed**2)
        rolling_force = ROLLING_COEFFICIENT * MASS * GRAVITY * math.cos(follower_angle)
        slope_force = MASS * GRAVITY * math.sin(follower_angle)
        net_force = drive_force - brake_force - drag_force - rolling_force - slope_force
        acceleration = net_force / MASS

        # The lead speed is prescribed. This is the effort needed to maintain it
        # at the lead car's OWN local slope; it is not a second PI controller.
        lead_required_force = (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA
                               * LEAD_SPEED**2
                               + ROLLING_COEFFICIENT * MASS * GRAVITY * math.cos(lead_angle)
                               + MASS * GRAVITY * math.sin(lead_angle))
        if not -MAX_BRAKE_FORCE <= lead_required_force <= MAX_DRIVE_FORCE:
            raise RuntimeError('Prescribed lead motion exceeds assumed force limits.')

        # Motion along the road, including stopping partway through a step.
        move_time = dt
        if acceleration < 0 and follower_speed + acceleration * dt < 0:
            move_time = -follower_speed / acceleration
        travelled = follower_speed * move_time + 0.5 * acceleration * move_time**2
        next_speed = max(0.0, follower_speed + acceleration * dt)
        drive_work += drive_force * travelled
        brake_work += brake_force * travelled
        resistance_work += (drag_force + rolling_force) * travelled
        gravity_work += slope_force * travelled
        # Interpolate each car's (x,y,z) coordinates from its own road distance.
        follower_index = int(follower_s)
        lead_index = int(lead_s)
        follower_fraction = follower_s - follower_index
        lead_fraction = lead_s - lead_index
        follower_x = road_x[follower_index] + follower_fraction * (road_x[follower_index+1] - road_x[follower_index])
        follower_z = road_z[follower_index] + follower_fraction * (road_z[follower_index+1] - road_z[follower_index])
        lead_x = road_x[lead_index] + lead_fraction * (road_x[lead_index+1] - road_x[lead_index])
        lead_z = road_z[lead_index] + lead_fraction * (road_z[lead_index+1] - road_z[lead_index])
        history.append({
            'follower_x_m': follower_x, 'follower_y_m': 0.0, 'follower_z_m': follower_z,
            'lead_x_m': lead_x, 'lead_y_m': 0.0, 'lead_z_m': lead_z,
            'time_s': time, 'follower_s_m': follower_s, 'lead_s_m': lead_s,
            'follower_speed_m_s': follower_speed, 'lead_speed_m_s': LEAD_SPEED,
            'target_speed_m_s': target_speed, 'gap_m': gap,
            'desired_gap_m': desired_gap, 'gap_error_m': gap_error,
            'acceleration_m_s2': acceleration, 'drive_force_N': drive_force,
            'brake_force_N': brake_force, 'follower_grade': math.tan(follower_angle),
            'lead_grade': math.tan(lead_angle), 'lead_required_force_N': lead_required_force,
            'emergency': emergency,
        })
        follower_s += travelled
        follower_speed = next_speed
        lead_s += LEAD_SPEED * dt

    errors = [row['gap_error_m'] for row in history]
    accelerations = [row['acceleration_m_s2'] for row in history]
    squared_speed_error = [(row['follower_speed_m_s'] - LEAD_SPEED)**2 for row in history]
    kinetic_change = 0.5 * MASS * (follower_speed**2 - LEAD_SPEED**2)
    summary = {
        'grade_amplitude': grade_amplitude, 'dt_s': dt, 'duration_s': DURATION,
        'gap_rmse_m': math.sqrt(sum(e**2 for e in errors) / len(errors)),
        'gap_shortfall_rmse_m': math.sqrt(sum(min(e, 0)**2 for e in errors) / len(errors)),
        'minimum_gap_m': min(row['gap_m'] for row in history),
        'maximum_gap_m': max(row['gap_m'] for row in history),
        'speed_rmse_m_s': math.sqrt(sum(squared_speed_error) / len(history)),
        'speed_min_m_s': min(row['follower_speed_m_s'] for row in history),
        'speed_max_m_s': max(row['follower_speed_m_s'] for row in history),
        'jerk_max_m_s3': max(abs(accelerations[i+1] - accelerations[i]) / dt
                             for i in range(len(accelerations) - 1)),
        'collision_samples': sum(row['gap_m'] <= 0 for row in history),
        'emergency_steps': sum(row['emergency'] for row in history),
        'follower_distance_m': follower_s,
        'final_speed_m_s': follower_speed,
        'drive_work_MJ': drive_work / 1e6,
        'final_gap_m': lead_s - follower_s - CAR_LENGTH,
        'final_gap_error_m': lead_s - follower_s - CAR_LENGTH - (MINIMUM_GAP + TIME_HEADWAY * follower_speed),
        'maximum_speed_excess_m_s': max(0.0, max(row['follower_speed_m_s'] for row in history) - SPEED_LIMIT),
        'brake_work_MJ': brake_work / 1e6,
        'energy_balance_error_J': (drive_work - brake_work - resistance_work
                                    - gravity_work - kinetic_change),
    }
    return summary, history


def main():
    OUTPUT.mkdir(exist_ok=True)
    summaries = []
    traces = {}
    for label, grade in [('Flat', 0.0), ('Hill', 0.06)]:
        result, rows = simulate(grade)
        result['scenario'] = label
        summaries.append(result)
        traces[label] = rows
        print(label, json.dumps(result, indent=2))
        with (OUTPUT / (label.lower() + '_trace.csv')).open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    checks = []
    for grade in [0.0, 0.03, 0.06, 0.09]:
        for dt in [0.1, 0.05]:
            result, rows = simulate(grade, dt)
            assert abs(result['energy_balance_error_J']) < 1e-4
            checks.append(result)
    assert summaries[0]['gap_rmse_m'] < 1e-8
    assert summaries[0]['speed_rmse_m_s'] < 1e-8
    (OUTPUT / 'summary.json').write_text(json.dumps({'main': summaries, 'checks': checks}, indent=2))

    # Road height is reconstructed from dz/ds=sin(theta), dx/ds=cos(theta).
    positions = [0.0]
    heights = [0.0]
    horizontal = [0.0]
    for i in range(1, 2201):
        angle = road_angle(i - 0.5, 0.06)
        positions.append(float(i))
        heights.append(heights[-1] + math.sin(angle))
        horizontal.append(horizontal[-1] + math.cos(angle))

    fig, axes = plt.subplots(4, 1, figsize=(7, 8), constrained_layout=True)
    axes[0].plot(positions, heights, color='black')
    axes[0].set(xlabel='Distance along road (m)', ylabel='Height (m)', xlim=(0, 1200))
    for label, rows in traces.items():
        times = [row['time_s'] for row in rows]
        axes[1].plot(times, [row['follower_speed_m_s'] / 0.44704 for row in rows], label=label)
        axes[2].plot(times, [row['gap_error_m'] for row in rows], label=label)
        axes[3].plot(times, [row['drive_force_N'] - row['brake_force_N'] for row in rows], label=label)
    axes[1].axhline(LEAD_SPEED / 0.44704, color='black', linestyle='--', linewidth=0.8,
                    label='Lead speed: 70 mph (prescribed)')
    axes[1].set_ylabel('Follower speed (mph)')
    axes[1].legend(fontsize=8)
    axes[2].axhline(0, color='grey', linewidth=0.8)
    axes[2].set_ylabel('Gap error (m)')
    axes[3].axhline(0, color='grey', linewidth=0.8)
    axes[3].set(ylabel='Drive - brake force (N)', xlabel='Time (s)')
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.savefig(OUTPUT / 'motorway_slope_results.png', dpi=180)
    fig.savefig(OUTPUT / 'motorway_slope_results.pdf')
    plt.close(fig)
    print('Saved plots, traces and results to', OUTPUT)


if __name__ == '__main__':
    main()
