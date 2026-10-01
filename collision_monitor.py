from mds_api import mds_api
import numpy as np
import math
import time

satellite_a = "Sat A"
satellite_b = "Sat B"

sat_list = [satellite_a, satellite_b]

# Safety boundary [m]
safety_distance = 5000

# Distance at which to pay closer attention [m]
caution_distance = 10000

# How far in future to look [s]
prediction_window = 120

# How frequently to print current state [s]
report_interval = 10.0

# How long to apply anomalous thruster burn [s]
burn_duration = 30 

# Thruster settings
Tmax_g1  = 30.0 # N
dVmax_g1 = 200.0 # m/s
Isp_g1   = 2000.0 # s

# Anomalous thrust value
burn_force = 100.0 # N

# MDS Setup
def configure_monitor():
    """
    Configure MDS to monitor distance between Sat A and B.
    """

    mds_api.set_constraint(
            satellite_a,            # Sat name
            6,                      # Range to target
            [safety_distance, 1e9], # [min, max] Distance constraint [m]
            satellite_b,            # Relative target
        )

    print(f"Monitoring distance between:\n{satellite_a} and {satellite_b}")
    print(f"Safety boundary:\n{safety_distance/1000:.2f} km")

# Set true anomaly for desired distance
def set_true_anomaly(distance, altitude):
    """
    Calculate true-anomaly separation required for two satellites on same circular orbit
    to have a specified distance along a cord.

    distance: desired separation [km]
    altitude: desired orbital altitude [km]
    """
    # Convert distance and altitude to m
    distance = distance * 1000
    altitude = altitude * 1000
    r_earth = 6371 * 1000 # m

    angle_rad = 2 * np.arcsin(distance / (2 * (altitude + r_earth)))
    return math.degrees(angle_rad)

def get_retrograde_thrust(sat_name, thrust_mag):
    jd, pos, vel = mds_api.get_sat_pos_vel(sat_name)

    vx, vy, vz = vel

    speed = math.sqrt(vx**2 + vy**2 + vz**2)
    if speed == 0:
        raise RuntimeError("Satellite velocity is zero.")

    thrust_vector = [
        -thrust_mag * vx / speed,
        -thrust_mag * vy / speed,
        -thrust_mag * vz / speed,
    ]

    return thrust_vector

def apply_anomalous_burn(sat_name, duration_s, thrust_mag):
    for _ in range(duration_s):
        thrust_vector = get_retrograde_thrust(sat_name, thrust_mag)
        mds_api.apply_thrust(sat_name, thrust_vector, inertial=False)
        mds_api.step_sim(1, True)

# Read current range from MDS
def get_range():
    """
    Read current range-to-target constraint from MDS
    """

    jd, constraints = mds_api.get_spacecraft_constraints(satellite_a, satellite_b)

    for sat, entries in constraints.items():
        for constraint in entries:
            if constraint.get("type") == 6:
                range_m = float(constraint["actualValue"])
                return jd, range_m

    raise RuntimeError("Range-to-target not returned by MDS.")

def calculate_closing_velocity(previous_range, current_range, elapsed_seconds):
    """
    Calculate radial closing velocity.
    Positive value = satellites getting closer
    Negative value = satellites drifitng apart
    """
    if elapsed_seconds <= 0:
        return 0.0

    distance_change = previous_range - current_range
    return distance_change / elapsed_seconds

def predict_collision_time(current_range, closing_velocity):
    """
    Estimate time until satellites reach safety boundary.
    """
    if closing_velocity <= 0:
        return None
    distance_remaining = current_range - safety_distance
    if distance_remaining <=0:
        return 0.0

    return distance_remaining / closing_velocity

def determine_risk(current_range, predicted_time):
    """
    Set risk determination:
    SAFE: Satellites outside set caution distance
    CAUTION: Satellites inside caution distance
    CRITICAL: Satellite already in or predicted to cross safety boundary
    """

    # Already inside safety boundary
    if current_range <= safety_distance:
        return "CRITICAL"

    # Predicted to cross boundary soon
    if (predicted_time is not None and predicted_time <= prediction_window):
        return "CRITICAL"

    # Satellite in caution boundary 
    if current_range <= caution_distance:
        return "CAUTION"

    return "SAFE"
    
def autonomous_response(risk):
    # Actions to be replaced with code to trigger autonomous responses.
    if risk == "SAFE":
        print("ACTION: Continue normal operations.")

    elif risk == "CAUTION":
        print("ACTION: Increase monitoring frequency.")

    elif risk == "CRITICAL":
        print("ACTION: Avoidance response required.")

def monitor():
    """
    Monitor satellites and printout data related to their respective velocities, boundary crossing times,
    current risk assessment, and current recommended operation actions. After 120 seconds, trigger 
    anomalous burn event.
    """

    configure_monitor()

    error_count = 0

    previous_range = None
    previous_jd  = None

    print("\nCollision monitor started.\n")
    sim_jd, current_range = get_range()
    init_jd = sim_jd

    # Burn event values
    event_triggered = False
    burn_active     = False
    burn_end_time   = None

    while True:
        try:
            # Advance simulation by one second
            mds_api.step_sim(1, True)

            # Read current state
            sim_jd, current_range = get_range()

            # Total elapsed time
            elapsed_since_init    = (sim_jd - init_jd) * 86400

            if not event_triggered and elapsed_since_init >= 120:
                event_triggered = True
                burn_active     = True
                burn_end_time   = elapsed_since_init + burn_duration

                thrust_vector = get_retrograde_thrust(satellite_b, burn_force)
                print("\nEVENT: Sat B trajectory deviation!")

            # Apply burn
            if burn_active:
                if elapsed_since_init < burn_end_time:
                    mds_api.apply_thrust(satellite_b, thrust_vector, inertial=True)

                else:
                    burn_active = False
                    print("EVENT: Sat B burn complete.")

            # First measurement
            if previous_range is None:

                closing_velocity = 0.0
                predicted_time   = None

                risk = determine_risk(current_range, predicted_time)

                # Update distance and time
                previous_range = current_range
                previous_jd    = sim_jd

                print(f"Initial risk:  {risk}")
                print(f"Initial range: {current_range/1000:.2f} km")

                # Set interval to print reports
                next_report_time = report_interval

            else:
                # Convert jd time from days to seconds
                elapsed          = (sim_jd - previous_jd) * 86400
                closing_velocity = calculate_closing_velocity(previous_range, current_range, elapsed)
                predicted_time   = predict_collision_time(current_range, closing_velocity)
                risk             = determine_risk(current_range, predicted_time)

            # Display current state at reporting intervals
            if elapsed_since_init >= next_report_time:
                print("-" * 40)
                print(f"Julian date: {sim_jd}")
                print(f"Elapsed time: {(sim_jd - init_jd) * 86400:.2f} s")
                print(f"Range:\n{current_range/1000:.3f} km")
                print(f"Closing Velocity:\n{closing_velocity:.3f} m/s")
                print(f"Predicted boundary crossing time:")
                print("N/A" if predicted_time is None else f"{predicted_time:.1f} s")
                print(f"Risk: {risk}")

                autonomous_response(risk)
                next_report_time += report_interval
                #get_retrograde_thrust(satellite_b)

            # Save current state
            previous_range = current_range
            previous_jd = sim_jd

            error_count = 0

        except KeyboardInterrupt:
            print("\nCollision monitor stopped.")
            break
        except Exception as error:
            error_count += 1
            print(f"Monitor error {error_count}/5: {error}")
            time.sleep(5)
            if error_count >= 5:
                print("Too many consecutive errors. Stopping monitor.")
                break

if __name__ == "__main__":
    print("""
Two satellites begin in a safe orbital configuration. After 120 seconds, an anomalous maneuver causes Sat B to deviate
from its original trajectory. The monitoring system tracks their separation and closing velocity, predicts whether they
will cross the 5 km safety boundary, and determines when an avoidance response is required.
""")

    # Scene setup
    mds_api.clear_scene()
    mds_api.enable_API_synchronization()
    mds_api.set_utc_date(2026, 1, 1, 12, 0, 0, 0)
    mds_api.set_simulation_timestep(1)

    print("Adding satellites to scene...")

    # Define constants
    init_distance = 15                                        # km
    init_alt      = 800                                       # km
    radius_earth  = 6371                                      # km
    semi_maj_a    = radius_earth + init_alt                   # km
    init_nu       = set_true_anomaly(init_distance, init_alt) # deg

    for n, sat_name in enumerate(sat_list):
        mds_api.add_sat_from_elements(
            sat_name,
            100,               # Mass [kg]
            semi_maj_a * 1000, # Semi-major axis [m]
            0,                 # Eccentricity
            180,               # Inclination
            90,                # Right ascension
            90,                # Periapsis [deg]
            init_nu * n,       # True anomoly [deg]
            "Earth"            # Central body
        )
        
        # Give satellites thrusters
        mds_api.add_thruster(
            sat_name,
            Tmax_g1,  # Thrust max
            dVmax_g1, # delta V
            Isp_g1,   # isp
            1000,     # burn duration limit
            5.0       # fuel consumption rate
        )
        print(f"Added {sat_name} to scene...")
    
    monitor()