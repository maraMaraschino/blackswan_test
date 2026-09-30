from mds_api import mds_api
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

# How frequently to sample MDS [s]
check_interval = 5.0

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
    if risk == "SAFE":
        print("ACTION: Continue normal operations.")

    elif risk == "CAUTION":
        print("ACTION: Increase monitoring frequency.")

    elif risk == "CRITICAL":
        print("ACTION: Avoidance response required.")

def monitor():
    """
    
    """

    configure_monitor()

    previous_range = None
    previous_jd  = None

    print("\nCollision monitor started.\n")

    while True:
        try:
            # Advance simulation by one second
            mds_api.step_sim(5, True)

            # Read current state
            sim_jd, current_range = get_range()

            # First measurement
            if previous_range is None:
                closing_velocity = 0.0
                predicted_time   = None

                # Update distance and time
                previous_range = current_range
                previous_jd  = sim_jd

                risk = "SAFE"
                print(f"Initial risk:  {risk}")
                print(f"Initial range: {current_range/1000:.2f} km")

            else:
                # Convert jd time from days to seconds
                elapsed          = (sim_jd - previous_jd) * 86400
                closing_velocity = calculate_closing_velocity(previous_range, current_range, elapsed)
                predicted_time   = predict_collision_time(current_range, closing_velocity)
                risk             = determine_risk(current_range, predicted_time)

                # Display current state
                print("-" * 40)
                print(f"Julian date: {sim_jd}")
                print(f"Range:\n{current_range/1000:.3f} km")
                print(f"Closing Velocity:\n{closing_velocity:.3f} m/s")
                print(f"Predicted boundary crossing time:")
                print("N/A" if predicted_time is None else f"{predicted_time:.1f} s")
                print(f"Risk: {risk}")

                autonomous_response(risk)

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
            time.sleep(check_interval)
            if error_count >= 5:
                print("Too many consecutive errors. Stopping monitor.")
                break

if __name__ == "__main__":
    print("""
Two satellites are in a normal safe range until an event, at which point a 
collision would become inevitable if not for autonomous systems preventing it.
""")

    # Scene setup
    mds_api.clear_scene()
    mds_api.enable_API_synchronization()
    mds_api.set_utc_date(2026, 1, 1, 12, 0, 0, 0)
    mds_api.set_simulation_timestep(1)


    # Create satellites (using values from example_constraint_monitor_correction.py)
    print("Adding satellites to scene...")

    # Thruster settings (values from example_constraint_monitor_correction.py)
    Tmax_g1  = 1.0 # N
    dVmax_g1 = 200.0 # m/s
    Isp_g1   = 2000.0 # s

    for n, sat_name in enumerate(sat_list):
        mds_api.add_sat_from_elements(
            sat_name,
            100,            # Mass [kg]
            7e6 + (5000*n), # Semi-major axis [m]
            1e-3,           # Eccentricity
            65,             # Inclination
            27,             # Right ascension
            32,             # Periapsis [deg]
            1 + 1*n,        # True anomoly [deg]
            "Earth"         # Central body
        )
        # Give satellites thrusters
        mds_api.add_thruster(
            sat_name,
            Tmax_g1,      # Thrust max
            dVmax_g1 * n, # delta V
            Isp_g1,       # isp
            1000,         # burn duration limit
            1.0 + 2*n     # fuel consumption rate
        )
        print(f"Added {sat_name} to scene...")
    
    monitor()