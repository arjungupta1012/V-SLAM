from dronekit import connect, VehicleMode, APIException
import time

def get_rangefinder_distance(vehicle):
    if vehicle.rangefinder:
        return vehicle.rangefinder.distance
    else:
        print("Rangefinder not available on this vehicle.")
        return None

def main():
    connection_string = '127.0.0.1:14444'
    print(f"Connecting to vehicle on: {connection_string}")

    try:
        vehicle = connect(connection_string)
        print("Connection established")
    except APIException as e:
        print(f"APIException occurred: {e}")
        return
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return

    try:
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        while True:
            distance = get_rangefinder_distance(vehicle)
            if distance is not None:
                print(f"Rangefinder distance: {distance} meters")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
