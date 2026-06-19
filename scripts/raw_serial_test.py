import serial
import time


PORT = "/dev/ttyUSB0"
BAUDRATE = 230400
READ_SIZE = 256
DURATION_SECONDS = 5


def main():
    print("COVENANT Ground Station - Raw Serial LiDAR Test")
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Duration: {DURATION_SECONDS} seconds")
    print()

    total_bytes = 0
    start_time = time.time()

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print("Serial port opened successfully.")
            print("Reading raw bytes...")
            print()

            while time.time() - start_time < DURATION_SECONDS:
                data = ser.read(READ_SIZE)

                if data:
                    total_bytes += len(data)

                    # Affiche seulement les 32 premiers octets pour garder un output lisible
                    print(data[:32].hex(" "))

    except serial.SerialException as error:
        print("Serial error:")
        print(error)
        print()
        print("Possible causes:")
        print("- wrong port")
        print("- missing permissions")
        print("- LiDAR not connected")
        print("- another program is already using the port")
        return

    print()
    print(f"Total bytes received: {total_bytes}")

    if total_bytes == 0:
        print("Result: no data received.")
        print("Check port, baudrate, cable, LiDAR power, or permissions.")
    else:
        print("Result: LiDAR is sending data.")


if __name__ == "__main__":
    main()
