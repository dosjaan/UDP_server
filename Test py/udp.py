
import socket

STM32_IP = "192.168.10.20"
STM32_PORT = 5000

SPEED_MIN = -500
SPEED_MAX = 500
STEER_MIN = -45000
STEER_MAX = 45000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

seq = 1

try:
    while True:
        try:
            speed = int(input("Speed setpoint (mm/s): "))
            steering = int(input("Steering (millidegrees): "))
        except ValueError:
            print("ERR: enter integer values")
            continue

        if not SPEED_MIN <= speed <= SPEED_MAX:
            print("ERR: SPEED_RANGE")
            continue

        if not STEER_MIN <= steering <= STEER_MAX:
            print("ERR: STEER_RANGE")
            continue

        packet = f"CMD,{seq},{speed},{steering}".encode("ascii")

        # Retry the exact same packet if no response arrives.
        for attempt in range(3):
            sock.sendto(packet, (STM32_IP, STM32_PORT))

            try:
                data, addr = sock.recvfrom(256)
                response = data.decode("ascii", errors="replace")
                print(f"STM32: {response} From: {addr}")

                if response.startswith(f"ACK,{seq},"):
                    seq = (seq + 1) & 0xFFFFFFFF
                    break

                if response.startswith("ERR,"):
                    break

            except socket.timeout:
                print(f"Timeout; retry {attempt + 1}/3")

        else:
            print("No response after retries")

finally:
    sock.close()
