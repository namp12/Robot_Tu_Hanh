import asyncio
import websockets
import serial
import serial.tools.list_ports
import json
import logging
import sys
import re
import math

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)

# WS Connected Clients
connected_clients = set()
serial_connection = None

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler angles (in degrees) to ROS 2 standard 4D Quaternion (x, y, z, w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return {
        "x": round(qx, 4),
        "y": round(qy, 4),
        "z": round(qz, 4),
        "w": round(qw, 4)
    }

def auto_detect_serial_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "COM5" in p.device:
            return p.device
    for p in ports:
        if "USB" in p.description or "CH340" in p.description or "Silicon Labs" in p.description:
            return p.device
    if ports:
        return ports[0].device
    return "COM5"

async def ws_handler(websocket):
    logging.info(f"WebSocket client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            logging.info(f"Received WS message: {message}")
            if message == "BEEP":
                if serial_connection and serial_connection.is_open:
                    serial_connection.write(b"beep\n")
                    logging.info("Sent text command 'beep' to ESP32 Serial")
            elif message.startswith("SET_MODE:"):
                try:
                    mode_val = int(message.split(":")[1])
                    mode_str = "manual" if mode_val == 0 else ("auto" if mode_val == 1 else "ros")
                    if serial_connection and serial_connection.is_open:
                        serial_connection.write(f"MODE {mode_str}\n".encode())
                        logging.info(f"Sent text command 'MODE {mode_str}' to ESP32 Serial")
                except Exception as e:
                    logging.error(f"Error parsing SET_MODE parameter: {e}")
            elif message.startswith("MOVE:"):
                cmd_str = message.split(":", 1)[1]
                if serial_connection and serial_connection.is_open:
                    serial_connection.write(f"{cmd_str}\n".encode())
                    logging.info(f"Sent text command '{cmd_str}' to ESP32 Serial")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)
        logging.info(f"WebSocket client disconnected: {websocket.remote_address}")

def parse_text_telemetry(line: str):
    """
    Parses telemetry text line:
    [TELEMETRY] MODE: MANUAL | STATUS: READY | BATTERY: 12.40V (95%) | FRONT_DISTANCE: 45.0cm | REAR_DISTANCE: 70.0cm | IMU: Yaw=12.5° Roll=1.2° Pitch=-0.5° | ENCODER: Dist=3.45m
    """
    try:
        mode_match = re.search(r"MODE:\s*(\w+)", line)
        status_match = re.search(r"STATUS:\s*(\w+)", line)
        battery_match = re.search(r"BATTERY:\s*([\d\.]+)V\s*\((\d+)%\)", line)
        front_match = re.search(r"FRONT_DISTANCE:\s*([\d\.]+)cm", line)
        rear_match = re.search(r"REAR_DISTANCE:\s*([\d\.]+)cm", line)
        imu_match = re.search(r"IMU:\s*Yaw=([\-\d\.]+)°\s*Roll=([\-\d\.]+)°\s*Pitch=([\-\d\.]+)°", line)
        encoder_match = re.search(r"ENCODER:\s*Dist=([\-\d\.]+)m", line)

        if not (mode_match and front_match and rear_match and imu_match):
            return None

        yaw = float(imu_match.group(1))
        roll = float(imu_match.group(2))
        pitch = float(imu_match.group(3))

        quaternion = euler_to_quaternion(roll, pitch, yaw)

        return {
            "mode": mode_match.group(1),
            "status": status_match.group(1) if status_match else "READY",
            "battery_volts": float(battery_match.group(1)) if battery_match else 0.0,
            "battery_pct": int(battery_match.group(2)) if battery_match else 0,
            "front_distance_cm": float(front_match.group(1)),
            "rear_distance_cm": float(rear_match.group(1)),
            "imu": {
                "yaw_deg": yaw,
                "roll_deg": roll,
                "pitch_deg": pitch,
                "quaternion": quaternion
            },
            "total_distance_m": float(encoder_match.group(1)) if encoder_match else 0.0
        }
    except Exception as e:
        logging.error(f"Error parsing text telemetry line: {e}")
        return None

async def serial_reader_loop():
    global serial_connection
    port = auto_detect_serial_port()
    baudrate = 115200

    logging.info(f"Attempting to open serial port {port} at {baudrate} baud...")
    while True:
        try:
            serial_connection = serial.Serial(port, baudrate, timeout=0.1)
            logging.info(f"Serial port {port} opened successfully!")
            break
        except Exception as e:
            logging.error(f"Failed to open serial port: {e}. Retrying in 2 seconds...")
            await asyncio.sleep(2)
        port = auto_detect_serial_port()

    line_buffer = ""
    while True:
        try:
            if not serial_connection.is_open:
                raise serial.SerialException("Serial port is closed")

            if serial_connection.in_waiting > 0:
                chunk = serial_connection.read(serial_connection.in_waiting).decode("utf-8", errors="ignore")
                line_buffer += chunk

                while "\n" in line_buffer:
                    line, line_buffer = line_buffer.split("\n", 1)
                    line = line.strip()
                    if line.startswith("[TELEMETRY]"):
                        telemetry = parse_text_telemetry(line)
                        if telemetry and connected_clients:
                            msg_str = json.dumps({"type": "telemetry", "data": telemetry})
                            ws_tasks = [client.send(msg_str) for client in connected_clients]
                            await asyncio.gather(*ws_tasks, return_exceptions=True)
            else:
                await asyncio.sleep(0.005)
        except Exception as e:
            logging.error(f"Error in serial loop: {e}. Reconnecting...")
            if serial_connection:
                try:
                    serial_connection.close()
                except:
                    pass
            await asyncio.sleep(2)
            while True:
                try:
                    port = auto_detect_serial_port()
                    serial_connection = serial.Serial(port, baudrate, timeout=0.1)
                    logging.info(f"Serial port {port} reconnected successfully!")
                    break
                except Exception as ex:
                    logging.error(f"Reconnection failed: {ex}. Retrying in 2 seconds...")
                    await asyncio.sleep(2)

async def main():
    ws_server = await websockets.serve(ws_handler, "0.0.0.0", 8080)
    logging.info("WebSocket Server started on ws://localhost:8080")
    asyncio.create_task(serial_reader_loop())
    await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Web bridge stopped by user.")
