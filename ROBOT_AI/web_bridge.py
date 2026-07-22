import asyncio
import websockets
import serial
import serial.tools.list_ports
import struct
import json
import logging
import sys

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)

# Protocol Constants
ROS2_HEADER1 = 0xFF
ROS2_HEADER2 = 0xFE
ROS2_TAIL    = 0xFD

MSG_ID_TELEMETRY   = 0x01
MSG_ID_CMD_VEL     = 0x02
MSG_ID_SET_MODE    = 0x03
MSG_ID_RESET_GOC   = 0x04
MSG_ID_ACK         = 0x05
MSG_ID_TRIGGER_BEEP = 0x06

# WS Connected Clients
connected_clients = set()
serial_connection = None

def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc

def make_beep_packet() -> bytes:
    header = bytes([ROS2_HEADER1, ROS2_HEADER2, MSG_ID_TRIGGER_BEEP, 0])
    crc = calculate_crc16(bytes([MSG_ID_TRIGGER_BEEP, 0]))
    crc_bytes = bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    tail = bytes([ROS2_TAIL])
    return header + crc_bytes + tail

def make_set_mode_packet(mode: int, e_stop: int = 0) -> bytes:
    payload = bytes([mode, e_stop])
    header = bytes([ROS2_HEADER1, ROS2_HEADER2, MSG_ID_SET_MODE, len(payload)]) + payload
    crc = calculate_crc16(bytes([MSG_ID_SET_MODE, len(payload)]) + payload)
    crc_bytes = bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    tail = bytes([ROS2_TAIL])
    return header + crc_bytes + tail

def auto_detect_serial_port():
    ports = serial.tools.list_ports.comports()
    # Sort ports, look for COM5 or USB Serial
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
                packet = make_beep_packet()
                if serial_connection and serial_connection.is_open:
                    serial_connection.write(packet)
                    logging.info("Sent BEEP packet to ESP32 Serial")
            elif message.startswith("SET_MODE:"):
                try:
                    mode = int(message.split(":")[1])
                    packet = make_set_mode_packet(mode)
                    if serial_connection and serial_connection.is_open:
                        serial_connection.write(packet)
                        logging.info(f"Sent SET_MODE packet ({mode}) to ESP32 Serial")
                except Exception as e:
                    logging.error(f"Error parsing SET_MODE parameter: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)
        logging.info(f"WebSocket client disconnected: {websocket.remote_address}")

def parse_telemetry_payload(payload_bytes: bytes):
    # Struct format: <IfffffffffffBBhhhhB
    # size: 4 + 11*4 + 1*2 + 2*4 + 1 = 59 bytes
    try:
        unpacked = struct.unpack("<IfffffffffffBBhhhhB", payload_bytes)
        return {
            "timestamp_ms": unpacked[0],
            "accel_x": round(unpacked[1], 4),
            "accel_y": round(unpacked[2], 4),
            "accel_z": round(unpacked[3], 4),
            "gyro_x": round(unpacked[4], 4),
            "gyro_y": round(unpacked[5], 4),
            "gyro_z": round(unpacked[6], 4),
            "roll": round(unpacked[7], 2),
            "pitch": round(unpacked[8], 2),
            "yaw": round(unpacked[9], 2),
            "front_distance": round(unpacked[10], 2),
            "rear_distance": round(unpacked[11], 2),
            "current_mode": unpacked[12],
            "auto_state": unpacked[13],
            "motor_fl_speed": unpacked[14],
            "motor_fr_speed": unpacked[15],
            "motor_rl_speed": unpacked[16],
            "motor_rr_speed": unpacked[17],
            "flags": unpacked[18]
        }
    except Exception as e:
        logging.error(f"Failed to unpack telemetry payload: {e}")
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

        # Re-detect in case it changed
        port = auto_detect_serial_port()

    # Parser state machine state
    # 0: Wait Header 1
    # 1: Wait Header 2
    # 2: Wait MsgID
    # 3: Wait Length
    # 4: Read Payload
    # 5: Read CRC
    # 6: Wait Tail
    state = 0
    msg_id = 0
    payload_len = 0
    payload_buf = bytearray()
    crc_buf = bytearray()

    while True:
        try:
            if not serial_connection.is_open:
                raise serial.SerialException("Serial port is closed")

            # Non-blocking read of available bytes
            if serial_connection.in_waiting > 0:
                data = serial_connection.read(serial_connection.in_waiting)
                for byte in data:
                    if state == 0:
                        if byte == ROS2_HEADER1:
                            state = 1
                    elif state == 1:
                        if byte == ROS2_HEADER2:
                            state = 2
                        else:
                            state = 0
                    elif state == 2:
                        msg_id = byte
                        state = 3
                    elif state == 3:
                        payload_len = byte
                        payload_buf = bytearray()
                        if payload_len > 0:
                            state = 4
                        else:
                            state = 5
                            crc_buf = bytearray()
                    elif state == 4:
                        payload_buf.append(byte)
                        if len(payload_buf) == payload_len:
                            state = 5
                            crc_buf = bytearray()
                    elif state == 5:
                        crc_buf.append(byte)
                        if len(crc_buf) == 2:
                            state = 6
                    elif state == 6:
                        if byte == ROS2_TAIL:
                            # Complete packet! Verify CRC
                            # CRC is calculated on: MsgID, Length, Payload
                            crc_calc_data = bytes([msg_id, payload_len]) + bytes(payload_buf)
                            crc_calculated = calculate_crc16(crc_calc_data)
                            crc_received = crc_buf[0] | (crc_buf[1] << 8)

                            if crc_calculated == crc_received:
                                if msg_id == MSG_ID_TELEMETRY:
                                    telemetry = parse_telemetry_payload(bytes(payload_buf))
                                    if telemetry:
                                        # Broadcast to all connected clients
                                        message_str = json.dumps({"type": "telemetry", "data": telemetry})
                                        if connected_clients:
                                            # Create a task to send to all connected clients
                                            websockets_tasks = [
                                                client.send(message_str) for client in connected_clients
                                            ]
                                            await asyncio.gather(*websockets_tasks, return_exceptions=True)
                            else:
                                logging.warning("CRC check failed on received serial packet!")
                        state = 0
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
            # Reconnect loop
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
    # Start WebSocket Server on port 8080
    ws_server = await websockets.serve(ws_handler, "0.0.0.0", 8080)
    logging.info("WebSocket Server started on ws://localhost:8080")

    # Start Serial Reader task
    asyncio.create_task(serial_reader_loop())

    # Keep running forever
    await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Web bridge stopped by user.")
