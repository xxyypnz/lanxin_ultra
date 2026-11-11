import json
import socket
import struct
import threading

import cv2
import numpy as np


SERVER_IP = "192.168.1.28"
SERVER_MESSAGE_PORT = 10001
SERVER_VIDEO_PORT = 10002
MSG_TYPE_VIDEO = 1
MSG_TYPE_JSON = 2
MAX_IMAGE_SIZE = 4 * 1024 * 1024


def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def decode_image(image_data):
    encoded = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("failed to decode PNG image")

    if image.ndim == 2:
        if image.dtype == np.uint16:
            display_image = cv2.normalize(
                image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
        else:
            display_image = image
        return "Depth / Amp", display_image, False

    if image.ndim == 3 and image.shape[2] == 3:
        return "RGB", image, True

    raise ValueError(f"unsupported decoded PNG shape: {image.shape}")


def draw_timestamps(image, sensor_ts, recv_ts, is_color):
    color = (255, 255, 255) if is_color else 255
    cv2.putText(image, f"Sensor: {sensor_ts}", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(image, f"Receive: {recv_ts}", (12, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return image


def receive_json(sock):
    length_data = recv_exact(sock, 4)
    if length_data is None:
        return False

    payload_length = struct.unpack(">I", length_data)[0]
    payload = recv_exact(sock, payload_length)
    if payload is None:
        return False

    print(f"[SERVER JSON] {payload.decode('utf-8')}")
    return True


def receive_video(sock):
    # Current C frame: type is already read, followed by two timestamps,
    # image length, and the encoded PNG bytes.
    header = recv_exact(sock, 20)
    if header is None:
        return False

    sensor_ts, recv_ts, image_length = struct.unpack(">QQI", header)
    if image_length == 0 or image_length > MAX_IMAGE_SIZE:
        raise ValueError(f"invalid PNG image length: {image_length}")

    image_data = recv_exact(sock, image_length)
    if image_data is None:
        return False

    name, image, is_color = decode_image(image_data)
    image = draw_timestamps(image, sensor_ts, recv_ts, is_color)
    cv2.imshow(name, image)
    return True


def receiver_thread(sock):
    try:
        while True:
            type_data = recv_exact(sock, 4)
            if type_data is None:
                return
            msg_type = struct.unpack(">I", type_data)[0]

            if msg_type == MSG_TYPE_JSON:
                if not receive_json(sock):
                    return
            elif msg_type == MSG_TYPE_VIDEO:
                if not receive_video(sock):
                    return
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return
            else:
                raise ValueError(f"unknown server message type: {msg_type}")
    except (OSError, UnicodeDecodeError, ValueError, struct.error) as error:
        print(f"Receiver stopped: {error}")


def send_json_command(sock, text):
    command = json.loads(text)
    if not isinstance(command, dict):
        raise ValueError("command must be a JSON object")

    payload = json.dumps(command, separators=(",", ":")).encode("utf-8")
    sock.sendall(payload)
    print(f"[SENT] {payload.decode('utf-8')}")


def main():
    port_text = input(f"Enter Port ({SERVER_MESSAGE_PORT}/{SERVER_VIDEO_PORT}): ")
    if port_text not in (str(SERVER_MESSAGE_PORT), str(SERVER_VIDEO_PORT)):
        print("Invalid port")
        return
    server_port = int(port_text)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((SERVER_IP, server_port))
        print(f"[SUCCESS] Connected to {SERVER_IP}:{server_port}")

        receiver = threading.Thread(target=receiver_thread, args=(client,), daemon=True)
        receiver.start()

        while True:
            text = input("Enter JSON command (q to quit): ").strip()
            if text.lower() == "q":
                break
            if not text:
                continue
            try:
                send_json_command(client, text)
            except (json.JSONDecodeError, ValueError) as error:
                print(f"Invalid command: {error}")
    except OSError as error:
        print(f"[ERROR] {error}")
    finally:
        client.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
