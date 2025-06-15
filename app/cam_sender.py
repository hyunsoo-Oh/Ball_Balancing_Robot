from picamera2 import Picamera2
import cv2
import socket
import struct
import pickle
import time

def start_picam_stream(ip="10.10.10.93", port=8485):
    # Picamera2 초기화
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)

    print("🎥 Picamera2 초기화 완료")

    # 소켓 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((ip, port))
        print("🔗 PC와 연결 성공")
    except Exception as e:
        print("❌ 연결 실패:", e)
        return

    trans_once = False
    while True:
        try:
            frame = picam2.capture_array()
            data = pickle.dumps(frame, protocol=pickle.HIGHEST_PROTOCOL)
            size = struct.pack(">L", len(data))
            client_socket.sendall(size + data)

            if not trans_once:
                print("📤 프레임 전송 완료:", len(data), "bytes")
                trans_once = True
        except (BrokenPipeError, ConnectionResetError):
            print("❌ 연결이 끊어졌습니다.")
            break
        except Exception as e:
            print("❌ 전송 중 예외 발생:", e)
            break

    client_socket.close()
    print("📴 연결 종료")

# 진입점
if __name__ == "__main__":
    start_picam_stream()
