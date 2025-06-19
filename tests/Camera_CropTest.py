import time
from picamera2 import Picamera2
import queue

frame_queue = queue.Queue(maxsize=2)

def debug_camera_settings():
    picam2 = Picamera2()

    print("=== Camera Debug Info ===")
    print(f"Sensor resolution: {picam2.sensor_resolution}")
    print(f"Available sensor modes:")
    for i, mode in enumerate(picam2.sensor_modes):
        print(f"  Mode {i}: {mode}")

    # 전체 센서를 사용하는 모드 찾기 (crop_limits가 (0,0)으로 시작하는 모드)
    full_sensor_modes = [mode for mode in picam2.sensor_modes
                        if mode['crop_limits'][0] == 0 and mode['crop_limits'][1] == 0]

    print(f"\nFull sensor modes: {len(full_sensor_modes)}")
    for mode in full_sensor_modes:
        print(f"  Size: {mode['size']}, FPS: {mode['fps']}")

    # 설정 후 실제 사용되는 값들 확인
    config = picam2.create_still_configuration(main={"size": picam2.sensor_resolution})
    picam2.configure(config)

    print(f"\nConfigured main size: {config['main']['size']}")
    print(f"Configured sensor size: {config.get('sensor', {}).get('output_size', 'Not set')}")

    picam2.start()
    time.sleep(1)

    # 현재 컨트롤 상태 확인
    try:
        metadata = picam2.capture_metadata()
        print(f"Current ScalerCrop: {metadata.get('ScalerCrop', 'Not available')}")
    except Exception as e:
        print(f"Could not get metadata: {e}")

    picam2.stop()

def picam2_init_full_view():
    picam2 = Picamera2()

    # 전체 센서 영역을 사용하는 모드 선택 (Mode 3: 3280x2464)
    sensor_modes = picam2.sensor_modes

    # 전체 센서를 사용하는 가장 큰 모드 찾기
    full_sensor_modes = [mode for mode in sensor_modes
                        if mode['crop_limits'][0] == 0 and mode['crop_limits'][1] == 0]

    if full_sensor_modes:
        # 가장 큰 해상도의 전체 센서 모드 선택
        selected_mode = max(full_sensor_modes, key=lambda x: x['size'][0] * x['size'][1])
        print(f"Selected sensor mode: {selected_mode['size']} at {selected_mode['fps']:.2f} fps")

        # 센서 모드를 명시적으로 지정
        config = picam2.create_still_configuration(
            main={"size": selected_mode['size'], "format": "RGB888"},
            sensor={"output_size": selected_mode['size'], "bit_depth": selected_mode['bit_depth']}
        )
    else:
        # 백업: 기본 센서 해상도 사용
        print("Using fallback sensor resolution")
        config = picam2.create_still_configuration(
            main={"size": picam2.sensor_resolution, "format": "RGB888"}
        )

    picam2.configure(config)

    # ScalerCrop을 전체 센서 영역으로 명시적 설정
    full_crop = (0, 0, picam2.sensor_resolution[0], picam2.sensor_resolution[1])
    picam2.set_controls({
        "ScalerCrop": full_crop
    })

    print(f"Set ScalerCrop to: {full_crop}")

    picam2.start()
    time.sleep(2)  # 안정화 시간

    return picam2

def picam2_init_full_view_alternative():
    """libcamera-hello와 유사한 설정을 시도하는 대안 방법"""
    picam2 = Picamera2()

    # Mode 1 (1640x1232) 사용 - 전체 센서 범위에서 다운샘플링
    config = picam2.create_preview_configuration(
        main={"size": (1640, 1232), "format": "RGB888"}
    )
    picam2.configure(config)

    # 전체 센서 크롭 설정
    picam2.set_controls({
        "ScalerCrop": (0, 0, 3280, 2464)
    })

    picam2.start()
    time.sleep(2)

    return picam2

def picam2_stream():
    """카메라 스트림 함수"""
    try:
        camera = picam2_init_full_view()

        print("Camera stream started with full view")

        while True:
            frame = camera.capture_array()
            if not frame_queue.full():
                frame_queue.put(frame)

    except Exception as e:
        print(f"Camera stream error: {e}")
    finally:
        try:
            camera.stop()
        except:
            pass

# 테스트 함수
def test_capture():
    """단일 이미지 캡처 테스트"""
    camera = picam2_init_full_view()

    try:
        # 메타데이터 확인
        metadata = camera.capture_metadata()
        print(f"Capture metadata - ScalerCrop: {metadata.get('ScalerCrop', 'Not available')}")

        # 이미지 캡처
        frame = camera.capture_array()
        print(f"Captured frame shape: {frame.shape}")

        return frame

    finally:
        camera.stop()

if __name__ == "__main__":
    # 디버그 정보 출력
    debug_camera_settings()

    print("\n" + "="*50)
    print("Testing single capture...")

    # 단일 캡처 테스트
    frame = test_capture()
