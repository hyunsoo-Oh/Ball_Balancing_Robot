import queue

# 공 추적용 프레임 공유
frame_queue = queue.Queue(maxsize=5)  # 카메라 → 트래킹

# 추적 결과 (좌표, 속도, 가속도)
tracking_data_queue = queue.Queue(maxsize=5)  # 트래킹 → 기구학

# 기구학 결과 (각 링크 위치, 목표 각도)
kinematics_data_queue = queue.Queue(maxsize=5)  # 기구학 → 제어기

# 제어 결과 (PWM, 각도, 에러 등)
control_output_queue = queue.Queue(maxsize=5)  # 제어 → 소켓 전송 등
