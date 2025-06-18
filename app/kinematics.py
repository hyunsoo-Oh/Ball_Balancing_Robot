import time
from utils.buffer import tracking_data_queue, kinematics_data_queue

# === tracking_data_queue에서 위치 데이터를 받고 속도, 가속도를 계산 ===
def kinematics_task():
    prev_time = None    # 이전 타임스탬프 저장 변수
    prev_pos = None     # 이전 위치 좌표 (x, y)
    prev_vel = (0, 0)   # 이전 속도 (vx, vy)

    while True:
        # tracking_data_queue에서 새로운 위치 데이터 수신
        data = tracking_data_queue.get()
        x, y = data['x'], data['y']     # 현재 위치
        curr_time = data['timestamp']   # 현재 시간 (초 단위)

        if prev_time is not None:
            # 시간 차이로 dt(시간 간격) 계산
            dt = curr_time - prev_time
            if dt < 0.01:
                dt = 0.01  # 최소 시간 간격 보정 (0.1초)

            # 속도 및 가속도 계산 함수 호출 (노이즈 제거 필터 포함)
            vx, vy, ax, ay = compute_kinematics(x, y, prev_pos, prev_vel, dt)
                
            # PID용 데이터 전달
            pid_data = {
                'x': x, 'y': y,         # 현재 위치
                'vx': vx, 'vy': vy,     # 현재 속도
                'ax': ax, 'ay': ay,     # 현재 가속도
                'timestamp': curr_time  # 현재 시간
            }
            # 데이터 큐가 가득 차지 않았을 경우에만 삽입
            if not kinematics_data_queue.full():
                kinematics_data_queue.put(pid_data)

            print(f"위치: ({x}, {y})")
            print(f"속도: vx={vx:.2f}, vy={vy:.2f}")
            print(f"가속도: ax={ax:.2f}, ay={ay:.2f}\n")

            # 현재 속도를 이전 속도로 저장 (다음 루프 계산에 사용)
            prev_vel = (vx, vy)
            
        # 현재 시간과 위치를 이전 값으로 저장 (다음 루프에서 사용)
        prev_time = curr_time
        prev_pos = (x, y)

# === 두 위치/속도 간 차이를 이용해 속도와 가속도 계산 ===
# threshold : 한 픽셀 이하의 변화는 무시 (위치, 속도 노이즈 제거용)
def compute_kinematics(x, y, prev_pos, prev_vel, dt, threshold=1):
    # 현재 위치와 이전 위치의 차이 (픽셀 기준)
    dx = x - prev_pos[0]
    dy = y - prev_pos[1]

    # 축별로 속도 계산 (노이즈 필터링 적용)
    vx = dx / dt if abs(dx) >= threshold else 0.0
    vy = dy / dt if abs(dy) >= threshold else 0.0

    # 축별로 가속도 계산 (속도 변화량 기준 필터링 적용)
    dvx = vx - prev_vel[0]
    dvy = vy - prev_vel[1]

    ax = dvx / dt if abs(dvx) >= threshold else 0.0
    ay = dvy / dt if abs(dvy) >= threshold else 0.0

    return vx, vy, ax, ay
