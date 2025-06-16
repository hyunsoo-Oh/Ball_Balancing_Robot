import time
from utils.buffer import tracking_data_queue, kinematics_data_queue

def kinematics_task():
    prev_time = None
    prev_pos = None
    prev_vel = (0, 0)

    while True:
        data = tracking_data_queue.get()
        x, y = data['x'], data['y']
        curr_time = data['timestamp']

        if prev_time is not None:
            dt = curr_time - prev_time
            vx = (x - prev_pos[0]) / dt
            vy = (y - prev_pos[1]) / dt
            ax = (vx - prev_vel[0]) / dt
            ay = (vy - prev_vel[1]) / dt

            # PID용 데이터 전달
            pid_data = {
                'x': x,
                'y': y,
                'vx': vx,
                'vy': vy,
                'ax': ax,
                'ay': ay,
                'timestamp': curr_time
            }
            if not kinematics_data_queue.full():
                kinematics_data_queue.put(pid_data)

            print(f"위치: ({x}, {y})")
            print(f"속도: vx={vx:.2f}, vy={vy:.2f}")
            print(f"가속도: ax={ax:.2f}, ay={ay:.2f}\n")

            prev_vel = (vx, vy)

        prev_time = curr_time
        prev_pos = (x, y)
