import random

# ──────────────── Random Search 루프 ────────────────
def random_pid_search(evaluate_pid, trials=30, early_stop_score=10.0):
    best_score = float('inf')
    best_params = (0, 0, 0)

    try:
        for i in range(30):  # 예: 30조합 평가
            Kp = random.uniform(0.1, 1.0)
            Ki = random.uniform(0.0, 0.1)
            Kd = random.uniform(0.05, 0.5)

            print(f"[{i+1}/30] Trying Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
            score = evaluate_pid(Kp, Ki, Kd, duration=3.0)
            print(f"→ 평균오차 Score = {score:.2f}")

            result_msg = f"RESULT,{Kp:.2f},{Ki:.2f},{Kd:.2f},{score:.2f}"

            if score < best_score - 0.01: #조금 더 좋아졌을 때만 갱신
                best_score = score
                best_params = (Kp, Ki, Kd)
                print(f"✅ Best updated: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} → Score={score:.2f}")
            # score가 일정 기준 이하일 때 종료
            if score < 10.0:
                print("🎯 임계값 도달 → 조기 종료!")
                break

    except KeyboardInterrupt:
        print("튜닝 중단됨")

    finally:
        print(f"🔚 최종 Best: Kp={best_params[0]:.2f}, Ki={best_params[1]:.2f}, Kd={best_params[2]:.2f}, Score={best_score:.2f}")
