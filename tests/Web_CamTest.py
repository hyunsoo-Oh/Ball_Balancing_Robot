import cv2

def run_webcam():
    # 웹캠 열기 (0: 첫 번째 장치, 보통 /dev/video0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # 해상도 설정 (320×320)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 자동 노출 끄기
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25로 설정 시 수동 모드
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)         # 노출 수동 조정 (값은 카메라에 따라 다름)

    # 자동 화이트밸런스 끄기
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4000)  # 값은 조정 필요
    
    print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 프레임 화면에 표시
        cv2.imshow("WebCam", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_webcam()
