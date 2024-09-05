# 통합코드

import RPi.GPIO as GPIO
import time
import threading
import pygame
import os
import evdev
import cv2
import numpy as np
from gtts import gTTS

# GPIO 핀 설정
BUTTON_PIN = 17   # 손잡이 버튼 핀
IN1 = 22          # 모터 A
IN2 = 23          # 모터 A
IN3 = 24          # 모터 B
IN4 = 25          # 모터 B
ENA = 12          # 모터 A 속도 조절 핀
ENB = 13          # 모터 B 속도 조절 핀
TRIG = 5          # 초음파 센서 트리거 핀
ECHO = 6          # 초음파 센서 에코 핀
PIR_PIN = 3       # PIR 센서 핀 번호
STOP_PIN = 4      # 비상 버튼 핀
FW = 18           # 조향 서보모터 (Forward)
BK = 19           # 조향 서보모터 (Backward)

# 거리 임계값 (단위: 센티미터)
DISTANCE_THRESHOLD = 50  # 임계값을 50cm로 설정

# GPIO 설정
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# 핀 설정
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(STOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(FW, GPIO.OUT)
GPIO.setup(BK, GPIO.OUT)

# PWM 설정
PWM_FREQUENCY = 100  # PWM 주파수 (Hz)
PWM_DUTY_CYCLE = 30  # PWM 듀티 사이클 (30%)
pwm_a = GPIO.PWM(ENA, PWM_FREQUENCY)
pwm_b = GPIO.PWM(ENB, PWM_FREQUENCY)
pwm_fw = GPIO.PWM(FW, 50)
pwm_bk = GPIO.PWM(BK, 50)

pwm_a.start(PWM_DUTY_CYCLE)
pwm_b.start(PWM_DUTY_CYCLE)
pwm_fw.start(7.5)  # FW 서보모터 중앙 위치
pwm_bk.start(7.5)  # BK 서보모터 중앙 위치

# 음성 출력 함수
def speak(text):
    tts = gTTS(text=text, lang='ko')
    audio_file = 'output.mp3'
    tts.save(audio_file)

    pygame.mixer.init()
    pygame.mixer.music.load(audio_file)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

    os.remove(audio_file)

# 모터 제어 함수
def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(PWM_DUTY_CYCLE)
    pwm_b.ChangeDutyCycle(PWM_DUTY_CYCLE)
    print("모터 전진")

def motor_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    print("모터 후진")

def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    print("모터 정지")

def move_right():
    pwm_fw.ChangeDutyCycle(10)  # FW를 120도(90도 + 30도)
    pwm_bk.ChangeDutyCycle(5)   # BK를 60도(90도 - 30도)
    print("조향: 오른쪽으로 회전")

def move_left():
    pwm_fw.ChangeDutyCycle(5)   # FW를 60도(90도 - 30도)
    pwm_bk.ChangeDutyCycle(10)  # BK를 120도(90도 + 30도)
    print("조향: 왼쪽으로 회전")

def move_center():
    pwm_fw.ChangeDutyCycle(7.5)  # FW와 BK를 90도(중앙)
    pwm_bk.ChangeDutyCycle(7.5)
    print("조향: 중앙으로 맞춤")

def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    pulse_duration = end_time - start_time
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

def read_qr_code():
    global qr_code_valid
    code = ""
    while True:
        for event in qr_device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                key_event = evdev.categorize(event)
                if key_event.keystate == evdev.KeyEvent.key_down:
                    if key_event.keycode == 'KEY_ENTER':
                        print(f"스캔된 QR 코드: {code}")
                        if code == "123456789":
                            qr_code_valid = True
                            print("QR 코드 유효")
                        else:
                            qr_code_valid = False
                            print("QR 코드 유효하지 않음")
                        code = ""
                    else:
                        code += key_event.keycode.lstrip('KEY_')
            time.sleep(0.1)

def check_pir_sensor():
    global motion_detected
    current_state = GPIO.input(PIR_PIN)
    if current_state and not motion_detected:
        print("PIR 센서: 움직임 감지됨!")
        speak('안녕하세요. 이지봇입니다. 왼쪽은 비상버튼으로 누르는 즉시 멈춥니다. 오른쪽에 승차권을 찍어주시고 손잡이를 눌러주세요.')
        motion_detected = True
    elif not current_state and motion_detected:
        print("PIR 센서: 움직임 없음")
        motion_detected = False

# QR 코드 인식 장치 찾기
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
qr_device = None

for device in devices:
    if 'USBKey' in device.name:
        qr_device = device
        break

if qr_device is None:
    print("QR 인식기를 찾을 수 없습니다.")
    exit(1)

print(f"QR 인식기 발견: {qr_device.name}")

motion_detected = False
qr_code_valid = False
motor_running = False

def button_handler():
    global motor_running
    while True:
        button_pressed = GPIO.input(BUTTON_PIN) == GPIO.LOW
        if qr_code_valid and button_pressed:
            if not motor_running:
                motor_forward()
                motor_running = True
        else:
            if motor_running:
                stop_motors()
                motor_running = False
        time.sleep(0.1)

def emergency_stop():
    while True:
        button_state = GPIO.input(STOP_PIN)
        if button_state == GPIO.LOW:
            print("비상 버튼 눌림")
            stop_motors()
            break
        time.sleep(0.1)

# 비디오 캡처 초기화
cap = cv2.VideoCapture(0)

def process_video():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 40, 40])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        edges = cv2.Canny(mask, 50, 150)

        height, width = edges.shape
        moments = cv2.moments(edges)

        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m00'] / moments['m00'])
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        cv2.imshow('Frame', frame)
        cv2.imshow('Edges', edges)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

try:
    # QR 코드 유효성 검사를 백그라운드에서 수행
    qr_thread = threading.Thread(target=read_qr_code)
    qr_thread.daemon = True
    qr_thread.start()

    # 버튼 핸들러를 별도의 스레드에서 실행
    button_thread = threading.Thread(target=button_handler)
    button_thread.daemon = True
    button_thread.start()

    # 비상 버튼을 별도의 스레드에서 모니터링
    emergency_thread = threading.Thread(target=emergency_stop)
    emergency_thread.daemon = True
    emergency_thread.start()

    # 비디오 처리 스레드 시작
    video_thread = threading.Thread(target=process_video)
    video_thread.daemon = True
    video_thread.start()

    print("모터 제어 준비 완료.")

    while True:
        # PIR 센서 체크
        check_pir_sensor()

        # 거리 측정 및 경고
        distance = measure_distance()
        if distance > DISTANCE_THRESHOLD:
            print("경고: 거리 임계값 초과! 모터 정지 및 후진...")
            stop_motors()
            speak('경고: 거리 임계값 초과! 모터 정지 및 후진합니다.')
            time.sleep(1)
            motor_backward()
            time.sleep(1)
            stop_motors()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("프로그램이 중단되었습니다.")

finally:
    pwm_a.stop()
    pwm_b.stop()
    pwm_fw.stop()
    pwm_bk.stop()
    GPIO.cleanup()
    pygame.mixer.quit()
    print("GPIO 정리 완료")
