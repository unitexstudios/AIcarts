import RPi.GPIO as GPIO
import time
import threading
import pygame
import os
import evdev
import cv2
import numpy as np
from gtts import gTTS
import serial

# GPIO 핀 설정
BUTTON_PIN = 17
IN1 = 22
IN2 = 23
IN3 = 24
IN4 = 25
ENA = 12
ENB = 13
TRIG = 5
ECHO = 6
PIR_PIN = 3
STOP_PIN = 4
FW = 18
BK = 19
ADDITIONAL_PIR_PINS = [2, 9, 10]

# 거리 임계값
DISTANCE_THRESHOLD = 50

# HC-05 블루투스 설정
BLUETOOTH_PORT = '/dev/ttyS0'
BAUD_RATE = 9600
ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

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

for pin in ADDITIONAL_PIR_PINS:
    GPIO.setup(pin, GPIO.IN)

PWM_FREQUENCY = 100
PWM_DUTY_CYCLE = 30
pwm_a = GPIO.PWM(ENA, PWM_FREQUENCY)
pwm_b = GPIO.PWM(ENB, PWM_FREQUENCY)
pwm_fw = GPIO.PWM(FW, 50)
pwm_bk = GPIO.PWM(BK, 50)

pwm_a.start(PWM_DUTY_CYCLE)
pwm_b.start(PWM_DUTY_CYCLE)
pwm_fw.start(7.5)
pwm_bk.start(7.5)

qr_code_value = None  # QR 코드 값 저장용 변수

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
    pwm_fw.ChangeDutyCycle(10)
    pwm_bk.ChangeDutyCycle(5)
    print("오른쪽 회전")

def move_left():
    pwm_fw.ChangeDutyCycle(5)
    pwm_bk.ChangeDutyCycle(10)
    print("왼쪽 회전")

def move_center():
    pwm_fw.ChangeDutyCycle(7.5)
    pwm_bk.ChangeDutyCycle(7.5)
    print("중앙 맞춤")

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
    global qr_code_value
    code = ""
    while True:
        for event in qr_device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                key_event = evdev.categorize(event)
                if key_event.keystate == evdev.KeyEvent.key_down:
                    if key_event.keycode == 'KEY_ENTER':
                        print(f"스캔된 QR 코드: {code}")
                        qr_code_value = code
                        code = ""
                    else:
                        code += key_event.keycode.lstrip('KEY_')
            time.sleep(0.1)

def process_intersection():
    global qr_code_value
    if qr_code_value:
        first_num = int(qr_code_value[0])
        second_num = int(qr_code_value[2])

        # 첫 번째 갈림길에서 방향 결정
        if first_num in [1, 2]:
            move_left()
        elif first_num in [3, 4]:
            move_right()

        time.sleep(2)  # 회전 시간
        move_center()

        # 두 번째 갈림길에서 방향 결정
        if second_num in [1, 2]:
            move_left()
        elif second_num in [3, 4]:
            move_right()

        time.sleep(2)
        move_center()

def check_pir_sensor():
    global motion_detected
    current_state = GPIO.input(PIR_PIN)
    if current_state and not motion_detected:
        print("PIR 센서: 움직임 감지됨!")
        speak('안녕하세요. 이지봇입니다.')
        motion_detected = True
    elif not current_state and motion_detected:
        print("PIR 센서: 움직임 없음")
        motion_detected = False

def check_additional_pir_sensors():
    for pin in ADDITIONAL_PIR_PINS:
        if GPIO.input(pin):
            print(f"추가 PIR 센서 {pin}: 가까운 물체 감지됨!")
            stop_motors()
            while GPIO.input(pin):
                time.sleep(0.1)
            motor_forward()

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

        moments = cv2.moments(edges)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

            # 교차로 감지
            if cx < 100 or cx > 540:
                print("교차로 감지됨, QR 코드 기반 회전 시작")
                process_intersection()

        cv2.imshow('Frame', frame)
        cv2.imshow('Edges', edges)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# 메인 루프
try:
    qr_thread = threading.Thread(target=read_qr_code)
    qr_thread.daemon = True
    qr_thread.start()

    video_thread = threading.Thread(target=process_video)
    video_thread.daemon = True
    video_thread.start()

    print("모터 제어 준비 완료.")

    while True:
        check_pir_sensor()
        check_additional_pir_sensors()
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
    ser.close()
    print("GPIO 정리 완료")
