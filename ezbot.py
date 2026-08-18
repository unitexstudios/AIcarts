import RPi.GPIO as GPIO
import time
from gtts import gTTS
import os
import pygame
import cv2
from pyzbar.pyzbar import decode
from bluetooth import *
import numpy as np
import torch
import nfc
from smartcard.System import readers
from smartcard.util import toHexString
import logging

# Initialize logging
logging.basicConfig(filename='robot_log.txt', level=logging.DEBUG, format='%(asctime)s:%(levelname)s:%(message)s')

# Initialize YOLO model for object detection
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Motor and GPIO settings
IN1, IN2, IN3, IN4 = 22, 23, 24, 25
ENA, ENB, FW, BK = 12, 13, 18, 19
HANDLE_PIN, EMERGENCY_PIN, PIR_SENSOR_PIN = 26, 27, 17
ULTRASONIC_TRIG, ULTRASONIC_ECHO = 5, 6
BT_ADDR, BT_PORT = '00:11:22:33:44:55', 1

# NFC setup for ACR122U
r = readers()
if not r:
    logging.error("No NFC readers found. Ensure ACR122U is connected.")
    raise Exception("No NFC readers found.")
nfc_reader = r[0].createConnection()
nfc_reader.connect()

GPIO.setmode(GPIO.BCM)
for pin in [IN1, IN2, IN3, IN4, ENA, ENB, FW, BK, HANDLE_PIN, EMERGENCY_PIN, PIR_SENSOR_PIN, ULTRASONIC_TRIG, ULTRASONIC_ECHO]:
    GPIO.setup(pin, GPIO.OUT if pin in [IN1, IN2, IN3, IN4, ENA, ENB, FW, BK, ULTRASONIC_TRIG] else GPIO.IN)

pwm_a, pwm_b, pwm_fw, pwm_bk = GPIO.PWM(ENA, 100), GPIO.PWM(ENB, 100), GPIO.PWM(FW, 50), GPIO.PWM(BK, 50)
for pwm in [pwm_a, pwm_b, pwm_fw, pwm_bk]: pwm.start(0)

languages = ['ko', 'en', 'zh']
distance_threshold, height_threshold = 20, 10
gate_map = {"TagID1": "Gate 1", "TagID2": "Gate 2", "TagID3": "Gate 3", "TagID4": "Gate 4"}

# Helper Functions
def set_language(lang_code):
    global language
    language = lang_code

def speak_message(messages):
    for lang in languages:
        tts = gTTS(text=messages[lang], lang=lang)
        tts.save(f'message_{lang}.mp3')
        pygame.mixer.init()
        pygame.mixer.music.load(f'message_{lang}.mp3')
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
        os.remove(f'message_{lang}.mp3')

def motor_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed); pwm_b.ChangeDutyCycle(speed)
    logging.info("Motor moving forward at speed %d", speed)

def motor_reverse(speed=50):
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed); pwm_b.ChangeDutyCycle(speed)
    logging.info("Motor reversing at speed %d", speed)

def stop_motors():
    for pin in [IN1, IN2, IN3, IN4]: GPIO.output(pin, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0); pwm_b.ChangeDutyCycle(0)
    logging.info("Motors stopped")

def turn_left(angle=90):
    pwm_fw.ChangeDutyCycle(5); pwm_bk.ChangeDutyCycle(10)
    time.sleep(angle / 90); move_center()
    logging.info("Turning left at angle %d", angle)

def turn_right(angle=90):
    pwm_fw.ChangeDutyCycle(10); pwm_bk.ChangeDutyCycle(5)
    time.sleep(angle / 90); move_center()
    logging.info("Turning right at angle %d", angle)

def move_center():
    pwm_fw.ChangeDutyCycle(7.5); pwm_bk.ChangeDutyCycle(7.5)
    logging.info("Centering")

def detect_qr():
    cap = cv2.VideoCapture(0)
    qr_data = None
    while True:
        if GPIO.input(PIR_SENSOR_PIN) == GPIO.HIGH:
            greet_user()
            ret, frame = cap.read()
            if not ret: continue
            for qr_code in decode(frame):
                qr_data = qr_code.data.decode('utf-8')
                logging.info("QR code detected: %s", qr_data)
                cap.release()
                return qr_data
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    cap.release()
    return None

def scan_nfc_tag():
    try:
        apdu = [0xFF, 0xCA, 0x00, 0x00, 0x00]
        response, sw1, sw2 = nfc_reader.transmit(apdu)
        if sw1 == 0x90 and sw2 == 0x00:
            tag_id = toHexString(response)
            logging.info("NFC Tag detected: %s", tag_id)
            return gate_map.get(tag_id, "Unknown Gate")
    except Exception as e:
        logging.error("NFC read error: %s", str(e))
    return None

def ultrasonic_height_check():
    GPIO.output(ULTRASONIC_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG, False)
    start, end = time.time(), time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 0: start = time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 1: end = time.time()
    distance = ((end - start) * 34300) / 2
    if distance > height_threshold:
        stop_motors()
        speak_message({
            'ko': "높은 위치가 감지되었습니다. 안전을 위해 정지합니다.",
            'en': "High elevation detected. Stopping for safety.",
            'zh': "检测到高海拔。为安全起见，请停止。"
        })
        logging.warning("Height threshold exceeded. Stopping motors.")
        return True
    return False

def check_emergency():
    if GPIO.input(EMERGENCY_PIN) == GPIO.HIGH:
        stop_motors()
        speak_message({
            'ko': "비상상태입니다. 문제를 해결해주세요.",
            'en': "Emergency state. Please resolve the issue.",
            'zh': "紧急状态。请解决问题。"
        })
        logging.critical("Emergency button pressed. System halted.")
        return True
    return False

def yolo_object_detection():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret: break
        results = model(frame)
        for *box, conf, cls in results.xyxy[0]:
            if conf > 0.5 and calculate_distance(box) < distance_threshold:
                stop_motors()
                speak_message({
                    'ko': "앞에 장애물이 있습니다.",
                    'en': "Obstacle detected. Stopping.",
                    'zh': "检测到障碍物。停止。"
                })
                logging.warning("Obstacle detected. Motors stopped.")
                return
        if cv2.waitKey(1) & 0xFF
            ord('q'): break
    cap.release()
    return False

def calculate_distance(box):
    xmin, xmax = box[0], box[2]
    distance = 500 / (xmax - xmin)
    logging.debug("Calculated distance: %f", distance)
    return distance

def communicate_with_elevator():
    sock = BluetoothSocket(RFCOMM)
    try:
        sock.connect((BT_ADDR, BT_PORT))
        sock.send("Robot waiting")
        while True:
            response = sock.recv(1024).decode('utf-8')
            logging.info("Elevator response: %s", response)
            if response == "Elevator ready":
                if detect_crowd():
                    speak_message({
                        'ko': "사람이 너무 많습니다. 협조 부탁드립니다.",
                        'en': "The elevator is too crowded. Please make room.",
                        'zh': "电梯太拥挤了。请让出空间。"
                    })
                    time.sleep(20)
                    continue
                else:
                    speak_message({
                        'ko': "엘리베이터가 준비되었습니다. 탑승하세요.",
                        'en': "Elevator is ready. Please board.",
                        'zh': "电梯已准备好。请登机。"
                    })
                    return True
            elif response == "Elevator close" and not detect_exit_confirmation():
                speak_message({
                    'ko': "하차가 확인될 때까지 엘리베이터가 이동할 수 없습니다.",
                    'en': "Elevator cannot proceed until exit is confirmed.",
                    'zh': "在确认下车之前，电梯无法继续。"
                })
                sock.send("Hold Elevator")
    except BluetoothError as e:
        speak_message({
            'ko': "엘리베이터와의 통신에 실패했습니다.",
            'en': "Failed to communicate with the elevator.",
            'zh': "与电梯通信失败。"
        })
        logging.error("Bluetooth error with elevator: %s", str(e))
        return False
    finally:
        sock.close()

def detect_crowd():
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        results = model(frame)
        person_count = sum([1 for *box, conf, cls in results.xyxy[0] if cls == 0 and conf > 0.5])  # Class 0 is 'person'
        if person_count > 5:
            cap.release()
            return True
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cap.release()
    return False

def detect_exit_confirmation():
    start_time = time.time()
    while time.time() - start_time < 30:
        apdu = [0xFF, 0xCA, 0x00, 0x00, 0x00]
        response, sw1, sw2 = nfc_reader.transmit(apdu)
        if sw1 == 0x90 and sw2 == 0x00:
            tag_id = toHexString(response)
            logging.info("Exit confirmation NFC tag detected: %s", tag_id)
            if tag_id == "ExitTag":
                return True
        time.sleep(0.5)
    logging.warning("Exit confirmation timeout: elevator cannot proceed")
    return False

# === Navigation Paths ===

def navigate_to_elevator():
    # Shared path for all gates from 2nd-floor start to elevator
    speak_message({
        'ko': "엘리베이터로 이동합니다.",
        'en': "Moving to the elevator.",
        'zh': "前往电梯。"
    })
    move_stage("forward", distance=3)
    turn_left(45)
    yolo_object_detection()
    move_stage("forward", distance=4)
    stop_motors()
    if communicate_with_elevator():
        speak_message({
            'ko': "엘리베이터 준비 완료. 탑승합니다.",
            'en': "Elevator ready. Boarding now.",
            'zh': "电梯已准备好。正在登机。"
        })
    else:
        logging.error("Failed to board the elevator.")
        return False
    return True

def navigate_to_gate(gate):
    # Elevator navigation completed; now on 1st floor, proceeding to specific gate
    if check_emergency() or ultrasonic_height_check():
        return

    if gate == "Gate 1":
        speak_message({
            'ko': "1번 탑승구로 이동합니다.",
            'en': "Moving to Boarding Gate 1.",
            'zh': "前往登机口 1。"
        })
        move_stage("forward", distance=3)
        turn_right(45)
        yolo_object_detection()
        move_stage("forward", distance=5)
        stop_motors()

    elif gate == "Gate 2":
        speak_message({
            'ko': "2번 탑승구로 이동합니다.",
            'en': "Moving to Boarding Gate 2.",
            'zh': "前往登机口 2。"
        })
        move_stage("forward", distance=4)
        turn_left(45)
        yolo_object_detection()
        move_stage("forward", distance=6)
        stop_motors()

    elif gate == "Gate 3":
        speak_message({
            'ko': "3번 탑승구로 이동합니다.",
            'en': "Moving to Boarding Gate 3.",
            'zh': "前往登机口 3。"
        })
        move_stage("forward", distance=3)
        turn_left(90)
        yolo_object_detection()
        move_stage("forward", distance=7)
        stop_motors()

    elif gate == "Gate 4":
        speak_message({
            'ko': "4번 탑승구로 이동합니다.",
            'en': "Moving to Boarding Gate 4.",
            'zh': "前往登机口 4。"
        })
        move_stage("forward", distance=5)
        turn_right(90)
        yolo_object_detection()
        move_stage("forward", distance=4)
        stop_motors()

    speak_message({
        'ko': f"{gate}에 도착했습니다.",
        'en': f"Arrived at {gate}.",
        'zh': f"到达{gate}。"
    })

    # Return path starts here
    return_to_start(gate)

def move_stage(direction, distance=None, angle=None):
    if direction == "forward":
        motor_forward()
        time.sleep(distance)
        stop_motors()
    elif direction == "reverse":
        motor_reverse()
        time.sleep(distance)
        stop_motors()
    elif direction == "left":
        turn_left(angle)
    elif direction == "right":
        turn_right(angle)

def return_to_start(gate):
    # Return path from each gate to the 1st-floor elevator
    speak_message({
        'ko': "출발 지점으로 복귀합니다.",
        'en': "Returning to the start point.",
        'zh': "返回到起点。"
    })

    if gate == "Gate 1":
        move_stage("reverse", distance=5)
        turn_left(45)
        move_stage("reverse", distance=3)
    
    elif gate == "Gate 2":
        move_stage("reverse", distance=6)
        turn_right(45)
        move_stage("reverse", distance=4)

    elif gate == "Gate 3":
        move_stage("reverse", distance=7)
        turn_right(90)
        move_stage("reverse", distance=3)

    elif gate == "Gate 4":
        move_stage("reverse", distance=4)
        turn_left(90)
        move_stage("reverse", distance=5)

    # Communicate with the elevator on the return trip
    if communicate_with_elevator():
        speak_message({
            'ko': "엘리베이터로 복귀합니다.",
            'en': "Returning via the elevator.",
            'zh': "通过电梯返回。"
        })

     # Final return path after exiting the elevator on the 2nd floor
    move_stage("reverse", distance=4)
    turn_right(45)
    move_stage("reverse", distance=3)
    stop_motors()
    
    # Announce completion of return to start point
    speak_message({
        'ko': "출발 지점으로 복귀 완료.",
        'en': "Returned to the start point.",
        'zh': "返回到起点。"
    })

# Initialization and Main Control Loop
try:
    while True:
        # Initial greeting and ticket verification
        if GPIO.input(PIR_SENSOR_PIN) == GPIO.HIGH:
            speak_message({
                'ko': "안녕하세요! QR 코드를 스캔해주세요.",
                'en': "Hello! Please scan your QR code.",
                'zh': "你好！请扫描二维码。"
            })
            qr_result = detect_qr()
            nfc_gate = scan_nfc_tag()
            gate_to_navigate = nfc_gate if nfc_gate in gate_map.values() else qr_result

            # Navigate to the elevator if a valid boarding gate is detected
            if gate_to_navigate in ["Gate 1", "Gate 2", "Gate 3", "Gate 4"]:
                if navigate_to_elevator():  # Move from the 2nd-floor start to the elevator
                    navigate_to_gate(gate_to_navigate)  # Move to the specific boarding gate on 1st floor
            else:
                speak_message({
                    'ko': "올바르지 않은 목적지입니다. 다시 시도하세요.",
                    'en': "Invalid destination. Please try again.",
                    'zh': "无效目的地。请再试一次。"
                })
        time.sleep(2)  # Brief delay before re-checking for new arrivals

except KeyboardInterrupt:
    print("Program interrupted.")
    logging.info("Program interrupted by user.")

finally:
    # Clean up resources and GPIO settings on exit
    pwm_a.stop()
    pwm_b.stop()
    pwm_fw.stop()
    pwm_bk.stop()
    GPIO.cleanup()
    pygame.mixer.quit()
    logging.info("All resources cleaned up. Exiting.")
