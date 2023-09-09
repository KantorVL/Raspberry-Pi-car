import time
from pynput import keyboard
from threading import Thread
import Adafruit_PCA9685
from unittest.mock import Mock
import RPi.GPIO as GPIO
import cv2

class Сontrol:
    def __init__(self):
        self.serv = [90] * 3
        GPIO.setwarnings(False)
        self.in1_pin = 17
        self.in2_pin = 27
        self.in3_pin = 22
        self.in4_pin = 23
        ena_pin = 18
        enb_pin = 13
        self.speed=50
        # Настройка пинов
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.in3_pin, GPIO.OUT)
        GPIO.setup(self.in4_pin, GPIO.OUT)
        GPIO.setup(ena_pin, GPIO.OUT)
        GPIO.setup(enb_pin, GPIO.OUT)
        # Инициализация PCA9685
        self.pwm_serv = Adafruit_PCA9685.PCA9685()
        # Настройка ШИМ
        self.pwm_a = GPIO.PWM(ena_pin, 60)
        self.pwm_b = GPIO.PWM(enb_pin, 60)

        self.stop()

        self.pwm_serv.set_pwm_freq(50)
        pulse = int(90* (500 / 180) + 110)
        self.pwm_serv.set_pwm(0, 0, pulse)
        self.pwm_serv.set_pwm(1, 0, pulse)
        ##self.pwm_serv.set_pwm(2, 0, pulse)

    def stop (self):
        GPIO.output(self.in1_pin, False)
        GPIO.output(self.in2_pin, False)
        GPIO.output(self.in3_pin, False)
        GPIO.output(self.in4_pin, False)
    def drive_forward(self) :
        self.pwm_a.start(self.speed)
        self.pwm_b.start(self.speed)
        GPIO.output(self.in1_pin, True)
        GPIO.output(self.in2_pin, False)
        GPIO.output(self.in3_pin, True)
        GPIO.output(self.in4_pin, False)
        time.sleep(0.1)
        self.stop()
    def drive_back(self,):
        self.pwm_a.start(self.speed)
        self.pwm_b.start(self.speed)
        GPIO.output(self.in1_pin, False)
        GPIO.output(self.in2_pin, True)
        GPIO.output(self.in3_pin, False)
        GPIO.output(self.in4_pin, True)
        time.sleep(0.1)
        self.stop()

    def adjust_servo_angle(self, servo_index, delta_angle):
        new_angle = self.serv[servo_index] + delta_angle
        new_angle = max(0, min(180, new_angle))
        pulse = int(new_angle * (500 / 180) + 110)
        self.pwm_serv.set_pwm(servo_index, 0, pulse)
        self.serv[servo_index] = new_angle
    def car_left(self):
        print("car_left")
      # Прибавляем 5° к текущему углу сервопривода с индексом 2
        self.adjust_servo_angle(2, -5)  # Вычитаем 5° из текущего угла сервопривода с индексом 2
        time.sleep(0.01)
        self.pwm_serv.set_pwm(2,0,0)

    def car_right(self):
        print("car_right")
        self.adjust_servo_angle(2, 5)
        time.sleep(0.01)
        self.pwm_serv.set_pwm(2,0,0)





    def camera_left(self):
       if self.serv[0]<180:
            print("camer_left")
            print("serv 0=",self.serv[0])
            self.serv[0]+=1
            pulse = int(self.serv[0]* (500 / 180) + 110)
            self.pwm_serv.set_pwm(0, 0, pulse)

    def camera_right(self):
        if self.serv[0]>0:
            print("camer_right")
            print("serv 0=",self.serv[0])
            self.serv[0]-=1
            pulse = int(self.serv[0]* (500 / 180) + 110)
            self.pwm_serv.set_pwm(0, 0, pulse)

    def camera_up(self):
        if self.serv[1]<=135:
            print("camer_up")
            self.serv[1]+=1
            pulse = int(self.serv[1]* (500 / 180) + 110)
            self.pwm_serv.set_pwm(1, 0, pulse)

    def camera_down(self):
        if self.serv[1]>=0:
            print("camer_down")
            self.serv[1]-=1
            print("serv 1=",self.serv[1])
            pulse = int(self.serv[1]* (500 / 180) + 110)
            self.pwm_serv.set_pwm(1, 0, pulse)

go=Сontrol()
cap = cv2.VideoCapture(0)
def on_press(key):
    try:
        if key == keyboard.Key.up:
            go.drive_forward()
        elif key == keyboard.Key.down:
            go.drive_back()
        elif key == keyboard.Key.left:
            go.car_left()
        elif key == keyboard.Key.right:
            go.car_right()
        elif key.char == "a":
            go.camera_left()
        elif key.char == "d":
            go.camera_right()
        elif key.char == "w":
            go.camera_up()
        elif key.char =="s":
            go.camera_down()

    except AttributeError:
        print('special key {0} pressed'.format(key))




def on_release(key):
    if key == keyboard.Key.esc:
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        return False

# Старт прослушивания
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    while True:
        # Читаем кадр
        ret, frame = cap.read()
        if ret:
            cv2.imshow('Camera Output', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print('Ошибка чтения кадра')

    listener.join()
    # Старт прослушивания