#!/usr/bin/env python3
"""
Jetson Nano + Pixhawk (ArduSub) ile 6 motorlu su altı aracı motor test kodu
Motorlarınızın bağlantı sırasına göre düzenlemeyi unutmayın!
"""

import time
from pymavlink import mavutil

class ArduSub6MotorTest:
    # Motor kanal eşlemesi (ArduSub'da ayarladığınıza göre değiştirin)
    MOTOR_MAPPING = {
        1: 0,  # Motor 1 -> Kanal 1 (Throttle)
        2: 1,  # Motor 2 -> Kanal 2 (Roll)
        3: 2,  # Motor 3 -> Kanal 3 (Pitch)
        4: 3,  # Motor 4 -> Kanal 4 (Yaw)
        5: 4,  # Motor 5 -> Kanal 5 (Forward)
        6: 5   # Motor 6 -> Kanal 6 (Lateral)
    }

    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
        print(f"ArduSub'a bağlanılıyor: {connection_string} @ {baudrate} baud")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.wait_heartbeat()
        print("Bağlantı kuruldu!")

        # PWM sınırları (kendi motorlarınıza göre ayarlayın)
        self.PWM_MIN = 1100
        self.PWM_NEUTRAL = 1500
        self.PWM_MAX = 1900

        # Manuel moda geç (motor kontrolü için zorunlu)
        self.set_mode('MANUAL')

    def set_mode(self, mode):
        """ArduSub modunu değiştir"""
        mode_id = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.master.mav.set_mode_send(
            self.master.target_system,
            mode_id,
            self._get_mode_number(mode))
        print(f"{mode} moduna geçildi")

    def _get_mode_number(self, mode_name):
        """Mod ismini numaraya çevir"""
        modes = {
            'MANUAL': 0,
            'STABILIZE': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'ACRO': 4,
            'POSHOLD': 5
        }
        return modes.get(mode_name.upper(), 0)

    def set_motor_pwm(self, motor_id, pwm_value):
        """Tek bir motora PWM değeri gönder"""
        if motor_id not in self.MOTOR_MAPPING:
            print(f"Hatalı motor ID: {motor_id}")
            return

        pwm_value = max(self.PWM_MIN, min(pwm_value, self.PWM_MAX))
        channel = self.MOTOR_MAPPING[motor_id]

        # 16 kanallı override mesajı (6 motor için ilk 6 kanal)
        channels = [65535] * 16  # 65535 = ignore
        channels[channel] = pwm_value

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *channels[:16])

        print(f"Motor {motor_id} (Kanal {channel+1}) -> PWM: {pwm_value}")

    def test_sequence(self):
        """6 motor için test senaryosu"""
        try:
            print("\n=== Tüm motorlar nötr konumda ===")
            self._all_motors(self.PWM_NEUTRAL)
            time.sleep(2)

            print("\n=== Tekli motor testi ===")
            for motor_id in range(1, 7):
                print(f"\nMotor {motor_id} testi:")
                self._ramp_motor(motor_id)
                time.sleep(1)

            print("\n=== Tüm motorlar senkron testi ===")
            self._ramp_all_motors()

            print("\n=== Test tamamlandı ===")
            self._all_motors(self.PWM_NEUTRAL)

        except KeyboardInterrupt:
            print("\nTest durduruldu!")
            self._all_motors(self.PWM_NEUTRAL)

    def _ramp_motor(self, motor_id, duration=3):
        """Motoru kademeli olarak çalıştır/durdur"""
        steps = [1100, 1300, 1500, 1700, 1900, 1700, 1500, 1300, 1100]
        for pwm in steps:
            self.set_motor_pwm(motor_id, pwm)
            time.sleep(duration/len(steps))

    def _ramp_all_motors(self, duration=5):
        """Tüm motorları senkronize test et"""
        steps = [1500, 1600, 1700, 1800, 1900, 1800, 1700, 1600, 1500]
        for pwm in steps:
            for motor_id in range(1, 7):
                self.set_motor_pwm(motor_id, pwm)
            time.sleep(duration/len(steps))

    def _all_motors(self, pwm_value):
        """Tüm motorlara aynı PWM değerini gönder"""
        for motor_id in range(1, 7):
            self.set_motor_pwm(motor_id, pwm_value)

if __name__ == "__main__":
    # Bağlantı ayarlarınızı düzenleyin
    tester = ArduSub6MotorTest(connection_string='/dev/ttyACM0', baudrate=115200)
    
    print("""
    6 Motorlu Su Altı Aracı Test Programı
    ------------------------------------
    1) Tek motor testi
    2) Tüm motorlar senkron testi
    3) Tam test senaryosu
    4) Tüm motorları durdur
    """)
    
    try:
        while True:
            choice = input("Seçiminiz (1-4, Çıkmak için q): ")
            
            if choice == '1':
                motor_id = int(input("Motor ID (1-6): "))
                tester._ramp_motor(motor_id)
            elif choice == '2':
                tester._ramp_all_motors()
            elif choice == '3':
                tester.test_sequence()
            elif choice == '4':
                tester._all_motors(tester.PWM_NEUTRAL)
            elif choice.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        print("\nProgram sonlandırıldı")
    finally:
        tester._all_motors(tester.PWM_NEUTRAL)
