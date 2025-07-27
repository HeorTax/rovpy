#!/usr/bin/env python3
"""
Jetson Nano ve Pixhawk ile 6 motorlu su altı aracı motor test kodu
"""

import time
from pymavlink import mavutil

class UnderwaterVehicleMotorTest:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
        """
        Pixhawk bağlantısını başlat
        :param connection_string: Pixhawk bağlantı noktası (örn. /dev/ttyACM0, /dev/ttyUSB0)
        :param baudrate: Seri bağlantı baud hızı
        """
        print(f"Pixhawk'a bağlanılıyor: {connection_string} @ {baudrate} baud")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        
        # Heartbeat göndererek bağlantıyı bekleyelim
        self.master.wait_heartbeat()
        print("Pixhawk ile bağlantı kuruldu!")
        
        # Motor sayısı
        self.motor_count = 6
        
        # Motor minimum ve maksimum PWM değerleri
        self.PWM_MIN = 1100  # Mikro saniye
        self.PWM_MAX = 1900  # Mikro saniye
        self.PWM_NEUTRAL = 1500  # Mikro saniye

    def set_motor_pwm(self, motor_id, pwm_value):
        """
        Belirli bir motora PWM değeri gönder
        :param motor_id: Motor ID'si (1-6)
        :param pwm_value: PWM değeri (1100-1900)
        """
        if motor_id < 1 or motor_id > self.motor_count:
            print(f"Hatalı motor ID: {motor_id}. 1-{self.motor_count} aralığında olmalı.")
            return
        
        # PWM değerini sınırla
        pwm_value = max(self.PWM_MIN, min(pwm_value, self.PWM_MAX))
        
        # MAV_CMD_DO_MOTOR_TEST komutunu gönder
        # Parametreler: instance (motor numarası, 0'dan başlar), throttle (PWM), duration (ms), motor count, test order
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,  # confirmation
            motor_id - 1,  # motor numarası (0'dan başlar)
            pwm_value,  # PWM değeri
            0,  # süre (0=sonsuz)
            1,  # motor sayısı
            0,  # test türü (0=motor testi)
            0   # test sırası
        )
        
        print(f"Motor {motor_id} PWM: {pwm_value}")

    def all_motors_stop(self):
        """Tüm motorları durdur (nötr PWM gönder)"""
        for motor_id in range(1, self.motor_count + 1):
            self.set_motor_pwm(motor_id, self.PWM_NEUTRAL)
        print("Tüm motorlar durduruldu (nötr PWM)")

    def test_individual_motors(self, test_duration=3):
        """
        Her motoru tek tek test et
        :param test_duration: Her motorun çalışma süresi (saniye)
        """
        print("\nBireysel motor testi başlıyor...")
        
        for motor_id in range(1, self.motor_count + 1):
            print(f"\nMotor {motor_id} test ediliyor...")
            
            # Motoru yavaşça çalıştır
            for pwm in range(self.PWM_NEUTRAL, self.PWM_NEUTRAL + 200, 50):
                self.set_motor_pwm(motor_id, pwm)
                time.sleep(0.5)
            
            # Tam güç
            self.set_motor_pwm(motor_id, self.PWM_MAX)
            time.sleep(test_duration)
            
            # Motoru durdur
            self.set_motor_pwm(motor_id, self.PWM_NEUTRAL)
            time.sleep(1)
        
        print("\nBireysel motor testi tamamlandı!")

    def test_all_motors_together(self, test_duration=5):
        """
        Tüm motorları birlikte test et
        :param test_duration: Test süresi (saniye)
        """
        print("\nTüm motorlar birlikte test ediliyor...")
        
        # Yavaşça hızlandır
        for pwm in range(self.PWM_NEUTRAL, self.PWM_NEUTRAL + 300, 50):
            for motor_id in range(1, self.motor_count + 1):
                self.set_motor_pwm(motor_id, pwm)
            time.sleep(1)
        
        # Tam güç
        for motor_id in range(1, self.motor_count + 1):
            self.set_motor_pwm(motor_id, self.PWM_MAX)
        time.sleep(test_duration)
        
        # Yavaşça durdur
        for pwm in range(self.PWM_MAX, self.PWM_NEUTRAL - 1, -50):
            for motor_id in range(1, self.motor_count + 1):
                self.set_motor_pwm(motor_id, pwm)
            time.sleep(0.5)
        
        print("\nTüm motorlar testi tamamlandı!")

    def run_comprehensive_test(self):
        """Kapsamlı motor testi yürüt"""
        print("\nKapsamlı motor testi başlıyor...")
        
        # 1. Adım: Tüm motorları durdur
        self.all_motors_stop()
        time.sleep(2)
        
        # 2. Adım: Motorları tek tek test et
        self.test_individual_motors()
        time.sleep(2)
        
        # 3. Adım: Tüm motorları birlikte test et
        self.test_all_motors_together()
        time.sleep(2)
        
        # 4. Adım: Testi sonlandır
        self.all_motors_stop()
        print("\nKapsamlı motor testi tamamlandı!")

if __name__ == "__main__":
    # Bağlantı ayarlarını buradan değiştirebilirsiniz
    # Örnek: '/dev/ttyUSB0' veya 'udp:127.0.0.1:14550' (SITL için)
    uv_test = UnderwaterVehicleMotorTest(connection_string='/dev/ttyACM0', baudrate=115200)
    
    try:
        # Kapsamlı testi çalıştır
        uv_test.run_comprehensive_test()
        
        # Veya tek tek test fonksiyonlarını çağırabilirsiniz:
        # uv_test.test_individual_motors()
        # uv_test.test_all_motors_together()
        
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu!")
        uv_test.all_motors_stop()
    except Exception as e:
        print(f"\nHata oluştu: {str(e)}")
        uv_test.all_motors_stop()
