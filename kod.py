#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Basit 6 Motorlu Su Altı Aracı Test Sistemi
Her motoru sırayla 5 saniye çalıştırır
"""

import time
from pymavlink import mavutil

class SimpleMotorTester:
    def _init_(self, connection_string='tcp:127.0.0.1:5760'):
        """
        Motor test sistemi
        """
        print("Araca bağlanılıyor...")
        self.master = mavutil.mavlink_connection(connection_string)
        
        # Heartbeat bekle
        self.master.wait_heartbeat()
        print(f"Bağlandı! System ID: {self.master.target_system}")
        
        # Motor isimleri
        self.motor_names = {
            1: "Sol Ön Motor",
            2: "Sağ Ön Motor", 
            3: "Sol Arka Motor",
            4: "Sağ Arka Motor",
            5: "Sol Dikey Motor",
            6: "Sağ Dikey Motor"
        }
        
        # PWM değerleri
        self.pwm_stop = 1500      # Dur
        self.pwm_test = 1600      # Test hızı

    def send_motor_pwm(self, motor_channel, pwm_value):
        """
        Motora PWM gönder (1-6 arası motor kanalı)
        """
        # 18 kanallı RC override mesajı
        rc_channels = [65535] * 18  # 65535 = değiştirme
        
        if 1 <= motor_channel <= 6:
            rc_channels[motor_channel - 1] = pwm_value
            
            msg = self.master.mav.rc_channels_override_encode(
                self.master.target_system,
                self.master.target_component,
                *rc_channels
            )
            self.master.mav.send(msg)

    def stop_all_motors(self):
        """
        Tüm motorları durdur
        """
        print("Tüm motorlar durduruluyor...")
        for motor in range(1, 7):
            self.send_motor_pwm(motor, self.pwm_stop)
        time.sleep(0.5)

    def test_motors(self):
        """
        Motorları sırayla test et
        """
        print("\n=== MOTOR TESTİ BAŞLIYOR ===")
        
        # Önce tüm motorları durdur
        self.stop_all_motors()
        
        # Her motoru sırayla test et
        for motor_num in range(1, 7):
            motor_name = self.motor_names[motor_num]
            
            print(f"\n{motor_num}. Motor Testi: {motor_name}")
            print("Çalıştırılıyor...")
            
            # Motoru çalıştır
            self.send_motor_pwm(motor_num, self.pwm_test)
            
            # 5 saniye bekle
            for i in range(5):
                print(f"  {i+1}/5 saniye")
                time.sleep(1)
            
            # Motoru durdur
            self.send_motor_pwm(motor_num, self.pwm_stop)
            print(f"{motor_name} durduruldu")
            
            # Sonraki motor için 2 saniye ara
            if motor_num < 6:
                print("Sonraki motor için bekleniyor...")
                time.sleep(2)
        
        print("\n=== TÜM MOTORLAR TEST EDİLDİ ===")

    def cleanup(self):
        """
        Temizlik
        """
        self.stop_all_motors()
        self.master.close()
        print("Bağlantı kapatıldı")

def main():
    """
    Ana fonksiyon
    """
    # Bağlantı ayarları
    connection = 'tcp:127.0.0.1:5760'  # Simulatör için
    # connection = 'udp:127.0.0.1:14550'  # UDP için
    # connection = '/dev/ttyUSB0'  # Serial için
    
    try:
        # Test sistemi oluştur
        tester = SimpleMotorTester(connection)
        
        # Test başlat
        input("Motor testini başlatmak için ENTER'a basın...")
        tester.test_motors()
        
        # Temizlik
        tester.cleanup()
        
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu!")
        tester.stop_all_motors()
        tester.cleanup()
        
    except Exception as e:
        print(f"Hata: {e}")

if _name_ == "_main_":
    main()
