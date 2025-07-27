#!/usr/bin/env python3
import time
from pymavlink import mavutil

class MAVLinkROVControl:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
        print("MAVLink bağlantısı kuruluyor...")
        
        # MAVLink bağlantısını oluştur
        try:
            self.connection = mavutil.mavlink_connection(connection_string, baud=baudrate)
            print(f"{connection_string} üzerinden bağlantı kuruldu (Baudrate: {baudrate})")
            
            # Heartbeat mesajı gönder
            self.connection.wait_heartbeat()
            print("Heartbeat alındı, bağlantı başarılı!")
            
            # ROV'nin sistem ID'sini al
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            print(f"Sistem ID: {self.target_system}, Komponent ID: {self.target_component}")
            
        except Exception as e:
            print(f"MAVLink bağlantı hatası: {str(e)}")
            self.connection = None

    def set_motor_speeds(self, speeds):
        """
        6 motorun hızlarını ayarlar (normalize edilmiş değerler: -1.0 ile 1.0 arası)
        speeds = [motor1, motor2, motor3, motor4, motor5, motor6]
        """
        if not self.connection:
            print("Bağlantı yok, motorlar kontrol edilemiyor")
            return False

        try:
            # Motor çıkışlarını ayarla (MAV_CMD_DO_MOTOR_TEST)
            # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST
            
            # Önce tüm motorları durdur
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,  # confirmation
                0,  # motor instance (0 for all)
                mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
                0,  # throttle percentage
                0,  # timeout
                0,  # motor count
                0,  # test order
                0   # empty
            )
            
            # Her motoru ayrı ayrı ayarla
            for i, speed in enumerate(speeds):
                motor_id = i + 1  # MAVLink motor numaralandırması genelde 1'den başlar
                
                # Hız değerini -1000 ile 1000 arasına ölçeklendir
                pwm_value = int(1000 + speed * 1000)  # 1000-2000 arası PWM
                pwm_value = max(1000, min(2000, pwm_value))  # Sınırları koru
                
                self.connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                    0,  # confirmation
                    motor_id,  # motor instance
                    mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM,
                    pwm_value,  # throttle PWM
                    1000,  # timeout (ms)
                    1,  # motor count
                    0,  # test order
                    0   # empty
                )
            
            return True
            
        except Exception as e:
            print(f"Motor kontrol hatası: {str(e)}")
            return False

    def test_motors(self):
        if not self.connection:
            print("Bağlantı yok, test yapılamaz")
            return

        print("\n6 Motorlu ROV MAVLink testi başlıyor...")
        print("Her motor sırayla 2 saniye çalıştırılacak")
        print("Çıkmak için CTRL+C tuşlarını kullanın\n")

        try:
            while True:
                # Motor 1 (Sol ön)
                print("Motor 1 (Sol ön) çalışıyor (ileri)")
                self.set_motor_speeds([0.5, 0, 0, 0, 0, 0])
                time.sleep(2)
                
                # Motor 2 (Sağ ön)
                print("Motor 2 (Sağ ön) çalışıyor (ileri)")
                self.set_motor_speeds([0, 0.5, 0, 0, 0, 0])
                time.sleep(2)
                
                # Motor 3 (Sol orta)
                print("Motor 3 (Sol orta) çalışıyor (ileri)")
                self.set_motor_speeds([0, 0, 0.5, 0, 0, 0])
                time.sleep(2)
                
                # Motor 4 (Sağ orta)
                print("Motor 4 (Sağ orta) çalışıyor (ileri)")
                self.set_motor_speeds([0, 0, 0, 0.5, 0, 0])
                time.sleep(2)
                
                # Motor 5 (Sol arka)
                print("Motor 5 (Sol arka) çalışıyor (geri)")
                self.set_motor_speeds([0, 0, 0, 0, -0.5, 0])
                time.sleep(2)
                
                # Motor 6 (Sağ arka)
                print("Motor 6 (Sağ arka) çalışıyor (geri)")
                self.set_motor_speeds([0, 0, 0, 0, 0, -0.5])
                time.sleep(2)
                
                # Tüm motorlar birlikte ileri
                print("Tüm motorlar ileri yönde çalışıyor")
                self.set_motor_speeds([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
                time.sleep(3)
                
                # Tüm motorlar birlikte geri
                print("Tüm motorlar geri yönde çalışıyor")
                self.set_motor_speeds([-0.3, -0.3, -0.3, -0.3, -0.3, -0.3])
                time.sleep(3)
                
                # Dikey hareket testi
                print("Dikey hareket testi")
                self.set_motor_speeds([0.2, 0.2, -0.2, -0.2, 0, 0])
                time.sleep(3)
                
                # Durdur
                print("Tüm motorlar durduruluyor")
                self.set_motor_speeds([0, 0, 0, 0, 0, 0])
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nTest sonlandırılıyor...")
        finally:
            self.set_motor_speeds([0, 0, 0, 0, 0, 0])
            print("Tüm motorlar durduruldu")

if __name__ == "__main__":
    # Bağlantı ayarlarını cihazınıza göre değiştirin
    # Örnekler: '/dev/ttyACM0', 'udp:127.0.0.1:14550', 'com3'
    rov = MAVLinkROVControl(connection_string='/dev/ttyACM0', baudrate=115200)
    rov.test_motors()
