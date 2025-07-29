#!/usr/bin/env python3
"""
6 Motorlu Su Altı Araç Motor Test Sistemi
PyMAVLink kullanarak Pixhawk ile iletişim
Jetson Nano uyumlu
"""

from pymavlink import mavutil
import time

class UnderwaterVehicleController:
    def _init_(self, connection_string='/dev/ttyACM0', baudrate=57600):
        """
        Su altı araç kontrol sınıfı
        
        Args:
            connection_string: Pixhawk bağlantı portu
            baudrate: Baud rate (genellikle 57600 veya 115200)
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.armed = False
        self.mode = None
        
        # Motor kanalları (ROV/Submarine için tipik konfigürasyon)
        self.motor_channels = {
            'front_right': 1,    # Sağ ön motor
            'front_left': 2,     # Sol ön motor
            'rear_right': 3,     # Sağ arka motor
            'rear_left': 4,      # Sol arka motor
            'vertical_front': 5, # Ön dikey motor
            'vertical_rear': 6   # Arka dikey motor
        }
        
        # PWM değerleri (1100-1900 arası, 1500 durgun)
        self.pwm_neutral = 1500
        self.pwm_min = 1100
        self.pwm_max = 1900
        
    def connect(self):
        """Pixhawk'a bağlan"""
        try:
            print(f"Pixhawk'a bağlanılıyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baudrate
            )
            
            # Heartbeat bekle
            print("Heartbeat bekleniyor...")
            self.master.wait_heartbeat()
            print(f"Heartbeat alındı! System ID: {self.master.target_system}")
            
            # Başlangıç parametrelerini al
            self.get_vehicle_status()
            return True
            
        except Exception as e:
            print(f"Bağlantı hatası: {e}")
            return False
    
    def get_vehicle_status(self):
        """Araç durumunu kontrol et"""
        try:
            # Mod bilgisini al
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                self.mode = mavutil.mode_string_v10(msg)
                print(f"Mevcut mod: {self.mode}")
                
                # Armed durumunu kontrol et
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.armed = True
                    print("Araç ARMED durumda")
                else:
                    self.armed = False
                    print("Araç DISARMED durumda")
                    
        except Exception as e:
            print(f"Durum alma hatası: {e}")
    
    def set_mode(self, mode):
        """Uçuş modunu değiştir"""
        try:
            # ArduSub için tipik modlar: MANUAL, STABILIZE, DEPTH_HOLD, POSITION_HOLD
            mode_id = self.master.mode_mapping().get(mode.upper())
            if mode_id is None:
                print(f"Geçersiz mod: {mode}")
                return False
                
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            print(f"Mod değiştiriliyor: {mode}")
            time.sleep(1)
            self.get_vehicle_status()
            return True
            
        except Exception as e:
            print(f"Mod değiştirme hatası: {e}")
            return False
    
    def check_arm_status(self):
        """Arm durumunu kontrol et ve detaylı bilgi ver"""
        try:
            # Heartbeat al
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("✅ Araç ARMED durumda")
                    self.armed = True
                    return True
                else:
                    print("❌ Araç DISARMED durumda")
                    self.armed = False
                    return False
            else:
                print("⚠  Heartbeat alınamadı")
                return False
                
        except Exception as e:
            print(f"Arm durumu kontrol hatası: {e}")
            return False
    
    def get_prearm_status(self):
        """Pre-arm kontrol durumlarını göster"""
        try:
            print("\n=== PRE-ARM KONTROL DURUMU ===")
            
            # SYS_STATUS mesajını al
            msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if msg:
                sensors_health = msg.onboard_control_sensors_health
                sensors_present = msg.onboard_control_sensors_present
                sensors_enabled = msg.onboard_control_sensors_enabled
                
                print(f"Sensör sağlığı: {sensors_health}")
                print(f"Mevcut sensörler: {sensors_present}")
                print(f"Aktif sensörler: {sensors_enabled}")
                
                # Batarya voltajı
                print(f"Batarya voltajı: {msg.voltage_battery/1000.0:.2f}V")
                
            # STATUSTEXT mesajlarını kontrol et (pre-arm hata mesajları için)
            print("\nPre-arm mesajları kontrol ediliyor...")
            start_time = time.time()
            while time.time() - start_time < 3:
                msg = self.master.recv_match(type='STATUSTEXT', blocking=False)
                if msg:
                    text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    if 'PreArm' in text or 'prearm' in text:
                        print(f"⚠  Pre-arm mesajı: {text}")
                time.sleep(0.1)
                        
        except Exception as e:
            print(f"Pre-arm durumu alma hatası: {e}")
    
    def arm_vehicle(self):
        """Aracı arm et"""
        try:
            print("\n=== ARM ETME İŞLEMİ ===")
            
            # Önce pre-arm durumunu kontrol et
            self.get_prearm_status()
            
            # Arm komutu gönder
            print("Arm komutu gönderiliyor...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            
            # Arm işleminin tamamlanmasını bekle
            print("Arm işlemi bekleniyor...")
            for i in range(10):  # 10 saniye bekle
                time.sleep(1)
                if self.check_arm_status():
                    print(f"✅ Araç {i+1} saniyede arm oldu!")
                    return True
                print(f"Bekleniyor... {i+1}/10")
            
            print("❌ Araç arm olmadı. Pre-arm kontrolleri başarısız olabilir.")
            
            # Arm olmama nedenlerini kontrol et
            print("\nArm olmama nedenleri kontrol ediliyor...")
            self.get_prearm_status()
            
            return False
            
        except Exception as e:
            print(f"Arm etme hatası: {e}")
            return False
    
    def disarm_vehicle(self):
        """Aracı disarm et"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            
            print("Araç disarm ediliyor...")
            time.sleep(2)
            self.get_vehicle_status()
            return not self.armed
            
        except Exception as e:
            print(f"Disarm etme hatası: {e}")
            return False
    
    def set_motor_pwm(self, channel, pwm_value):
        """Belirli bir motora PWM değeri gönder"""
        try:
            # PWM değerini sınırla
            pwm_value = max(self.pwm_min, min(self.pwm_max, pwm_value))
            
            # RC override komutu gönder
            channels = [65535] * 18  # 18 kanal, kullanılmayan kanallar için 65535
            channels[channel - 1] = pwm_value  # Kanal indeksi 0'dan başlar
            
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *channels
            )
            
            return True
            
        except Exception as e:
            print(f"PWM gönderme hatası: {e}")
            return False
    
    def stop_all_motors(self):
        """Tüm motorları durdur"""
        print("Tüm motorlar durduruluyor...")
        for channel in self.motor_channels.values():
            self.set_motor_pwm(channel, self.pwm_neutral)
        time.sleep(0.1)
    
    def test_individual_motors(self, test_duration=3, test_pwm=1600):
        """Her motoru tek tek test et"""
        print(f"\n=== Bireysel Motor Testleri ===")
        print(f"Test süresi: {test_duration} saniye")
        print(f"Test PWM: {test_pwm}")
        
        for motor_name, channel in self.motor_channels.items():
            print(f"\n{motor_name.upper()} motoru test ediliyor (Kanal {channel})...")
            
            # Motoru çalıştır
            self.set_motor_pwm(channel, test_pwm)
            time.sleep(test_duration)
            
            # Motoru durdur
            self.set_motor_pwm(channel, self.pwm_neutral)
            time.sleep(1)
            
            input("Devam etmek için Enter'a basın...")
    
    def test_roll_movement(self, test_duration=3, roll_pwm=1600):
        """Roll hareketi testi (sağa-sola yatma)"""
        print(f"\n=== ROLL Hareketi Testi ===")
        
        # Sağa roll
        print("Sağa roll...")
        self.set_motor_pwm(self.motor_channels['front_left'], roll_pwm)
        self.set_motor_pwm(self.motor_channels['rear_left'], roll_pwm)
        self.set_motor_pwm(self.motor_channels['front_right'], self.pwm_neutral - (roll_pwm - self.pwm_neutral))
        self.set_motor_pwm(self.motor_channels['rear_right'], self.pwm_neutral - (roll_pwm - self.pwm_neutral))
        time.sleep(test_duration)
        
        self.stop_all_motors()
        time.sleep(1)
        
        # Sola roll
        print("Sola roll...")
        self.set_motor_pwm(self.motor_channels['front_right'], roll_pwm)
        self.set_motor_pwm(self.motor_channels['rear_right'], roll_pwm)
        self.set_motor_pwm(self.motor_channels['front_left'], self.pwm_neutral - (roll_pwm - self.pwm_neutral))
        self.set_motor_pwm(self.motor_channels['rear_left'], self.pwm_neutral - (roll_pwm - self.pwm_neutral))
        time.sleep(test_duration)
        
        self.stop_all_motors()
    
    def test_pitch_movement(self, test_duration=3, pitch_pwm=1600):
        """Pitch hareketi testi (öne-arkaya eğilme)"""
        print(f"\n=== PITCH Hareketi Testi ===")
        
        # Öne pitch (burun aşağı)
        print("Burun aşağı (öne pitch)...")
        self.set_motor_pwm(self.motor_channels['vertical_front'], self.pwm_neutral - (pitch_pwm - self.pwm_neutral))
        self.set_motor_pwm(self.motor_channels['vertical_rear'], pitch_pwm)
        time.sleep(test_duration)
        
        self.stop_all_motors()
        time.sleep(1)
        
        # Arkaya pitch (burun yukarı)
        print("Burun yukarı (arkaya pitch)...")
        self.set_motor_pwm(self.motor_channels['vertical_front'], pitch_pwm)
        self.set_motor_pwm(self.motor_channels['vertical_rear'], self.pwm_neutral - (pitch_pwm - self.pwm_neutral))
        time.sleep(test_duration)
        
        self.stop_all_motors()
    
    def test_yaw_movement(self, test_duration=3, yaw_pwm=1600):
        """Yaw hareketi testi (sağa-sola dönme)"""
        print(f"\n=== YAW Hareketi Testi ===")
        
        # Sağa dönme
        print("Sağa dönme...")
        self.set_motor_pwm(self.motor_channels['front_left'], yaw_pwm)
        self.set_motor_pwm(self.motor_channels['rear_right'], yaw_pwm)
        self.set_motor_pwm(self.motor_channels['front_right'], self.pwm_neutral - (yaw_pwm - self.pwm_neutral))
        self.set_motor_pwm(self.motor_channels['rear_left'], self.pwm_neutral - (yaw_pwm - self.pwm_neutral))
        time.sleep(test_duration)
        
        self.stop_all_motors()
        time.sleep(1)
        
        # Sola dönme
        print("Sola dönme...")
        self.set_motor_pwm(self.motor_channels['front_right'], yaw_pwm)
        self.set_motor_pwm(self.motor_channels['rear_left'], yaw_pwm)
        self.set_motor_pwm(self.motor_channels['front_left'], self.pwm_neutral - (yaw_pwm - self.pwm_neutral))
        self.set_motor_pwm(self.motor_channels['rear_right'], self.pwm_neutral - (yaw_pwm - self.pwm_neutral))
        time.sleep(test_duration)
        
        self.stop_all_motors()
    
    def test_forward_backward(self, test_duration=3, thrust_pwm=1600):
        """İleri-geri hareket testi"""
        print(f"\n=== İLERİ-GERİ Hareket Testi ===")
        
        # İleri hareket
        print("İleri hareket...")
        for channel in [self.motor_channels['front_left'], self.motor_channels['front_right'],
                       self.motor_channels['rear_left'], self.motor_channels['rear_right']]:
            self.set_motor_pwm(channel, thrust_pwm)
        time.sleep(test_duration)
        
        self.stop_all_motors()
        time.sleep(1)
        
        # Geri hareket
        print("Geri hareket...")
        reverse_pwm = self.pwm_neutral - (thrust_pwm - self.pwm_neutral)
        for channel in [self.motor_channels['front_left'], self.motor_channels['front_right'],
                       self.motor_channels['rear_left'], self.motor_channels['rear_right']]:
            self.set_motor_pwm(channel, reverse_pwm)
        time.sleep(test_duration)
        
        self.stop_all_motors()
    
    def test_vertical_movement(self, test_duration=3, vertical_pwm=1600):
        """Dikey hareket testi (yukarı-aşağı)"""
        print(f"\n=== DİKEY Hareket Testi ===")
        
        # Yukarı çıkma
        print("Yukarı çıkma...")
        self.set_motor_pwm(self.motor_channels['vertical_front'], vertical_pwm)
        self.set_motor_pwm(self.motor_channels['vertical_rear'], vertical_pwm)
        time.sleep(test_duration)
        
        self.stop_all_motors()
        time.sleep(1)
        
        # Aşağı inme
        print("Aşağı inme...")
        reverse_pwm = self.pwm_neutral - (vertical_pwm - self.pwm_neutral)
        self.set_motor_pwm(self.motor_channels['vertical_front'], reverse_pwm)
        self.set_motor_pwm(self.motor_channels['vertical_rear'], reverse_pwm)
        time.sleep(test_duration)
        
        self.stop_all_motors()

def main():
    """Ana fonksiyon"""
    print("=== 6 Motorlu Su Altı Araç Motor Test Sistemi ===")
    print("Jetson Nano + Pixhawk + PyMAVLink")
    
    # Bağlantı string'ini kullanıcıdan al
    connection = input("Bağlantı portu (/dev/ttyACM0): ").strip()
    if not connection:
        connection = '/dev/ttyACM0'
    
    # Controller oluştur
    controller = UnderwaterVehicleController(connection)
    
    # Bağlantı kur
    if not controller.connect():
        print("Bağlantı kurulamadı. Çıkılıyor...")
        return
    
    try:
        # Manuel moda geç
        controller.set_mode('MANUAL')
        
        # Güvenlik için kullanıcı onayı
        print("\n!!! DİKKAT !!!")
        print("Motor testleri başlamak üzere.")
        print("Aracın suya değmediğinden emin olun!")
        confirm = input("Devam etmek istiyor musunuz? (y/N): ").strip().lower()
        
        if confirm != 'y':
            print("Test iptal edildi.")
            return
        
        # Arm et
        if not controller.arm_vehicle():
            print("Araç arm edilemedi. Test iptal edildi.")
            return
        
        while True:
            print("\n=== TEST MENÜSÜ ===")
            print("1. Bireysel motor testleri")
            print("2. Roll hareketi testi")
            print("3. Pitch hareketi testi") 
            print("4. Yaw hareketi testi")
            print("5. İleri-geri hareket testi")
            print("6. Dikey hareket testi")
            print("7. Tüm motorları durdur")
            print("8. Araç durumunu göster")
            print("9. Arm durumu kontrol et")
            print("0. Çıkış")
            
            choice = input("Seçiminiz: ").strip()
            
            if choice == '1':
                controller.test_individual_motors()
            elif choice == '2':
                controller.test_roll_movement()
            elif choice == '3':
                controller.test_pitch_movement()
            elif choice == '4':
                controller.test_yaw_movement()
            elif choice == '5':
                controller.test_forward_backward()
            elif choice == '6':
                controller.test_vertical_movement()
            elif choice == '7':
                controller.stop_all_motors()
                print("Tüm motorlar durduruldu.")
            elif choice == '8':
                print("\n=== ARAÇ DURUMU ===")
                controller.get_vehicle_status()
                controller.check_arm_status()
                controller.get_prearm_status()
            elif choice == '9':
                print("\n=== ARM DURUMU KONTROL ===")
                controller.check_arm_status()
                if not controller.armed:
                    print("Tekrar arm etmeyi deneyin? (y/N): ", end="")
                    retry = input().strip().lower()
                    if retry == 'y':
                        controller.arm_vehicle()
            elif choice == '0':
                break
            else:
                print("Geçersiz seçim!")
    
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu.")
    
    except Exception as e:
        print(f"Hata oluştu: {e}")
    
    finally:
        # Temizlik
        print("Temizlik yapılıyor...")
        controller.stop_all_motors()
        time.sleep(1)
        controller.disarm_vehicle()
        print("Test tamamlandı.")

if __name__ == "__main__":
    main()
