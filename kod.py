#!/usr/bin/env python3
import time
import rovpy

class SixMotorROVTest:
    def __init__(self):
        print("6 Motorlu ROV bağlantısı kuruluyor...")
        try:
            # ROV bağlantısını başlat
            self.rov = rovpy.rov()
            print("ROV bağlantısı başarılı!")
            
            # Motor kontrolcüsünü al
            self.thrusters = self.rov.thrusters
            print("Motor kontrolcüsü hazır (6 motor)")
            
        except Exception as e:
            print(f"Bağlantı hatası: {str(e)}")
            self.rov = None

    def test_motors(self):
        if not self.rov:
            print("ROV bağlantısı yok, test yapılamaz")
            return

        print("\n6 Motorlu ROV testi başlıyor...")
        print("Her motor sırayla 2 saniye çalıştırılacak")
        print("Çıkmak için CTRL+C tuşlarını kullanın\n")

        try:
            while True:
                # Motor 1 (Sol ön)
                print("Motor 1 (Sol ön) çalışıyor (ileri)")
                self.thrusters.set_speeds([0.5, 0, 0, 0, 0, 0])
                time.sleep(2)
                
                # Motor 2 (Sağ ön)
                print("Motor 2 (Sağ ön) çalışıyor (ileri)")
                self.thrusters.set_speeds([0, 0.5, 0, 0, 0, 0])
                time.sleep(2)
                
                # Motor 3 (Sol orta)
                print("Motor 3 (Sol orta) çalışıyor (ileri)")
                self.thrusters.set_speeds([0, 0, 0.5, 0, 0, 0])
                time.sleep(2)
                
                # Motor 4 (Sağ orta)
                print("Motor 4 (Sağ orta) çalışıyor (ileri)")
                self.thrusters.set_speeds([0, 0, 0, 0.5, 0, 0])
                time.sleep(2)
                
                # Motor 5 (Sol arka)
                print("Motor 5 (Sol arka) çalışıyor (geri)")
                self.thrusters.set_speeds([0, 0, 0, 0, -0.5, 0])
                time.sleep(2)
                
                # Motor 6 (Sağ arka)
                print("Motor 6 (Sağ arka) çalışıyor (geri)")
                self.thrusters.set_speeds([0, 0, 0, 0, 0, -0.5])
                time.sleep(2)
                
                # Tüm motorlar birlikte ileri
                print("Tüm motorlar ileri yönde çalışıyor")
                self.thrusters.set_speeds([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
                time.sleep(3)
                
                # Tüm motorlar birlikte geri
                print("Tüm motorlar geri yönde çalışıyor")
                self.thrusters.set_speeds([-0.3, -0.3, -0.3, -0.3, -0.3, -0.3])
                time.sleep(3)
                
                # Dikey hareket testi (eğer motorlar dikey hareket için kullanılıyorsa)
                print("Dikey hareket testi")
                self.thrusters.set_speeds([0.2, 0.2, -0.2, -0.2, 0, 0])  # Örnek konfigürasyon
                time.sleep(3)
                
                # Durdur
                print("Tüm motorlar durduruluyor")
                self.thrusters.set_speeds([0, 0, 0, 0, 0, 0])
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nTest sonlandırılıyor...")
        finally:
            self.thrusters.set_speeds([0, 0, 0, 0, 0, 0])
            self.rov.close()
            print("ROV bağlantısı kapatıldı")

    def show_motor_config(self):
        print("\nMevcut Motor Konfigürasyonu:")
        print("""
        [Motor 1: Sol ön]    [Motor 2: Sağ ön]
        [Motor 3: Sol orta]  [Motor 4: Sağ orta]
        [Motor 5: Sol arka]  [Motor 6: Sağ arka]
        """)
        print("Not: Motor yönleri ve pozisyonları ROV tasarımınıza göre değişebilir")

if __name__ == "__main__":
    tester = SixMotorROVTest()
    tester.show_motor_config()
    tester.test_motors()
