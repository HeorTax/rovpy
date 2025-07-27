#!/usr/bin/env python3

import time
from pymavlink import mavutil

try:
    # ACM0 bağlantı
    print("Bağlanılıyor...")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
    master.wait_heartbeat()
    print("Bağlandı!")
    
    # Motor test fonksiyonu
    def motor_pwm(channel, pwm):
        rc = [65535] * 18
        rc[channel-1] = pwm
        msg = master.mav.rc_channels_override_encode(
            master.target_system, master.target_component, *rc)
        master.mav.send(msg)
    
    # Test başlat
    print("Test başlıyor...")
    
    # Motorları sırayla test et
    for motor in range(1, 7):
        print(f"Motor {motor} çalışıyor...")
        motor_pwm(motor, 1600)  # Çalıştır
        time.sleep(5)           # 5 saniye bekle
        motor_pwm(motor, 1500)  # Durdur
        print(f"Motor {motor} durduruldu")
        time.sleep(1)
    
    print("Test bitti!")
    
except Exception as e:
    print(f"Hata: {e}")
    
finally:
    try:
        # Tüm motorları durdur
        for i in range(1, 7):
            motor_pwm(i, 1500)
        master.close()
    except:
        pass
