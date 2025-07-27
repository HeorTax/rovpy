#!/usr/bin/env python3

import time
from pymavlink import mavutil

# USB bağlantı
connection_string = '/dev/ttyUSB0'

print("USB bağlantısı kuruluyor...")
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Bağlantı kuruldu!")

# Motor isimleri
motors = ["Sol Ön", "Sağ Ön", "Sol Arka", "Sağ Arka", "Sol Dikey", "Sağ Dikey"]

def send_pwm(channel, pwm):
    """PWM gönder"""
    rc = [65535] * 18
    rc[channel-1] = pwm
    
    msg = master.mav.rc_channels_override_encode(
        master.target_system,
        master.target_component,
        *rc
    )
    master.mav.send(msg)

def stop_all():
    """Tüm motorları durdur"""
    for i in range(1, 7):
        send_pwm(i, 1500)

# Test başlat
print("\nMotor testi başlıyor...")
stop_all()
time.sleep(1)

# Her motoru sırayla test et
for motor_num in range(1, 7):
    motor_name = motors[motor_num-1]
    
    print(f"\nMotor {motor_num}: {motor_name}")
    
    # Motoru çalıştır
    send_pwm(motor_num, 1600)
    
    # 5 saniye bekle
    for i in range(5):
        print(f"  {i+1}/5")
        time.sleep(1)
    
    # Durdur
    send_pwm(motor_num, 1500)
    print(f"  {motor_name} durduruldu")
    
    if motor_num < 6:
        time.sleep(2)

# Bitir
stop_all()
print("\nTest tamamlandı!")
master.close()
