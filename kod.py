from pymavlink import mavutil
import time

# Pixhawk bağlantısı
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Bağlantı kuruldu. Sistem hazır.")

# ARM etme fonksiyonu
def arm():
    print("ARM ediliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Araç ARM edildi.")
            break

# Mod değiştirme
def set_mode(mode_str='MANUAL'):
    modes = master.mode_mapping()
    if mode_str not in modes:
        print(f"{mode_str} modu bulunamadı. Mevcut modlar: {list(modes.keys())}")
        return
    mode_id = modes[mode_str]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"{mode_str} moduna geçildi.")
            break

# PWM ile ileri hareket (RC override)
def move_forward(duration=2):
    print("İleri gidiliyor...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1600, 1600, 1500, 1500, 1500, 1500, 0, 0  # İleri yön (kanal 1-2)
    )
    time.sleep(duration)
    stop()

# Sağa dönme (örnek: kanal 4 - yaw)
def yaw_right(duration=1):
    print("Sağa dönülüyor...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1500, 1600, 1500, 1500, 0, 0
    )
    time.sleep(duration)
    stop()

# Sola dönme
def yaw_left(duration=1):
    print("Sola dönülüyor...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1500, 1400, 1500, 1500, 0, 0
    )
    time.sleep(duration)
    stop()

# Motorları durdur
def stop():
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 0, 0
    )
    print("Durdu.")

# Ana görev
def kablo_takip():
    set_mode('MANUAL')   # ArduSub için güvenli bir başlangıç
    arm()
    time.sleep(1)
    move_forward(3)
    yaw_right(1)
    move_forward(2)
    yaw_left(1)
    move_forward(2)
    stop()

# Çalıştır
if __name__ == '__main__':
    kablo_takip()
