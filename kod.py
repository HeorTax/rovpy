from pymavlink import mavutil
import time

# Pixhawk bağlantısı (örnek olarak USB üzerinden)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ArduSub başlatmasını bekle
master.wait_heartbeat()
print("Bağlantı kuruldu. Sistem hazır.")

# Uçuş modunu değiştirme fonksiyonu
def set_mode(mode_str='STABILIZE'):  # Derinlik sabitleme yok, STABILIZE mod yeterli
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
        ack_msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if ack_msg.custom_mode == mode_id:
            print(f"{mode_str} moduna geçildi.")
            break

# Aracı ileri hareket ettir
def move_forward(duration=3, speed=0.5):
    print(f"{duration} saniye boyunca ileri gidiliyor.")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity kontrolü
        0, speed, 0,         # X ekseninde hız (ileri)
        0, 0, 0,             # z=0 (derinlik yok)
        0, 0, 0,
        0, 0
    )
    time.sleep(duration)
    stop()

# Sağa dönme
def yaw_right(yaw_rate=0.5, duration=2):
    print(f"{duration} saniye boyunca sağa dönülüyor.")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000000,  # yaw kontrolü
        0, 0, 0,
        0, 0, 0,
        0, 0, yaw_rate,
        0, 0
    )
    time.sleep(duration)
    stop()

# Sola dönme
def yaw_left(yaw_rate=0.5, duration=2):
    print(f"{duration} saniye boyunca sola dönülüyor.")
    yaw_right(-yaw_rate, duration)

# Durdurma
def stop():
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print("Araç durdu.")

# Ana görev akışı
def kablo_takip():
    # ARM et
    print("ARM ediliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    
    set_mode('STABILIZE')  # Derinlik kontrolü yoksa STABILIZE önerilir
    time.sleep(1)
    move_forward(3)
    yaw_right(0.5, 1)
    move_forward(2)
    yaw_left(0.5, 1)
    move_forward(2)

# Program çalıştır
if __name__ == '__main__':
    kablo_takip()
