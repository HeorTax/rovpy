from pymavlink import mavutil
import time

# Pixhawk bağlantısı (USB üzerinden ACM0 portu)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Bağlantı kuruldu.")

# GUIDED moda geç
def set_guided_mode():
    master.set_mode_apm('GUIDED')
    print("GUIDED moda geçildi.")
    time.sleep(1)

# Aracı armla
def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Motorlar aktifleştirildi.")
    time.sleep(2)

# Araç ileri hareket etsin
def move_forward(duration=3, speed=0.3):
    print(f"{duration} saniye boyunca ileri hareket ediliyor.")
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms yerine 0 yazıldı
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity kontrolü aktif
        0, 0, 0,             # konum
        speed, 0, 0,         # hız (x yönü = ileri)
        0, 0, 0,             # ivme
        0, 0                 # yaw, yaw rate
    )
    time.sleep(duration)
    stop()

# Sağ veya sola dönüş (yaw)
def yaw_right(duration=2, yaw_rate=0.5):
    print(f"{duration} saniye boyunca yaw dönüşü (rate: {yaw_rate}).")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000000,  # sadece yaw kontrolü
        0, 0, 0,
        0, 0, 0,
        0, 0, yaw_rate,      # sadece yaw rate aktif
        0, 0
    )
    time.sleep(duration)
    stop()

# Araç durdur
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

# Görev Senaryosu
def kablo_takip_senaryosu():
    set_guided_mode()
    arm()

    move_forward(3)        # düz git
    yaw_right(1.5, 0.5)    # sağa dön
    move_forward(2)
    yaw_right(1.5, -0.5)   # sola dön
    move_forward(3)
    yaw_right(1.5, -0.5)   # tekrar sola dön
    move_forward(2)
    yaw_right(1.5, 0.5)    # tekrar sağa dön
    move_forward(3)

    stop()

# Kod başlatıcı
if __name__ == '__main__':
    kablo_takip_senaryosu()
