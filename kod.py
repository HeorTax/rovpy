from pymavlink import mavutil
import time

# Pixhawk ile bağlantı kur
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Bağlantı kuruldu.")

# GUIDED moda geçiş
def set_guided_mode():
    master.set_mode_apm('GUIDED')
    print("GUIDED moda geçildi.")
    time.sleep(1)

# Motorları arming (çalıştırma)
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
    print(f"{duration} saniye boyunca ileri hareket.")
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity kontrolü
        0, 0, 0,
        speed, 0, 0,  # X yönü = ileri
        0, 0, 0,
        0, 0
    )
    time.sleep(duration)
    stop()

# Dönme (yaw)
def yaw_right(duration=2, yaw_rate=0.3):
    print("Sağa dönülüyor.")
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000000,
        0, 0, 0,
        0, 0, 0,
        0, 0, yaw_rate,
        0, 0
    )
    time.sleep(duration)
    stop()

# Araç durdur
def stop():
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
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

# Görev Senaryosu (şu an kamera yok – sadece zamanlama ile simülasyon)
def kablo_takip_senaryosu():
    set_guided_mode()
    arm()

    # Başlangıç düz yol
    move_forward(3)

    # Sağa dön - ilk dönüş
    yaw_right(1.5)
    move_forward(2)

    # Sola dön
    yaw_right(-1.5)
    move_forward(3)

    # Sola dön
    yaw_right(-1.5)
    move_forward(2)
from pymavlink import mavutil
import time

# Pixhawk ile bağlantı kur
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Bağlantı kuruldu.")

# GUIDED moda geçiş
def set_guided_mode():
    master.set_mode_apm('GUIDED')
    print("GUIDED moda geçildi.")
    time.sleep(1)

# Motorları arming (çalıştırma)
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
    print(f"{duration} saniye boyunca ileri hareket.")
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity kontrolü
        0, 0, 0,
        speed, 0, 0,  # X yönü = ileri
        0, 0, 0,
        0, 0
    )
    time.sleep(duration)
    stop()

# Dönme (yaw)
def yaw_right(duration=2, yaw_rate=0.3):
    print("Sağa dönülüyor.")
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000000,
        0, 0, 0,
        0, 0, 0,
        0, 0, yaw_rate,
        0, 0
    )
    time.sleep(duration)
    stop()

# Araç durdur
def stop():
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
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

# Görev Senaryosu (şu an kamera yok – sadece zamanlama ile simülasyon)
def kablo_takip_senaryosu():
    set_guided_mode()
    arm()

    # Başlangıç düz yol
    move_forward(3)

    # Sağa dön - ilk dönüş
    yaw_right(1.5)
    move_forward(2)

    # Sola dön
    yaw_right(-1.5)
    move_forward(3)

    # Sola dön
    yaw_right(-1.5)
    move_forward(2)

    # Sağa dön
    yaw_right(1.5)
    move_forward(3)

    # Durdur
    stop()

# Kod çalıştırma
if __name__ == '__main__':
    kablo_takip_senaryosu()
    # Sağa dön
    yaw_right(1.5)
    move_forward(3)

    # Durdur
    stop()

# Kod çalıştırma
if _name_ == '_main_':
    kablo_takip_senaryosu()
