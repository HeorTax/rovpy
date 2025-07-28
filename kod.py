from pymavlink import mavutil
import time

# Pixhawk bağlantısı
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Pixhawk bağlantısı kuruldu!")

# MANUAL moda geç (ArduSub motor testlerinde genellikle bu mod aktif olur)
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"{mode} moduna geçiliyor...")
    time.sleep(1)

# Belirli motora PWM değeri gönder
def set_servo_pwm(servo_number, pwm_value):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value,
        0, 0, 0, 0, 0)
    print(f"Motor {servo_number} → PWM: {pwm_value}")

# Motorları sırayla test et
def test_all_motors():
    set_mode("MANUAL")  # ArduSub'da motor sürmek için genelde MANUAL mod gerekir

    for servo in range(1, 7):  # SERVO1 - SERVO6
        print(f"--- Motor {servo} test ediliyor ---")
        set_servo_pwm(servo, 1700)  # motoru çalıştır (orta değeri geçmeli)
        time.sleep(2)
        set_servo_pwm(servo, 1500)  # motoru durdur
        time.sleep(1)

    print("Tüm motorlar test edildi.")

# Ana fonksiyon
if __name__ == "__main__":
    test_all_motors()
