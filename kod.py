import time
from pymavlink import mavutil

# Pixhawk'a bağlan
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Pixhawk'a bağlandı!")

def test_motor(motor_number, pwm_value=1600, duration=2):
    print(f"Motor {motor_number} {pwm_value} PWM ile {duration} saniye çalışıyor.")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        motor_number, pwm_value, 0, 0, 0, 0, 0
    )
    time.sleep(duration)

    # Motoru durdur
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        motor_number, 1500, 0, 0, 0, 0, 0
    )
    time.sleep(1)

# Sadece 6 motoru test et
for i in range(1, 7):
    test_motor(i)
