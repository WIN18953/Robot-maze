import sys
import time
from robomaster import robot

# 1. เชื่อมต่อหุ่นยนต์ผ่านโหมด AP
ep_robot = robot.Robot()
print("Connecting via AP Mode...")
ep_robot.initialize(conn_type="ap")

sensor = ep_robot.sensor
sensor_adaptor = ep_robot.sensor_adaptor
sensor_adaptor.start()

tof_front = 0

def on_tof_data(sub_info):
    global tof_front
    if isinstance(sub_info, (list, tuple)) and len(sub_info) > 0:
        tof_front = sub_info[0]
    elif isinstance(sub_info, (int, float)):
        tof_front = sub_info

# Subscribe ข้อมูล TOF ด้านหน้า
sensor.sub_distance(freq=10, callback=on_tof_data)

print("=" * 95)
print("             ROBOMASTER EP: 5-SENSOR LIVE MONITOR (PORT 1 ALL HUBS)            ")
print("=" * 95)
print("Hub 1: Sharp ขวา  |  Hub 2: Sharp ซ้าย  |  Hub 3: IR ขวา  |  Hub 4: IR ซ้าย  |  ToF หน้า")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        # อ่านค่า Analog (ADC) จาก Sharp เซนเซอร์ (Hub 1 ขวา, Hub 2 ซ้าย)
        sharp_r_adc = sensor_adaptor.get_adc(id=1, port=1)
        sharp_l_adc = sensor_adaptor.get_adc(id=2, port=1)

        # อ่านค่า Digital IO (0/1) จาก IR เซนเซอร์ (Hub 3 ขวา, Hub 4 ซ้าย)
        ir_r_raw = sensor_adaptor.get_io(id=3, port=1)
        ir_l_raw = sensor_adaptor.get_io(id=4, port=1)

        # แปลงสถานะ IR (0 = ติด/เจอกำแพง, 1 = ดับ/โล่ง)
        ir_r_txt = "DETECT (0)" if ir_r_raw == 0 else f"CLEAR  ({ir_r_raw})"
        ir_l_txt = "DETECT (0)" if ir_l_raw == 0 else f"CLEAR  ({ir_l_raw})"

        # แสดงผล Real-time บนบรรทัดเดียว
        line = (
            f"\r[ToF]: {str(tof_front):>4} mm | "
            f"[Sharp-L2]: {str(sharp_l_adc):>4} | "
            f"[Sharp-R1]: {str(sharp_r_adc):>4} | "
            f"[IR-L4]: {ir_l_txt} | "
            f"[IR-R3]: {ir_r_txt}"
        )
        sys.stdout.write(line)
        sys.stdout.flush()

        time.sleep(0.08)

except KeyboardInterrupt:
    print("\n\nMonitor stopped.")
finally:
    sensor.unsub_distance()
    sensor_adaptor.stop()
    ep_robot.close()