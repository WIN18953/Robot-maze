from collections import deque
import time
from robomaster import robot

# =============================================================================
# 1. การตั้งค่าระบบ Grid 4x4 (0,0 ถึง 3,3)
# =============================================================================
GRID_SIZE = 0.60  # ระยะทาง 1 ช่อง = 60 cm
GOAL_POS = (3, 3) # พิกัดเป้าหมายสูงสุด

BASE_SPEED = 0.16 # ความเร็วเดินหน้า (m/s)
TURN_SPEED = 45   # ความเร็วหมุนตัว (deg/s)

# พารามิเตอร์ PID
KP_YAW = 1.4      # สัมประสิทธิ์คุมมุมองศาหัวรถให้ตรง
KD_YAW = 0.08     # แดมป์การส่ายของหัวรถ
KP_WALL = 0.0007  # สัมประสิทธิ์สไลด์รักษากึ่งกลางเลน
MAX_CORRECTION_Y = 0.10

DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 0:N, 1:E, 2:S, 3:W
DIR_NAMES = ["NORTH (^)", "EAST (>)", "SOUTH (v)", "WEST (<)"]
DIR_ICONS = ["^", ">", "v", "<"]
YAW_TARGETS = [0.0, 90.0, 180.0, -90.0]

maze_map = {}
step_history = {(0, 0): 0}
step_counter = 0
current_pos = (0, 0)
current_dir_idx = 0

# =============================================================================
# 2. ฟังก์ชันวาดแผนที่ Fixed 4x4 Grid
# =============================================================================
def print_fixed_map():
    global current_pos, current_dir_idx, GOAL_POS, step_history, maze_map
    print("\n" + "=" * 48)
    print(f"      4x4 MAZE NAVIGATOR  |  TARGET: {GOAL_POS}  ")
    print("=" * 48)

    for y in range(3, -1, -1):
        top_str = ""
        for x in range(4):
            cell = (x, y)
            if cell in maze_map and maze_map[cell].get(0) == "WALL":
                top_str += "+---"
            else:
                top_str += "+   " if cell in maze_map else "+..."
        print(top_str + "+")

        mid_str = ""
        for x in range(4):
            cell = (x, y)
            wall_w = "|" if (cell in maze_map and maze_map[cell].get(3) == "WALL") else (" " if cell in maze_map else ".")
            
            if cell == current_pos:
                content = f"[{DIR_ICONS[current_dir_idx]}]"
            elif cell == GOAL_POS:
                content = " *G*"
            elif cell in step_history:
                content = f" {step_history[cell]:>2}"
            elif cell in maze_map:
                content = " . "
            else:
                content = " ? "
            mid_str += wall_w + content
        
        last_cell = (3, y)
        wall_e = "|" if (last_cell in maze_map and maze_map[last_cell].get(1) == "WALL") else "."
        print(mid_str + wall_e + f"  (Y={y})")

    bot_str = ""
    for x in range(4):
        cell = (x, 0)
        bot_str += "+---" if (cell in maze_map and maze_map[cell].get(2) == "WALL") else "+..."
    print(bot_str + "+")
    print("  X=0  X=1  X=2  X=3")
    print(f"\n📍 Position: {current_pos} | Heading: {DIR_NAMES[current_dir_idx]}")
    print("=" * 48 + "\n")

# =============================================================================
# 3. BFS Pathfinding (ปรับแก้ไขให้รองรับ Backtracking ไปช่อง Unexplored ?)
# =============================================================================
def bfs_path_to_goal(start, target):
    queue = deque([[start]])
    visited = {start}

    while queue:
        path = queue.popleft()
        node = path[-1]

        # เจอเป้าหมาย หรือเจอช่องที่ไม่เคยเข้าสแกน
        if node == target or node not in maze_map:
            return path

        for d_idx, (dx, dy) in enumerate(DIRS):
            neighbor = (node[0] + dx, node[1] + dy)
            if 0 <= neighbor[0] < 4 and 0 <= neighbor[1] < 4:
                # ทางต้องไม่ติดกำแพงใน maze_map
                wall_status = maze_map[node].get(d_idx, "OPEN")
                if wall_status != "WALL" and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])
    return None

# =============================================================================
# 4. ฟังก์ชันอ่านค่า Sensor พร้อมทำ Filter
# =============================================================================
def get_filtered_adc(sensor_adaptor, sensor_id, port, samples=3):
    vals = []
    for _ in range(samples):
        v = sensor_adaptor.get_adc(id=sensor_id, port=port) or 0
        if v > 0: vals.append(v)
        time.sleep(0.01)
    return sum(vals) // len(vals) if vals else 0

# =============================================================================
# 5. ฟังก์ชันหลัก
# =============================================================================
def main():
    global current_pos, current_dir_idx, step_counter

    ep_robot = robot.Robot()
    print("Connecting via AP Mode...")
    ep_robot.initialize(conn_type="ap")

    chassis = ep_robot.chassis
    sensor = ep_robot.sensor
    sensor_adaptor = ep_robot.sensor_adaptor
    sensor_adaptor.start()

    tof_front = 9999
    current_yaw = 0.0

    def on_tof_data(sub_info):
        nonlocal tof_front
        if isinstance(sub_info, (list, tuple)) and len(sub_info) > 0:
            tof_front = sub_info[0]
        elif isinstance(sub_info, (int, float)):
            tof_front = sub_info

    def on_attitude_data(sub_info):
        nonlocal current_yaw
        yaw, pitch, roll = sub_info
        current_yaw = yaw

    sensor.sub_distance(freq=20, callback=on_tof_data)
    chassis.sub_attitude(freq=20, callback=on_attitude_data)

    # -------------------------------------------------------------------------
    # Calibration (Port 3=Sharp ซ้าย, Port 4=Sharp ขวา)
    # -------------------------------------------------------------------------
    print("=" * 70)
    print(" 3 วินาทีแรก: วางหุ่นตรงกึ่งกลางช่อง (0,0) เพื่อจำ Reference ซ้าย-ขวา ")
    print("=" * 70)

    samples_r = []
    samples_l = []
    start_time = time.time()

    while time.time() - start_time < 3.0:
        val_l = sensor_adaptor.get_adc(id=1, port=3) or 0
        val_r = sensor_adaptor.get_adc(id=2, port=4) or 0
        if val_r > 0: samples_r.append(val_r)
        if val_l > 0: samples_l.append(val_l)
        
        rem_time = 3.0 - (time.time() - start_time)
        print(f"\r[CALIBRATING: {rem_time:.1f}s] Sharp-L: {val_l:4d} | Sharp-R: {val_r:4d} | ToF: {tof_front:4d}mm", end="")
        time.sleep(0.05)

    ref_sharp_r = sum(samples_r) / len(samples_r) if samples_r else 550
    ref_sharp_l = sum(samples_l) / len(samples_l) if samples_l else 550
    initial_yaw_offset = current_yaw

    threshold_r = ref_sharp_r - 60
    threshold_l = ref_sharp_l - 60

    print(f"\n\n>>> Target R={ref_sharp_r:.0f} | Target L={ref_sharp_l:.0f} | Yaw-Offset={initial_yaw_offset:.1f}° <<<")

    def normalize_angle(angle):
        while angle > 180.0: angle -= 360.0
        while angle <= -180.0: angle += 360.0
        return angle

    def rotate_to_dir(target_dir):
        global current_dir_idx
        turn_steps = (target_dir - current_dir_idx) % 4
        if turn_steps == 0:
            return

        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.1)

        if turn_steps == 1:
            chassis.move(x=0, y=0, z=-90, z_speed=TURN_SPEED).wait_for_completed()
        elif turn_steps == 2:
            chassis.move(x=0, y=0, z=180, z_speed=TURN_SPEED).wait_for_completed()
        elif turn_steps == 3:
            chassis.move(x=0, y=0, z=90, z_speed=TURN_SPEED).wait_for_completed()
        current_dir_idx = target_dir
        time.sleep(0.2)

    def drive_one_grid_pid():
        duration = GRID_SIZE / BASE_SPEED
        t_start = time.time()
        last_yaw_error = 0.0

        target_yaw = initial_yaw_offset + YAW_TARGETS[current_dir_idx]

        while time.time() - t_start < duration:
            if 0 < tof_front <= 200:
                break

            sharp_l = sensor_adaptor.get_adc(id=3, port=1) or 0
            sharp_r = sensor_adaptor.get_adc(id=4, port=2) or 0
            ir_l = sensor_adaptor.get_io(id=1, port=1)
            ir_r = sensor_adaptor.get_io(id=2, port=1)

            yaw_error = normalize_angle(target_yaw - current_yaw)
            d_yaw = yaw_error - last_yaw_error
            z_speed = (KP_YAW * yaw_error) + (KD_YAW * d_yaw)
            last_yaw_error = yaw_error

            if ir_r == 0:
                y_speed = -0.12
            elif ir_l == 0:
                y_speed = 0.12
            else:
                has_wall_r = (sharp_r >= threshold_r)
                has_wall_l = (sharp_l >= threshold_l)

                if has_wall_r and has_wall_l:
                    diff = (sharp_l - ref_sharp_l) - (sharp_r - ref_sharp_r)
                    y_speed = KP_WALL * diff
                elif has_wall_r:
                    err_r = ref_sharp_r - sharp_r
                    y_speed = KP_WALL * err_r
                elif has_wall_l:
                    err_l = ref_sharp_l - sharp_l
                    y_speed = -KP_WALL * err_l
                else:
                    y_speed = 0.0

                y_speed = max(min(y_speed, MAX_CORRECTION_Y), -MAX_CORRECTION_Y)

            chassis.drive_speed(x=BASE_SPEED, y=y_speed, z=z_speed)
            time.sleep(0.02)

        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.25)

    # =========================================================================
    # Main Navigation Loop
    # =========================================================================
    try:
        while True:
            if current_pos == GOAL_POS:
                rotate_to_dir(0)
                print(f"\n🎉 บรรลุเป้าหมายพิกัด {GOAL_POS} เรียบร้อยแล้ว!")
                print_fixed_map()
                break

            time.sleep(0.25)
            # แก้ไขการดึงค่าด้วย Port ให้ถูกต้อง
            sharp_l = get_filtered_adc(sensor_adaptor, 1, port=3)
            sharp_r = get_filtered_adc(sensor_adaptor, 2, port=4)
            
            front_wall = (0 < tof_front < 400)
            right_wall = (sharp_r >= threshold_r)
            left_wall  = (sharp_l >= threshold_l)

            if current_pos not in maze_map:
                maze_map[current_pos] = {}

            # แปลงทิศสัมพัทธ์ (Relative) เป็นทิศทางจริง (Absolute Grid Direction)
            f_dir = current_dir_idx
            r_dir = (current_dir_idx + 1) % 4
            b_dir = (current_dir_idx + 2) % 4
            l_dir = (current_dir_idx + 3) % 4

            maze_map[current_pos][f_dir] = "WALL" if front_wall else "OPEN"
            maze_map[current_pos][r_dir] = "WALL" if right_wall else "OPEN"
            maze_map[current_pos][l_dir] = "WALL" if left_wall else "OPEN"
            
            if b_dir not in maze_map[current_pos]:
                maze_map[current_pos][b_dir] = "OPEN"

            # ขอบสนามภายนอก 4x4
            x, y = current_pos
            if y == 3: maze_map[current_pos][0] = "WALL"
            if x == 3: maze_map[current_pos][1] = "WALL"
            if y == 0: maze_map[current_pos][2] = "WALL"
            if x == 0: maze_map[current_pos][3] = "WALL"

            # บันทึกข้อมูลกำแพงสองฝั่งไปยัง Cell ข้างเคียงให้สอดคล้องกัน
            for d_idx in range(4):
                if maze_map[current_pos].get(d_idx) == "WALL":
                    dx, dy = DIRS[d_idx]
                    neighbor = (x + dx, y + dy)
                    if 0 <= neighbor[0] < 4 and 0 <= neighbor[1] < 4:
                        if neighbor not in maze_map:
                            maze_map[neighbor] = {}
                        maze_map[neighbor][(d_idx + 2) % 4] = "WALL"

            print(f"Debug: Front={tof_front}mm | R={sharp_r} (Th:{threshold_r:.0f}) | L={sharp_l} (Th:{threshold_l:.0f})")
            print_fixed_map()

            # คำนวณเส้นทาง BFS
            path = bfs_path_to_goal(current_pos, GOAL_POS)
            if not path or len(path) < 2:
                print("❌ สำรวจจนหมดแล้ว ไม่มีเส้นทางไปถึงเป้าหมายได้")
                break

            # แปลงพิกัดและหมุนตัว
            next_node = path[1]
            dx = next_node[0] - current_pos[0]
            dy = next_node[1] - current_pos[1]
            target_dir = DIRS.index((dx, dy))

            rotate_to_dir(target_dir)

            # Re-verify ระยะ ToF หลังจากหมุนตัวเสร็จแล้ว
            time.sleep(0.2)
            if 0 < tof_front <= 200:
                print("⚠️ ตรวจพบกำแพงขวางหน้า อัปเดตแผนที่และหาเส้นทางใหม่...")
                maze_map[current_pos][target_dir] = "WALL"
                
                # บันทึกทางฝั่งเพื่อนบ้านด้วย
                neighbor = next_node
                if 0 <= neighbor[0] < 4 and 0 <= neighbor[1] < 4:
                    if neighbor not in maze_map:
                        maze_map[neighbor] = {}
                    maze_map[neighbor][(target_dir + 2) % 4] = "WALL"
                continue

            # เดินหน้าไป 1 ช่อง
            drive_one_grid_pid()

            # อัปเดตทางเชื่อมย้อนกลับ
            back_dir = (target_dir + 2) % 4
            if next_node not in maze_map:
                maze_map[next_node] = {}
            maze_map[next_node][back_dir] = "OPEN"

            current_pos = next_node
            if current_pos not in step_history:
                step_counter += 1
                step_history[current_pos] = step_counter

    except KeyboardInterrupt:
        print("\nหยุดการทำงานโดยผู้ใช้")

    finally:
        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        sensor.unsub_distance()
        chassis.unsub_attitude()
        sensor_adaptor.stop()
        ep_robot.close()
        print("ปิดระบบเรียบร้อย")

if __name__ == '__main__':
    main()