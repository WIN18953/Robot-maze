from collections import deque
import time
from robomaster import robot

# =============================================================================
# 1. การตั้งค่าระบบ Grid 4x4
# =============================================================================
GRID_SIZE = 0.60  # ระยะทาง 1 ช่อง = 60 cm
GOAL_POS = (1, 1)

BASE_SPEED = 0.12 # ความเร็วปกติเดินหน้า (m/s)
TURN_SPEED = 35   # ความเร็วหมุนตัว (deg/s)

SAFETY_DIST_STOP = 100  # ระยะเบรกฉุกเฉิน ToF (10 cm = 100 mm)

KP_YAW = 1.4
KD_YAW = 0.08
KP_WALL = 0.0020  # ความไวการคุมระยะห่างด้วย Sharp
MAX_CORRECTION_Y = 0.25 # ขีดจำกัดความเร็วสไลด์เบี่ยงตัว

DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 0:N, 1:E, 2:S, 3:W
DIR_NAMES = ["NORTH (^)", "EAST (>)", "SOUTH (v)", "WEST (<)"]
DIR_ICONS = ["^", ">", "v", "<"]
YAW_TARGETS = [0.0, 90.0, 180.0, -90.0]

maze_map = {}
step_history = {(0, 0): 0}
step_counter = 0
current_pos = (2, 2)
current_dir_idx = 0
visited_nodes = {(0, 0)}

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

def bfs_path_to_goal(start, target, visited_global):
    queue = deque([[start]])
    visited_local = {start}
    unexplored_paths = []

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == target:
            return path

        if node not in maze_map:
            unexplored_paths.append(path)
            continue

        for d_idx, (dx, dy) in enumerate(DIRS):
            neighbor = (node[0] + dx, node[1] + dy)
            if 0 <= neighbor[0] < 4 and 0 <= neighbor[1] < 4:
                wall_status = maze_map[node].get(d_idx, "OPEN")
                if wall_status != "WALL" and neighbor not in visited_local:
                    visited_local.add(neighbor)
                    queue.append(path + [neighbor])

    if unexplored_paths:
        unexplored_paths.sort(key=lambda p: (
            sum(1 for step in p[1:] if step in visited_global),
            len(p), 
            abs(p[-1][0] - target[0]) + abs(p[-1][1] - target[1])
        ))
        return unexplored_paths[0]

    return None

def main():
    global current_pos, current_dir_idx, step_counter, visited_nodes

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

    print("=" * 70)
    print(" Calibrating Reference Values ")
    print("=" * 70)

    samples_r, samples_l = [], []
    start_time = time.time()

    while time.time() - start_time < 2.0:
        val_l = sensor_adaptor.get_adc(id=3, port=1) or 0
        val_r = sensor_adaptor.get_adc(id=4, port=2) or 0
        if val_r > 0: samples_r.append(val_r)
        if val_l > 0: samples_l.append(val_l)
        time.sleep(0.05)

    ref_sharp_r = sum(samples_r) / len(samples_r) if samples_r else 550
    ref_sharp_l = sum(samples_l) / len(samples_l) if samples_l else 550
    initial_yaw_offset = current_yaw

    threshold_r = ref_sharp_r - 80
    threshold_l = ref_sharp_l - 80

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
        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.1)

    def scan_surroundings_if_needed():
        global current_dir_idx
        origin_dir = current_dir_idx
        
        if current_pos not in maze_map:
            maze_map[current_pos] = {}

        dirs_to_check = [
            origin_dir,
            (origin_dir + 1) % 4,
            (origin_dir + 3) % 4
        ]

        needs_scan = any(d not in maze_map[current_pos] for d in dirs_to_check)

        if needs_scan:
            print("🔍 พบพื้นที่ใหม่ สแกน ToF เพิ่มเติม...")
            if origin_dir not in maze_map[current_pos]:
                time.sleep(0.15)
                maze_map[current_pos][origin_dir] = "WALL" if (0 < tof_front <= 480) else "OPEN"

            r_dir = (origin_dir + 1) % 4
            if r_dir not in maze_map[current_pos]:
                rotate_to_dir(r_dir)
                time.sleep(0.15)
                maze_map[current_pos][r_dir] = "WALL" if (0 < tof_front <= 480) else "OPEN"

            l_dir = (origin_dir + 3) % 4
            if l_dir not in maze_map[current_pos]:
                rotate_to_dir(l_dir)
                time.sleep(0.15)
                maze_map[current_pos][l_dir] = "WALL" if (0 < tof_front <= 480) else "OPEN"

            rotate_to_dir(origin_dir)

    def drive_one_grid_pid():
        duration = GRID_SIZE / BASE_SPEED
        t_start = time.time()
        last_yaw_error = 0.0
        target_yaw = initial_yaw_offset + YAW_TARGETS[current_dir_idx]

        while time.time() - t_start < duration:
            # 🛑 เบรกฉุกเฉิน ห้ามชนกำแพงระยะ <= 10 cm (100 mm)
            if 0 < tof_front <= SAFETY_DIST_STOP:
                chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                print(f"🛑 Emergency Brake! ตรวจพบสิ่งกีดขวางด้านหน้า {tof_front} mm (<= 10 cm)")
                break

            sharp_l = sensor_adaptor.get_adc(id=3, port=1) or 0
            sharp_r = sensor_adaptor.get_adc(id=4, port=2) or 0
            
            # อ่านค่า IR เซนเซอร์ ซ้าย (Port 1) และ ขวา (Port 2)
            ir_l = sensor_adaptor.get_io(id=1, port=1)
            ir_r = sensor_adaptor.get_io(id=2, port=1)

            yaw_error = normalize_angle(target_yaw - current_yaw)
            d_yaw = yaw_error - last_yaw_error
            z_speed = (KP_YAW * yaw_error) + (KD_YAW * d_yaw)
            last_yaw_error = yaw_error

            current_x_speed = BASE_SPEED
            y_speed = 0.0

            # ---------------------------------------------------------
            # 🚨 ระบบ IR บังคับเบี่ยงหนีทันที (แก้ทิศทางสไลด์หนี ไม่ให้ดูดเข้าหา)
            # ---------------------------------------------------------
            if ir_l == 0:  # IR ซ้ายพบสิ่งกีดขวาง -> สไลด์ไปขวาอย่างแรง (+Y)
                print("🚨 IR ซ้ายพบสิ่งกีดขวาง! สไลด์หนีไปทางขวาทันที")
                current_x_speed = 0.04  # ลดความเร็วเดินหน้าเพื่อให้หลบพ้น
                y_speed = 0.23          # ค่า +Y สไลด์ออกขวาหนีกำแพงซ้าย
                z_speed = 20.0          # หมุนเบี่ยงทิศช่วย

            elif ir_r == 0:  # IR ขวาพบสิ่งกีดขวาง -> สไลด์ไปซ้ายอย่างแรง (-Y)
                print("🚨 IR ขวาพบสิ่งกีดขวาง! สไลด์หนีไปทางซ้ายทันที")
                current_x_speed = 0.04  # ลดความเร็วเดินหน้าเพื่อให้หลบพ้น
                y_speed = -0.23         # ค่า -Y สไลด์ออกซ้ายหนีกำแพงขวา
                z_speed = -20.0         # หมุนเบี่ยงทิศช่วย

            else:
                # ---------------------------------------------------------
                # 2. ระบบ Sharp Sensor (ทำงานเฉพาะเวลา IR ไม่เจอวัตถุระยะประชิด)
                # ---------------------------------------------------------
                has_wall_r = (sharp_r >= threshold_r)
                has_wall_l = (sharp_l >= threshold_l)

                if has_wall_r and has_wall_l:
                    diff = (sharp_r - ref_sharp_r) - (sharp_l - ref_sharp_l)
                    y_speed = KP_WALL * diff
                elif has_wall_r:
                    # ใกล้ขวา -> สไลด์ไปซ้าย (-Y)
                    err_r = sharp_r - ref_sharp_r
                    y_speed = -KP_WALL * err_r
                elif has_wall_l:
                    # ใกล้ซ้าย -> สไลด์ไปขวา (+Y)
                    err_l = sharp_l - ref_sharp_l
                    y_speed = KP_WALL * err_l

            y_speed = max(min(y_speed, MAX_CORRECTION_Y), -MAX_CORRECTION_Y)

            chassis.drive_speed(x=current_x_speed, y=y_speed, z=z_speed)
            time.sleep(0.02)

        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)

    try:
        while True:
            if current_pos == GOAL_POS:
                rotate_to_dir(0)
                print(f"\n🎉 บรรลุเป้าหมายพิกัด {GOAL_POS} เรียบร้อยแล้ว!")
                print_fixed_map()
                break

            scan_surroundings_if_needed()

            b_dir = (current_dir_idx + 2) % 4
            if b_dir not in maze_map[current_pos]:
                maze_map[current_pos][b_dir] = "OPEN"

            x, y = current_pos
            if y == 3: maze_map[current_pos][0] = "WALL"
            if x == 3: maze_map[current_pos][1] = "WALL"
            if y == 0: maze_map[current_pos][2] = "WALL"
            if x == 0: maze_map[current_pos][3] = "WALL"

            for d_idx in range(4):
                if maze_map[current_pos].get(d_idx) == "WALL":
                    dx, dy = DIRS[d_idx]
                    neighbor = (x + dx, y + dy)
                    if 0 <= neighbor[0] < 4 and 0 <= neighbor[1] < 4:
                        if neighbor not in maze_map:
                            maze_map[neighbor] = {}
                        maze_map[neighbor][(d_idx + 2) % 4] = "WALL"

            print_fixed_map()

            path = bfs_path_to_goal(current_pos, GOAL_POS, visited_nodes)
            if not path or len(path) < 2:
                print("❌ สำรวจจนครบพื้นที่แล้ว")
                break

            next_node = path[1]
            dx = next_node[0] - current_pos[0]
            dy = next_node[1] - current_pos[1]
            target_dir = DIRS.index((dx, dy))

            rotate_to_dir(target_dir)
            drive_one_grid_pid()

            current_pos = next_node
            visited_nodes.add(current_pos)
            
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