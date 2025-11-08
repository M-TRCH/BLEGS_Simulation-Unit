import pybullet as p
import pybullet_data
import time
import math

# 1. เชื่อมต่อกับ Physics Server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# 2. โหลดหุ่นยนต์
urdf_path = "urdf/my_robot.urdf"
startPos = [0, 0, 0.5] # ยังเริ่มที่ 0.5m เหมือนเดิม
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

# 3. รวบรวม Joints และ Links
joint_name_to_id = {}
link_name_to_id = {}
controllable_joints_ids = [] 

num_joints = p.getNumJoints(robotId)
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    link_name = joint_info[12].decode('utf-8') 
    joint_name_to_id[joint_name] = i
    link_name_to_id[link_name] = i 
    if joint_type == p.JOINT_REVOLUTE:
        controllable_joints_ids.append(i)

FR_foot_link_id = link_name_to_id['FR_foot_link']
FL_foot_link_id = link_name_to_id['FL_foot_link']
RR_foot_link_id = link_name_to_id['RR_foot_link']
RL_foot_link_id = link_name_to_id['RL_foot_link']

FR_joint_ids = [joint_name_to_id['FR_hip_joint'], joint_name_to_id['FR_thigh_joint'], joint_name_to_id['FR_shank_joint']]
FL_joint_ids = [joint_name_to_id['FL_hip_joint'], joint_name_to_id['FL_thigh_joint'], joint_name_to_id['FL_shank_joint']]
RR_joint_ids = [joint_name_to_id['RR_hip_joint'], joint_name_to_id['RR_thigh_joint'], joint_name_to_id['RR_shank_joint']]
RL_joint_ids = [joint_name_to_id['RL_hip_joint'], joint_name_to_id['RL_thigh_joint'], joint_name_to_id['RL_shank_joint']]

# =====================================================================
# 4. ตั้งค่าพารามิเตอร์การเดิน (Slower Gait)
# =====================================================================

base_x = 0.195 
base_y = 0.145 
z_height = -0.25 # ความสูงที่ยืน (ต่ำกว่าตัว 25cm)

home_foot_positions = {
    'FR_foot_link': [base_x, -base_y, z_height],
    'FL_foot_link': [base_x, base_y, z_height],
    'RR_foot_link': [-base_x, -base_y, z_height],
    'RL_foot_link': [-base_x, base_y, z_height],
}

# --- SLOWER GAIT PARAMS ---
STEP_LENGTH = 0.05  # ก้าวสั้นลง
LIFT_HEIGHT = 0.05 
STEP_TIME = 0.6     # ใช้เวลานานขึ้น

gait_state = 0 
state_timer = 0.0
sim_time = 0.0
time_step = 1./240. 

pair_1 = ['FR_foot_link', 'RL_foot_link']
pair_2 = ['FL_foot_link', 'RR_foot_link']

# --- WARM-UP / SETTLING LOGIC ---
WARMUP_TIME = 2.0 # ให้เวลา 2 วินาทีในการ "ยืน" ให้นิ่งก่อน
is_walking = False # เริ่มต้นโดยยังไม่เดิน


# =====================================================================
# 5. Main Simulation Loop
# =====================================================================
print("\n--- เริ่มการจำลอง (Gait Control v4: Stiff) ---") # CHANGED
print(f"--- WARMING UP for {WARMUP_TIME} seconds... ---")
try:
    while True:
        # อัปเดตเวลา
        sim_time += time_step
        
        # 5.1: WARM-UP LOGIC
        if not is_walking and sim_time >= WARMUP_TIME:
            is_walking = True
            print("--- WARM-UP COMPLETE. Starting Trot Gait. ---")
            state_timer = 0.0 # รีเซ็ตไทม์เมอร์สำหรับ Gait
        
        # 5.2: State Machine Logic
        if is_walking:
            state_timer += time_step
            if state_timer >= STEP_TIME:
                state_timer = 0.0 
                gait_state = (gait_state + 1) % 2 
        
        # 5.3: คำนวณพิกัดเป้าหมายของเท้า
        target_foot_positions_REL = {} 
        
        if is_walking:
            # --- TROT LOGIC ---
            state_progress = state_timer / STEP_TIME
            if gait_state == 0:
                swing_legs = pair_1
                stance_legs = pair_2
            else:
                swing_legs = pair_2
                stance_legs = pair_1

            for leg_name in swing_legs:
                home_pos = home_foot_positions[leg_name]
                z_move = LIFT_HEIGHT * math.sin(state_progress * math.pi)
                x_move = STEP_LENGTH * (state_progress * 2 - 1.0)
                target_foot_positions_REL[leg_name] = [home_pos[0] + x_move, home_pos[1], home_pos[2] + z_move]

            for leg_name in stance_legs:
                home_pos = home_foot_positions[leg_name]
                x_shift = STEP_LENGTH * (1.0 - state_progress * 2)
                target_foot_positions_REL[leg_name] = [home_pos[0] + x_shift, home_pos[1], home_pos[2]]
        
        else:
            # --- WARM-UP STATE: ยืนนิ่งๆ ที่ท่า Home ---
            target_foot_positions_REL = home_foot_positions


        # 5.4: อ่านตำแหน่ง Base
        basePos, baseOrn = p.getBasePositionAndOrientation(robotId)

        # 5.5: วนลูป IK สำหรับแต่ละขา
        for foot_link_name, target_pos_REL in target_foot_positions_REL.items():
            
            if foot_link_name == 'FR_foot_link':
                current_leg_joint_ids = FR_joint_ids
                current_foot_link_id = FR_foot_link_id
            elif foot_link_name == 'FL_foot_link':
                current_leg_joint_ids = FL_joint_ids
                current_foot_link_id = FL_foot_link_id
            elif foot_link_name == 'RR_foot_link':
                current_leg_joint_ids = RR_joint_ids
                current_foot_link_id = RR_foot_link_id
            elif foot_link_name == 'RL_foot_link':
                current_leg_joint_ids = RL_joint_ids
                current_foot_link_id = RL_foot_link_id
            else:
                continue

            world_target_pos, _ = p.multiplyTransforms(basePos, baseOrn, target_pos_REL, [0, 0, 0, 1])
            joint_angles_for_leg = p.calculateInverseKinematics(
                robotId, current_foot_link_id, world_target_pos, 
                jointDamping=[0.1] * len(current_leg_joint_ids), maxNumIterations=50)

            # --- (!!! THIS IS THE CHANGED PART !!!) ---
            # เพิ่ม Gain ให้ "แข็ง" (Stiff) ขึ้น
            p.setJointMotorControlArray(
                robotId, current_leg_joint_ids, p.POSITION_CONTROL,
                targetPositions=joint_angles_for_leg[:len(current_leg_joint_ids)],
                forces=[10] * len(current_leg_joint_ids),
                positionGains=[0.8] * len(current_leg_joint_ids), # CHANGED: 0.3 -> 0.8
                velocityGains=[1.0] * len(current_leg_joint_ids)  # CHANGED: 0.5 -> 1.0
            )
            # --- (!!! END OF CHANGE !!!) ---
        
        # 5.6: Step Simulation
        p.stepSimulation()
        time.sleep(time_step) 

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")

except p.error as e:
    print(f"\n--- เกิดข้อผิดพลาดร้ายแรงกับ PyBullet! ---")
    print(e)
    # อาจจะเกิดจากหาไฟล์ plane.urdf ไม่เจอ (ถ้า path ผิด)
    
finally:
    if p.isConnected():
        p.disconnect()