import pybullet as p
import pybullet_data
import time
import math

# 1. เชื่อมต่อกับ Physics Server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# 2. โหลดหุ่นยนต์ (URDF ที่ hip_joint เป็น fixed)
urdf_path = "urdf/my_robot.urdf"
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

# =====================================================================
# 3. (CHANGED) รวบรวม Joints และ Links (แก้บั๊ก .strip() ที่แท้จริง)
# =====================================================================
joint_name_to_id = {}
link_name_to_id = {}

num_joints = p.getNumJoints(robotId)
print(f"--- (v13 Debug) Reading {num_joints} Joints ---")

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    
    # (!!! THIS IS THE FIX !!!)
    # แทนที่ช่องว่างพิเศษ (ASCII 160) ก่อน แล้วค่อย strip
    joint_name = joint_info[1].decode('utf-8').replace(u'\xa0', u' ').strip()
    link_name = joint_info[12].decode('utf-8').replace(u'\xa0', u' ').strip()
    # (!!! END OF FIX !!!)
    
    print(f"  Found Joint Index {i}: '{joint_name}' (Type: {joint_info[2]})") # DEBUG
    
    joint_name_to_id[joint_name] = i
    if link_name: 
        link_name_to_id[link_name] = i 

# รวบรวม controllable joints
controllable_joints_ids = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    if joint_type == p.JOINT_REVOLUTE:
        controllable_joints_ids.append(i)

# สร้าง mapping จาก joint ID ไปยัง index ใน IK result
joint_id_to_ik_index = {joint_id: idx for idx, joint_id in enumerate(controllable_joints_ids)}

print("--- Debug: Finished populating dictionary ---")

FR_foot_link_id = link_name_to_id['FR_foot_link']
FL_foot_link_id = link_name_to_id['FL_foot_link']
RR_foot_link_id = link_name_to_id['RR_foot_link']
RL_foot_link_id = link_name_to_id['RL_foot_link']

FR_moveable_joint_ids = [joint_name_to_id['FR_thigh_joint'], joint_name_to_id['FR_shank_joint']]
FL_moveable_joint_ids = [joint_name_to_id['FL_thigh_joint'], joint_name_to_id['FL_shank_joint']]
RR_moveable_joint_ids = [joint_name_to_id['RR_thigh_joint'], joint_name_to_id['RR_shank_joint']]
RL_moveable_joint_ids = [joint_name_to_id['RL_thigh_joint'], joint_name_to_id['RL_shank_joint']]

FR_all_joint_ids = [joint_name_to_id['FR_hip_joint'], joint_name_to_id['FR_thigh_joint'], joint_name_to_id['FR_shank_joint']]
FL_all_joint_ids = [joint_name_to_id['FL_hip_joint'], joint_name_to_id['FL_thigh_joint'], joint_name_to_id['FL_shank_joint']]
RR_all_joint_ids = [joint_name_to_id['RR_hip_joint'], joint_name_to_id['RR_thigh_joint'], joint_name_to_id['RR_shank_joint']]
RL_all_joint_ids = [joint_name_to_id['RL_hip_joint'], joint_name_to_id['RL_thigh_joint'], joint_name_to_id['RL_shank_joint']]

# =====================================================================
# 4. ตั้งค่าพารามิเตอร์ (จูนจาก v12)
# =====================================================================
base_x = 0.195 
base_y = 0.145 
z_height = -0.25 
home_foot_positions = {
    'FR_foot_link': [base_x, -base_y, z_height],
    'FL_foot_link': [base_x, base_y, z_height],
    'RR_foot_link': [-base_x, -base_y, z_height],
    'RL_foot_link': [-base_x, base_y, z_height],
}

STEP_LENGTH = 0.04  # ลดจาก 0.05 เป็น 0.04 (ก้าวเล็กลง)
LIFT_HEIGHT = 0.04  # ลดจาก 0.05 เป็น 0.04 (ยกเท้าต่ำลง)
STEP_TIME = 0.6     # เพิ่มจาก 0.5 เป็น 0.6 (เดินช้าลง)

gait_state = 0 
state_timer = 0.0
sim_time = 0.0
time_step = 1./240. 
pair_1 = ['FR_foot_link', 'RL_foot_link']
pair_2 = ['FL_foot_link', 'RR_foot_link']
WARMUP_TIME = 2.0 
is_walking = False 

BALANCE_KP_PITCH = 0.005  # ลดจาก 0.02 เป็น 0.005 (ชดเชยนุ่มนวลขึ้น)
BALANCE_KD_PITCH = 0.01   # ลดจาก 0.05 เป็น 0.01 (ลดการตอบสนองเร็ว)
prev_pitch_error = 0.0
# =====================================================================

# 5. Main Simulation Loop
print("\n--- เริ่มการจำลอง (Gait Control v13: Robust Names) ---")
print(f"--- WARMING UP for {WARMUP_TIME} seconds... ---")
try:
    while True:
        sim_time += time_step
        basePos, baseOrn = p.getBasePositionAndOrientation(robotId)
        
        # 5.1: WARM-UP LOGIC
        if not is_walking and sim_time >= WARMUP_TIME:
            is_walking = True
            print("--- WARM-UP COMPLETE. Starting 2-DOF Compliant Trot. ---")
            state_timer = 0.0
        
        # 5.2: State Machine Logic
        if is_walking:
            state_timer += time_step
            if state_timer >= STEP_TIME:
                state_timer = 0.0 
                gait_state = (gait_state + 1) % 2 
        
        # 5.3: คำนวณพิกัดเป้าหมาย (Trot Logic + Pitch Balance)
        target_foot_positions_REL = {} 
        pitch_correction = 0.0
        
        if is_walking:
            # --- TROT LOGIC ---
            state_progress = state_timer / STEP_TIME
            if gait_state == 0: swing_legs = pair_1; stance_legs = pair_2
            else: swing_legs = pair_2; stance_legs = pair_1
            for leg_name in swing_legs:
                home_pos = home_foot_positions[leg_name]
                z_move = LIFT_HEIGHT * math.sin(state_progress * math.pi)
                x_move = STEP_LENGTH * (state_progress * 2 - 1.0)
                target_foot_positions_REL[leg_name] = [home_pos[0] + x_move, home_pos[1], home_pos[2] + z_move]
            for leg_name in stance_legs:
                home_pos = home_foot_positions[leg_name]
                x_shift = STEP_LENGTH * (1.0 - state_progress * 2)
                target_foot_positions_REL[leg_name] = [home_pos[0] + x_shift, home_pos[1], home_pos[2]]
            
            # --- BALANCE (Pitch-Only) ---
            euler_angles = p.getEulerFromQuaternion(baseOrn)
            actual_pitch = euler_angles[1]
            pitch_error = 0.0 - actual_pitch
            pitch_derivative = (pitch_error - prev_pitch_error) / time_step
            pitch_correction = (BALANCE_KP_PITCH * pitch_error) + (BALANCE_KD_PITCH * pitch_derivative)
            prev_pitch_error = pitch_error
        
        else:
            # --- WARM-UP STATE ---
            target_foot_positions_REL = home_foot_positions 

        # 5.4: วนลูป IK
        for foot_link_name, target_pos_REL in target_foot_positions_REL.items():
            
            if foot_link_name == 'FR_foot_link':
                current_moveable_joint_ids = FR_moveable_joint_ids
                current_all_joint_ids = FR_all_joint_ids
                current_foot_link_id = FR_foot_link_id
            elif foot_link_name == 'FL_foot_link':
                current_moveable_joint_ids = FL_moveable_joint_ids
                current_all_joint_ids = FL_all_joint_ids
                current_foot_link_id = FL_foot_link_id
            elif foot_link_name == 'RR_foot_link':
                current_moveable_joint_ids = RR_moveable_joint_ids
                current_all_joint_ids = RR_all_joint_ids
                current_foot_link_id = RR_foot_link_id
            elif foot_link_name == 'RL_foot_link':
                current_moveable_joint_ids = RL_moveable_joint_ids
                current_all_joint_ids = RL_all_joint_ids
                current_foot_link_id = RL_foot_link_id
            else:
                continue

            corrected_target_pos_REL = list(target_pos_REL) 
            corrected_target_pos_REL[0] += pitch_correction # X (Pitch)

            world_target_pos, _ = p.multiplyTransforms(basePos, baseOrn, corrected_target_pos_REL, [0, 0, 0, 1])
            
            # แก้ไข: ใช้ jointDamping แทน jointIndices
            joint_angles_all = p.calculateInverseKinematics(
                robotId, current_foot_link_id, world_target_pos, 
                jointDamping=[0.1] * num_joints,
                maxNumIterations=50
            )
            
            # ดึงมุมของ thigh และ shank ด้วย mapping
            target_angles_for_leg = [joint_angles_all[joint_id_to_ik_index[j]] for j in current_moveable_joint_ids]

            # (Dual-Gain Logic - ปรับให้นุ่มนวลขึ้น)
            if is_walking:
                p.setJointMotorControlArray(
                    robotId, current_moveable_joint_ids, p.POSITION_CONTROL,
                    targetPositions=target_angles_for_leg,
                    forces=[8] * len(current_moveable_joint_ids),  # ลดจาก 10 เป็น 8
                    positionGains=[0.3] * len(current_moveable_joint_ids), # ลดจาก 0.4 เป็น 0.3 (นุ่มนวลขึ้น)
                    velocityGains=[0.6] * len(current_moveable_joint_ids)  # ลดจาก 0.8 เป็น 0.6
                )
            else: # WARM-UP
                p.setJointMotorControlArray(
                    robotId, current_moveable_joint_ids, p.POSITION_CONTROL,
                    targetPositions=target_angles_for_leg,
                    forces=[10] * len(current_moveable_joint_ids),
                    positionGains=[0.6] * len(current_moveable_joint_ids), # ลดจาก 0.8 เป็น 0.6
                    velocityGains=[0.8] * len(current_moveable_joint_ids)  # ลดจาก 1.0 เป็น 0.8
                )
        
        # 5.5: Step Simulation
        p.stepSimulation()
        time.sleep(time_step) 

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")
except KeyError as e:
    print(f"\n--- ❌ CRITICAL: KeyError (ไม่พบชื่อ Joint/Link) ---")
    print(f"   ไม่พบ Key: {e}")
    print("   โปรดตรวจสอบไฟล์ .urdf ว่ามีช่องว่างพิเศษ หรือสะกดผิดหรือไม่")
except p.error as e:
    print(f"\n--- เกิดข้อผิดพลาดร้ายแรงกับ PyBullet! ---")
    print(e)
finally:
    if p.isConnected():
        p.disconnect()