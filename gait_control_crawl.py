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
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

# 3. รวบรวม Joints และ Links (เหมือนเดิม)
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
# 4. (NEW) ตั้งค่าพารามิเตอร์การเดิน (Gait Parameters)
# =====================================================================

# พิกัด "Home" ของปลายเท้า (เทียบกับ base_link)
base_x = 0.195 
base_y = 0.145 
z_height = -0.25 # ความสูงที่ยืน (ต่ำกว่าตัว 25cm)

# ท่า Home เริ่มต้น
home_foot_positions = {
    'FR_foot_link': [base_x, -base_y, z_height],
    'FL_foot_link': [base_x, base_y, z_height],
    'RR_foot_link': [-base_x, -base_y, z_height],
    'RL_foot_link': [-base_x, base_y, z_height],
}

# พารามิเตอร์การก้าว
STEP_LENGTH = 0.1  # ก้าวไกล 10 cm
LIFT_HEIGHT = 0.05 # ยกขาสูง 5 cm
STEP_TIME = 0.5    # แต่ละ state ใช้เวลา 0.5 วินาที

# ตัวแปรสำหรับ State Machine
gait_state = 0 # เริ่มที่ State 0
state_timer = 0.0
sim_time = 0.0
time_step = 1./240. # 240 Hz

# ลำดับขาที่จะก้าว
leg_order = ['FR_foot_link', 'RL_foot_link', 'FL_foot_link', 'RR_foot_link']


# =====================================================================
# 5. Main Simulation Loop
# =====================================================================
print("\n--- เริ่มการจำลอง (Gait Control v1: Crawl) ---")
try:
    while True:
        # อัปเดตเวลา
        state_timer += time_step
        sim_time += time_step

        # 5.1: (NEW) State Machine Logic
        if state_timer >= STEP_TIME:
            state_timer = 0.0 # รีเซ็ตไทม์เมอร์
            gait_state = (gait_state + 1) % 8 # ไป State ถัดไป (0-7)

        # 5.2: (NEW) คำนวณพิกัดเป้าหมายของเท้า
        target_foot_positions_REL = {} # พิกัดเป้าหมาย (Relative)
        
        # เปอร์เซ็นต์ความคืบหน้าของ state ปัจจุบัน (0.0 ถึง 1.0)
        state_progress = state_timer / STEP_TIME

        # ตรวจสอบว่าเป็น State "ก้าวขา" (0, 2, 4, 6) หรือ "เลื่อนตัว" (1, 3, 5, 7)
        is_swing_state = (gait_state % 2 == 0)

        if is_swing_state:
            # === State ก้าวขา ===
            swing_leg_index = gait_state // 2 # 0, 1, 2, 3
            swing_leg_name = leg_order[swing_leg_index]

            for leg_name, home_pos in home_foot_positions.items():
                if leg_name == swing_leg_name:
                    # ขานี้กำลังก้าว (Swing)
                    # ใช้ sin(pi * x) เพื่อให้ขา "ยกขึ้น" และ "วางลง" อย่างนุ่มนวล
                    z_move = LIFT_HEIGHT * math.sin(state_progress * math.pi)
                    # ขาจะก้าวจาก -STEP_LENGTH/2 ไป +STEP_LENGTH/2
                    x_move = STEP_LENGTH * (state_progress - 0.5)
                    
                    target_foot_positions_REL[leg_name] = [
                        home_pos[0] + x_move,
                        home_pos[1],
                        home_pos[2] + z_move
                    ]
                else:
                    # ขาที่เหลือ (Stance) จะอยู่ที่เดิม
                    target_foot_positions_REL[leg_name] = home_pos
        
        else:
            # === State เลื่อนตัว ===
            # ขาทุกข้างอยู่บนพื้น และ "ดัน" ตัวไปข้างหน้า
            # ขาจะเลื่อนจาก +STEP_LENGTH/2 กลับไปที่ -STEP_LENGTH/2
            x_shift = STEP_LENGTH * (0.5 - state_progress)
            
            for leg_name, home_pos in home_foot_positions.items():
                target_foot_positions_REL[leg_name] = [
                    home_pos[0] + x_shift,
                    home_pos[1],
                    home_pos[2]
                ]

        # 5.3: อ่านตำแหน่ง Base (เหมือนเดิม)
        basePos, baseOrn = p.getBasePositionAndOrientation(robotId)

        # 5.4: วนลูป IK สำหรับแต่ละขา (เหมือนเดิม)
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

            # แปลงพิกัด (เหมือนเดิม)
            world_target_pos, _ = p.multiplyTransforms(
                basePos, baseOrn,               
                target_pos_REL, [0, 0, 0, 1]  
            )

            # คำนวณ IK (เหมือนเดิม)
            joint_angles_for_leg = p.calculateInverseKinematics(
                bodyUniqueId=robotId,
                endEffectorLinkIndex=current_foot_link_id,
                targetPosition=world_target_pos, 
                jointDamping=[0.1] * len(current_leg_joint_ids),
                maxNumIterations=50 
            )

            # ส่งคำสั่ง (เหมือนเดิม)
            p.setJointMotorControlArray(
                bodyUniqueId=robotId,
                jointIndices=current_leg_joint_ids,
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_angles_for_leg[:len(current_leg_joint_ids)],
                forces=[10] * len(current_leg_joint_ids),
                positionGains=[0.3] * len(current_leg_joint_ids), 
                velocityGains=[0.5] * len(current_leg_joint_ids)
            )
        
        # 5.5: Step Simulation
        p.stepSimulation()
        time.sleep(time_step) # หน่วงเวลาตาม time_step

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")

p.disconnect()