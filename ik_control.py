import pybullet as p
import pybullet_data
import time
import math

# 1. เชื่อมต่อกับ Physics Server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 2. ตั้งค่าสภาพแวดล้อม
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# 3. โหลดหุ่นยนต์
urdf_path = "urdf/my_robot.urdf"
startPos = [0, 0, 0.5] # เริ่มที่ 0.5m เหนือพื้น
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)
print(f"--- โหลด URDF สำเร็จ! Robot ID: {robotId} ---")

# =====================================================================
# 4. รวบรวม Joints และ Links (เหมือนเดิม)
# =====================================================================
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
# 5. (CHANGED) กำหนดพิกัดปลายเท้าเป้าหมาย (เทียบกับ Body Frame)
# =====================================================================
# พิกัดเหล่านี้เป็น [x, y, z] เทียบกับ *จุดศูนย์กลางของ base_link*
# ขาของเรายาว 0.145 + 0.145 = 0.29m
# เราจะสั่งให้หุ่นยืนโดยที่ปลายเท้าอยู่ต่ำกว่า body 0.25m (งอเข่าเล็กน้อย)
target_z_height = -0.25 

# พิกัด x, y ของปลายเท้า เทียบกับ base_link (อิงจากตำแหน่ง hip_joint)
base_x = 0.195 
base_y = 0.145 

target_foot_positions_REL = { # REL = Relative (เทียบกับ Body)
    'FR_foot_link': [base_x, -base_y, target_z_height],
    'FL_foot_link': [base_x, base_y, target_z_height],
    'RR_foot_link': [-base_x, -base_y, target_z_height],
    'RL_foot_link': [-base_x, base_y, target_z_height],
}

# =====================================================================
# 6. (CHANGED) Main Simulation Loop (ใช้ IK + PID ที่แข็งแรงขึ้น)
# =====================================================================
print("\n--- เริ่มการจำลอง (ใช้ IK v2) ---")
try:
    while True:
        # 6.1: (NEW) อ่านตำแหน่งปัจจุบันของ base_link
        basePos, baseOrn = p.getBasePositionAndOrientation(robotId)

        # 6.2: วนลูปสำหรับแต่ละขา
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

            # 6.3: (NEW) แปลงพิกัดเป้าหมาย (Relative) ไปเป็น (World)
            # เราต้องการพิกัด World Frame ที่จะส่งให้ IK
            world_target_pos, _ = p.multiplyTransforms(
                basePos, baseOrn,               # พิกัด World ของ Base
                target_pos_REL, [0, 0, 0, 1]  # พิกัด Local ของเท้า
            )

            # 6.4: คำนวณ IK
            joint_angles_for_leg = p.calculateInverseKinematics(
                bodyUniqueId=robotId,
                endEffectorLinkIndex=current_foot_link_id,
                targetPosition=world_target_pos, # ใช้พิกัด World ที่เพิ่งคำนวณ
                jointDamping=[0.1] * len(current_leg_joint_ids),
                maxNumIterations=50 # เพิ่มความเร็วในการคำนวณ
            )

            # 6.5: (CHANGED) ส่งคำสั่งมุม + Gain ที่แข็งแรงขึ้น
            p.setJointMotorControlArray(
                bodyUniqueId=robotId,
                jointIndices=current_leg_joint_ids,
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_angles_for_leg[:len(current_leg_joint_ids)],
                forces=[10] * len(current_leg_joint_ids),
                positionGains=[0.3] * len(current_leg_joint_ids), # <--- CHANGED: เพิ่ม P-Gain (0.05 -> 0.3)
                velocityGains=[0.5] * len(current_leg_joint_ids)  # <--- CHANGED: ลด D-Gain (1.0 -> 0.5)
            )
        
        # 6.6: ให้ฟิสิกส์ทำงาน 1 ก้าว
        p.stepSimulation()
        
        time.sleep(1./240.) 

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")

p.disconnect()