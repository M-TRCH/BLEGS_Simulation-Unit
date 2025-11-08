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
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)
print(f"--- โหลด URDF สำเร็จ! Robot ID: {robotId} ---")

# =====================================================================
# 4. รวบรวม Joints และ Links ที่ควบคุมได้ (สำหรับ IK)
# =====================================================================
joint_name_to_id = {}
link_name_to_id = {}
controllable_joints_ids = [] # เก็บเฉพาะ ID ของ Joint ที่หมุนได้

num_joints = p.getNumJoints(robotId)
print(f"--- ค้นหา Joints และ Links ---")
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    link_name = joint_info[12].decode('utf-8') # ชื่อ Link ที่เป็น child ของ Joint นี้

    joint_name_to_id[joint_name] = i
    link_name_to_id[link_name] = i # Link ID จะเป็น ID ของ Joint ที่เชื่อมกับมัน

    if joint_type == p.JOINT_REVOLUTE:
        controllable_joints_ids.append(i)
        print(f"  Found Revolute Joint: {joint_name} (ID: {i}) -> Link: {link_name}")

# กำหนด ID ของปลายเท้าแต่ละข้าง
# เราใช้ชื่อ Link ของ Foot เพื่อหา ID ของมัน
FR_foot_link_id = link_name_to_id['FR_foot_link']
FL_foot_link_id = link_name_to_id['FL_foot_link']
RR_foot_link_id = link_name_to_id['RR_foot_link']
RL_foot_link_id = link_name_to_id['RL_foot_link']

# รวบรวม Joint IDs ของแต่ละขา (ตามลำดับ Hip, Thigh, Shank)
# PyBullet IK ต้องการ Joint IDs ที่เกี่ยวข้องกับขาตามลำดับ
# ตรวจสอบชื่อ Joint ของคุณอีกครั้งให้แน่ใจว่าตรงกัน
FR_joint_ids = [joint_name_to_id['FR_hip_joint'], joint_name_to_id['FR_thigh_joint'], joint_name_to_id['FR_shank_joint']]
FL_joint_ids = [joint_name_to_id['FL_hip_joint'], joint_name_to_id['FL_thigh_joint'], joint_name_to_id['FL_shank_joint']]
RR_joint_ids = [joint_name_to_id['RR_hip_joint'], joint_name_to_id['RR_thigh_joint'], joint_name_to_id['RR_shank_joint']]
RL_joint_ids = [joint_name_to_id['RL_hip_joint'], joint_name_to_id['RL_thigh_joint'], joint_name_to_id['RL_shank_joint']]


# =====================================================================
# 5. กำหนดพิกัดปลายเท้าเป้าหมาย (Target Foot Positions)
# =====================================================================
# พิกัดเหล่านี้เป็น [x, y, z] เทียบกับจุดกำเนิดของโลก (World Frame)
# เราต้องการให้หุ่นยืนตรง โดยที่ปลายเท้าอยู่ที่ระดับความสูง -0.2 เมตร จากจุดเริ่มต้น
# (หุ่นเริ่มต้นที่ 0.5m, พื้นที่ 0m, ดังนั้นปลายเท้าจะอยู่เหนือพื้นเล็กน้อย)

# ลองให้หุ่นยืนในท่าที่ปลายเท้าแตะพื้นเล็กน้อย
# ตำแหน่ง x, y ประมาณจาก origin ของ hip_joint + ความยาวขา
# ปลายเท้าแต่ละข้างจะอยู่ใต้ hip_joint ประมาณ 0.195 + 0.145 + 0.145 = 0.485m ในแนวตั้ง
# เนื่องจากหุ่นเริ่มต้นที่ 0.5m ดังนั้นปลายเท้าจะอยู่เหนือพื้น 0.5 - 0.485 = 0.015m

# พิกัด x, y ของปลายเท้าเมื่อหุ่นยืนตรง
# (0.195 คือครึ่งความยาว body, 0.145 คือครึ่งความกว้าง body)
base_x = 0.195 + 0.02 # สมมุติว่ายื่นไปข้างหน้าเล็กน้อยจากบอดี้
base_y = 0.145 + 0.02 # สมมุติว่ายื่นออกด้านข้างเล็กน้อยจากบอดี้
ground_z = -0.015 # พิกัด Z ที่ปลายเท้าควรอยู่ (ติดพื้นเล็กน้อย)

target_foot_positions = {
    'FR_foot_link': [base_x, -base_y, ground_z],
    'FL_foot_link': [base_x, base_y, ground_z],
    'RR_foot_link': [-base_x, -base_y, ground_z],
    'RL_foot_link': [-base_x, base_y, ground_z],
}

# =====================================================================
# 6. Main Simulation Loop (ใช้ IK เพื่อสั่งงาน)
# =====================================================================
print("\n--- เริ่มการจำลอง (ใช้ IK) (กด Ctrl+C ใน Terminal เพื่อหยุด) ---")
try:
    # Set PID gains (สำคัญมากสำหรับ IK control)
    # PyBullet จะใช้ PID นี้เพื่อพยายามให้ Joint ไปถึง target_joint_angles
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=controllable_joints_ids,
        controlMode=p.POSITION_CONTROL,
        forces=[10]*len(controllable_joints_ids),  # Max force for each joint
        positionGains=[0.05]*len(controllable_joints_ids), # P-gain
        velocityGains=[1.0]*len(controllable_joints_ids) # D-gain
    )

    while True:
        # 6.1: คำนวณ IK สำหรับแต่ละขา
        for foot_link_name, target_pos in target_foot_positions.items():
            
            # เลือก Joint IDs ที่เกี่ยวข้องกับขาปัจจุบัน
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
                continue # Skip if not a foot link we defined

            # คำนวณ IK: หาชุดมุม Joint ที่ทำให้ปลายเท้าไปอยู่ที่ target_pos
            # useNullSpace=0 (False) ทำให้ PyBullet หา IK โดยไม่ต้องคำนึงถึง Null Space (ง่ายกว่า)
            # เราใช้ jointLimits เพื่อช่วยแก้ปัญหา IK (จาก URDF)
            # solver=0 คือ default DLS method
            joint_angles_for_leg = p.calculateInverseKinematics(
                bodyUniqueId=robotId,
                endEffectorLinkIndex=current_foot_link_id,
                targetPosition=target_pos,
                # ไม่จำเป็นต้องกำหนด targetOrientation สำหรับหุ่นยนต์ 4 ขาเบื้องต้น
                # upperLimits, lowerLimits, jointRanges, restPoses: ใช้จาก URDF ที่เรากำหนดไปแล้ว
                jointDamping=[0.1, 0.1, 0.1] * 4, # damping สำหรับ joint ที่เกี่ยวข้อง
                #solver=p.IK_DLS # หรือ p.IK_SDLS
            )

            # 6.2: ส่งคำสั่งมุมที่ได้จาก IK ไปยัง Joint ของขา
            # PyBullet จะปรับ Joint Angles ที่ได้จาก IK ให้ตรงกับ Joint ID ของเรา
            p.setJointMotorControlArray(
                bodyUniqueId=robotId,
                jointIndices=current_leg_joint_ids, # เฉพาะ Joint ของขาปัจจุบัน
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_angles_for_leg[:len(current_leg_joint_ids)], # ส่งเฉพาะ Joint ที่เกี่ยวข้อง
                forces=[10]*len(current_leg_joint_ids),
                positionGains=[0.05]*len(current_leg_joint_ids),
                velocityGains=[1.0]*len(current_leg_joint_ids)
            )
        
        # 6.3: ให้ฟิสิกส์ทำงาน 1 ก้าว
        p.stepSimulation()
        
        # 6.4: หน่วงเวลา (เพื่อให้เรามองเห็น)
        time.sleep(1./240.) 

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")

p.disconnect()