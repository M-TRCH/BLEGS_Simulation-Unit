import pybullet as p
import pybullet_data
import time
import math # เพิ่ม math เข้ามาเพื่อใช้ค่า PI

# 1. เชื่อมต่อกับ Physics Server
physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# 2. ตั้งค่าสภาพแวดล้อม
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# 3. โหลดหุ่นยนต์
urdf_path = "urdf/my_robot.urdf"
startPos = [0, 0, 0.5] # เริ่มที่ความสูง 0.5 เมตร
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)
print(f"--- โหลด URDF สำเร็จ! Robot ID: {robotId} ---")

# =====================================================================
# 4. รวบรวม Joints ที่ควบคุมได้
# =====================================================================
# เราจะเก็บ ID ของ Joint ที่เป็น 'revolute' (หมุนได้) เท่านั้น
joint_name_to_id = {}
num_joints = p.getNumJoints(robotId)

print(f"--- ค้นหา Joints ที่ควบคุมได้ (Revolute) ---")
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    
    # ตรวจสอบว่าเป็น Joint ที่หมุนได้ (ไม่ใช่ 'fixed')
    if joint_type == p.JOINT_REVOLUTE:
        joint_name_to_id[joint_name] = i
        print(f"  พบ Joint: {joint_name} (ID: {i})")

# รายชื่อ Joint 12 อันที่เราจะควบคุม
controllable_joints = list(joint_name_to_id.keys())
print(f"--- จำนวน Joint ที่ควบคุมได้: {len(controllable_joints)} ---")

# =====================================================================
# 5. กำหนดท่าเป้าหมาย (Target Pose)
# =====================================================================
# เราจะกำหนด "ท่าเริ่มต้น" หรือ "ท่ายืน" (Home Pose)
# ค่ามุมเป็นเรเดียน (radian)

# คำนวณค่ามุม (ตัวอย่าง)
# 0 องศา = 0
# 30 องศา = math.pi / 6 ≈ 0.52
# -60 องศา = -math.pi / 3 ≈ -1.04

# นี่คือท่า "ย่อตัว" เล็กน้อย (Crouch)
target_pose = {
    # ขาหน้าขวา (FR)
    'FR_hip_joint': 0,         # 0 องศา (ตรง)
    'FR_thigh_joint': math.pi / 6,   # 30 องศา (งอเข้า)
    'FR_shank_joint': -math.pi / 3,  # -60 องศา (งอเข่า)
    # ขาหน้าซ้าย (FL)
    'FL_hip_joint': 0,
    'FL_thigh_joint': math.pi / 6,
    'FL_shank_joint': -math.pi / 3,
    # ขาหลังขวา (RR)
    'RR_hip_joint': 0,
    'RR_thigh_joint': math.pi / 6,
    'RR_shank_joint': -math.pi / 3,
    # ขาหลังซ้าย (RL)
    'RL_hip_joint': 0,
    'RL_thigh_joint': math.pi / 6,
    'RL_shank_joint': -math.pi / 3,
}

# =====================================================================
# 6. Main Simulation Loop (ส่งคำสั่งควบคุม)
# =====================================================================
print("\n--- เริ่มการจำลอง (กด Ctrl+C ใน Terminal เพื่อหยุด) ---")
try:
    while True:
        # 6.1: ส่งคำสั่งไปยัง Joint ทั้ง 12 อันในทุกๆ Step
        for joint_name, target_angle in target_pose.items():
            joint_id = joint_name_to_id[joint_name]
            
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL, # <-- นี่คือหัวใจหลัก!
                targetPosition=target_angle,    # <-- องศาเป้าหมาย
                force=10,                       # <-- แรงบิดสูงสุด (จาก URDF)
                maxVelocity=10                  # <-- ความเร็วสูงสุด (จาก URDF)
            )
        
        # 6.2: ให้ฟิสิกส์ทำงาน 1 ก้าว
        p.stepSimulation()
        
        # 6.3: หน่วงเวลา (เพื่อให้เรามองเห็น)
        time.sleep(1./240.) 

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")

p.disconnect()