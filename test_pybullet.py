import pybullet as p
import pybullet_data
import time

# เริ่มการเชื่อมต่อกับ engine ฟิสิกส์
physicsClient = p.connect(p.GUI) 

print("PyBullet Installation Successful!")
print("PyBullet version:", p.getAPIVersion())

# ตั้งค่า search path สำหรับ URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# โหลดพื้นเข้ามาใน simulation
p.loadURDF("plane.urdf") 
p.setGravity(0, 0, -9.8)

# Loop เพื่อให้หน้าต่าง simulation ค้างไว้
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except p.error as e:
    # ผู้ใช้ปิดหน้าต่าง simulation
    print("Simulation window closed.")
finally:
    p.disconnect()