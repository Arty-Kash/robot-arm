import pybullet as p
import pybullet_data
import meshcat
import meshcat.geometry as g
import time
import os
import numpy as np

# 1. MeshCat（ブラウザ表示）の準備
vis = meshcat.Visualizer()
print(f"★ブラウザでこちらを開いてください: {vis.url()}")

# MeshCat側に「ロボットの先端」を示す目印を表示
vis["end_effector"].set_object(g.Sphere(0.05), g.MeshLambertMaterial(color=0xff0000))

# 2. PyBullet（物理計算エンジン）の準備
# GUIで見たい場合は p.connect(p.GUI) に書き換えてください
p.connect(p.DIRECT) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 3. URDFファイルの読み込み
# main.py と同じフォルダにある arm.urdf を読み込む
urdf_path = os.path.join(os.path.dirname(__file__), "arm.urdf")

if not os.path.exists(urdf_path):
    print(f"エラー: {urdf_path} が見つかりません。")
    exit()

robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# 4. メインループ
print("シミュレーションを開始します... (Ctrl+Cで終了)")
try:
    t = 0
    while True:
        t += 0.02
        
        # 各関節の目標角度を計算 (サイン波でゆらゆら)
        # Joint 0: 旋回, Joint 1: 肩, Joint 2: 肘
        target_angles = [
            1.5 * np.sin(t),     # Joint 0
            0.8 * np.sin(t*0.5), # Joint 1
            0.8 * np.sin(t*1.2)  # Joint 2
        ]
        
        # PyBullet内のロボットに目標角度を指令
        for i in range(len(target_angles)):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angles[i]
            )
        
        p.stepSimulation()

        # 先端（link3 = インデックス2）の座標を取得してMeshCat側に反映
        # getLinkState(robot_id, 2) は link3 の情報を返す
        link_info = p.getLinkState(robot_id, 2)
        pos = link_info[0] # 世界座標系での位置 [x, y, z]
        
        # MeshCatの赤い球体を移動させる
        vis["end_effector"].set_transform(meshcat.transformations.translation_matrix(pos))
        
        # ターミナルにも座標を表示（動作確認用）
        # print(f"先端座標: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

        time.sleep(0.02)
except KeyboardInterrupt:
    p.disconnect()
    print("\n終了します")