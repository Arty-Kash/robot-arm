import pybullet as p
import pybullet_data
import meshcat
import time
import os

# 1. MeshCat（ブラウザ表示）の準備
vis = meshcat.Visualizer()
print(f"★ブラウザでこちらを開いてください: {vis.url()}")

# 2. PyBullet（物理計算エンジン）の準備
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 3. URDFファイルの読み込み
# 同じフォルダにある arm.urdf を読み込みます
urdf_path = os.path.join(os.path.dirname(__file__), "arm.urdf")
robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# 4. メインループ（自動で腕をゆらゆら動かします）
print("シミュレーションを開始します...")
try:
    t = 0
    while True:
        t += 0.02
        
        # 各関節の目標角度を計算（4自由度のうち3つを使用）
        angles = [
            1.5 * (t % 6.28 - 3.14) / 3.14, # Joint 0: 旋回
            0.5 * (t % 3.14 - 1.57),        # Joint 1: 肩
            0.8 * (t % 3.14 - 1.57)         # Joint 2: 肘
        ]
        
        # PyBullet内の仮想ロボットを動かす
        for i in range(len(angles)):
            p.resetJointState(robot_id, i, angles[i])
        
        # MeshCat側の見た目も更新（今回は簡易的に毎フレーム位置を同期するイメージ）
        # ※本来はMeshCatにURDFを直接読み込ませるのがベストですが、
        #   まずは「計算して描画する」流れを体験するため、簡単な球体などで先端を表示することも可能です。
        #   ここではPyBulletが動いていることをターミナルでも確認できるよう座標を出力します。
        
        pos, _ = p.getLinkState(robot_id, 3)[:2] # 先端の座標を取得
        # print(f"先端座標: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

        p.stepSimulation()
        time.sleep(0.05)
except KeyboardInterrupt:
    p.disconnect()
    print("終了します")