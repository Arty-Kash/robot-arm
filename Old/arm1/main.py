import pybullet as p
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import time
import numpy as np
import os

def main():
    # 1. PyBulletの設定
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)

    # 2. URDFの読み込み
    urdf_path = "simple_arm.urdf"
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} not found!")
        return
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)

    # 3. Meshcatの設定
    vis = meshcat.Visualizer()
    raw_url = vis.url()
    if not raw_url.endswith('/'): raw_url += '/'
    print("-" * 60)
    print(f"Meshcat Server started.")
    print(f"Access URL: {raw_url}static/")
    print("-" * 60)

    # 4. Meshcat上の形状生成とオフセット情報の保持
    visual_offsets = {} # パーツごとの位置ズレ情報を保存する辞書
    visual_data = p.getVisualShapeData(robot_id)

    for data in visual_data:
        link_id = data[1]
        geom_type = data[2]
        size = data[3]
        v_pos = data[5] # URDFのvisual origin位置
        v_quat = data[6] # URDFのvisual origin回転
        rgba = data[7]
        
        path = f"robot/link_{link_id}"
        
        # 形状の作成と向きの補正
        if geom_type == p.GEOM_BOX:
            obj = g.Box(size)
            # Boxの場合は補正なしでOK
            geom_transform = tf.identity_matrix()
        elif geom_type == p.GEOM_CYLINDER:
            obj = g.Cylinder(size[0], size[1])
            # MeshcatのCylinder(Y向き)をPyBullet(Z向き)に合わせるため90度回転
            geom_transform = tf.rotation_matrix(np.pi/2, [1, 0, 0])
        else:
            continue
            
        vis[path].set_object(obj, g.MeshPhongMaterial(color=int(rgba[0]*255)<<16 | int(rgba[1]*255)<<8 | int(rgba[2]*255)))

        # 「パーツ自体のズレ」を計算して保存
        offset_matrix = tf.translation_matrix(v_pos) @ tf.quaternion_matrix([v_quat[3], v_quat[0], v_quat[1], v_quat[2]])
        visual_offsets[link_id] = offset_matrix @ geom_transform

    # 5. シミュレーションループ
    print("Running simulation... Connect to Meshcat to see the arm.")
    t = 0
    try:
        while True:
            # 物理演算側の制御（サイン波でアームを振る）
            p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=np.sin(t))
            p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=np.cos(t) * 1.0)
            
            p.stepSimulation()
            
            # Meshcat側の各パーツ位置を更新
            for i in range(-1, num_joints):
                if i == -1:
                    pos, quat = p.getBasePositionAndOrientation(robot_id)
                else:
                    state = p.getLinkState(robot_id, i)
                    pos, quat = state[4], state[5] # 世界座標での位置と回転
                
                # 1. PyBulletから来た世界座標の行列
                world_m = tf.translation_matrix(pos) @ tf.quaternion_matrix([quat[3], quat[0], quat[1], quat[2]])
                
                # 2. そのリンク固有のオフセット（向き補正など）を掛け合わせる
                if i in visual_offsets:
                    final_m = world_m @ visual_offsets[i]
                    vis[f"robot/link_{i}"].set_transform(final_m)
            
            t += 0.05
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        p.disconnect()
        print("\nStopped.")

if __name__ == "__main__":
    main()