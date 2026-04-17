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

    # 2. 床の作成
    floor_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 0.01])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_col_id, basePosition=[0, 0, -0.01])

    # 3. ボールを載せる「台（ティー）」の作成
    tee_height = 0.285 # 台の高さ
    tee_pos = [0.15, 0.0, tee_height/2]
    tee_col_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=tee_height)
    # 質量0（baseMass=0）にすることで、叩かれても動かない固定された台になります
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tee_col_id, basePosition=tee_pos)

    # 4. ロボット(URDF)の読み込み
    urdf_path = "simple_arm.urdf"
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)

    # 5. ボールの作成（台の上に載せる）
    ball_radius = 0.04
    ball_mass = 0.1
    col_ball_id = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
    # 台の高さ + ボールの半径 の位置に配置
    ball_initial_pos = [0.15, 0.0, tee_height + ball_radius] 
    ball_body_id = p.createMultiBody(baseMass=ball_mass, baseCollisionShapeIndex=col_ball_id, basePosition=ball_initial_pos)

    # 6. Meshcatの設定
    vis = meshcat.Visualizer()
    raw_url = vis.url()
    if not raw_url.endswith('/'): raw_url += '/'
    print("-" * 60)
    print(f"Meshcat URL: {raw_url}static/")
    print("-" * 60)

    # 7. Meshcat上の表示設定
    vis["floor"].set_object(g.Box([2, 2, 0.01]), g.MeshPhongMaterial(color=0x888888))
    vis["floor"].set_transform(tf.translation_matrix([0, 0, -0.01]))
    
    # 台（ティー）の表示
    vis["tee"].set_object(g.Cylinder(tee_height, 0.01), g.MeshPhongMaterial(color=0xffff00))
    vis["tee"].set_transform(tf.translation_matrix(tee_pos) @ tf.rotation_matrix(np.pi/2, [1,0,0]))
    
    # ボールの表示
    vis["ball"].set_object(g.Sphere(ball_radius), g.MeshPhongMaterial(color=0xff0000))

    # ロボットの表示 (以前と同じ)
    visual_offsets = {}
    visual_data = p.getVisualShapeData(robot_id)
    for data in visual_data:
        link_id, geom_type, size, v_pos, v_quat, rgba = data[1], data[2], data[3], data[5], data[6], data[7]
        path = f"robot/link_{link_id}"
        if geom_type == p.GEOM_BOX: obj = g.Box(size); geom_transform = tf.identity_matrix()
        elif geom_type == p.GEOM_CYLINDER: obj = g.Cylinder(size[0], size[1]); geom_transform = tf.rotation_matrix(np.pi/2, [1, 0, 0])
        else: continue
        vis[path].set_object(obj, g.MeshPhongMaterial(color=int(rgba[0]*255)<<16 | int(rgba[1]*255)<<8 | int(rgba[2]*255)))
        visual_offsets[link_id] = tf.translation_matrix(v_pos) @ tf.quaternion_matrix([v_quat[3], v_quat[0], v_quat[1], v_quat[2]]) @ geom_transform

    # 8. メインループ
    t = 0
    try:
        while True:
            # スイング
            target_pos1 = np.sin(t * 3.0) * 1.5 
            target_pos2 = 1.57 # 水平
            p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=target_pos1, force=30)
            p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=target_pos2, force=30)
            
            p.stepSimulation()
            
            # 同期
            for i in range(-1, num_joints):
                if i == -1: pos, quat = p.getBasePositionAndOrientation(robot_id)
                else: state = p.getLinkState(robot_id, i); pos, quat = state[4], state[5]
                if i in visual_offsets:
                    vis[f"robot/link_{i}"].set_transform(tf.translation_matrix(pos) @ tf.quaternion_matrix([quat[3], quat[0], quat[1], quat[2]]) @ visual_offsets[i])
            
            b_pos, b_quat = p.getBasePositionAndOrientation(ball_body_id)
            vis["ball"].set_transform(tf.translation_matrix(b_pos) @ tf.quaternion_matrix([b_quat[3], b_quat[0], b_quat[1], b_quat[2]]))
            
            # ボールが飛ばされたら3秒後にリセット
            if np.linalg.norm(np.array(b_pos) - np.array(ball_initial_pos)) > 0.1:
                if b_pos[2] < 0 or np.linalg.norm(b_pos) > 2.0:
                    time.sleep(1) # 飛んでいく余韻
                    p.resetBasePositionAndOrientation(ball_body_id, ball_initial_pos, [0,0,0,1])
                    p.resetBaseVelocity(ball_body_id, [0,0,0], [0,0,0])

            t += 0.05
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        p.disconnect()
        print("\nStopped.")

if __name__ == "__main__":
    main()