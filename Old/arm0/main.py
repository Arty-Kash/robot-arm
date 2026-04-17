import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import time
import numpy as np

def main():
    # 1. ビジュアライザの起動
    vis = meshcat.Visualizer()

    # 2. クリック可能なURLを表示（末尾に /static/ を付加）
    raw_url = vis.url()
    # Codespacesのプロキシを通る際、末尾の / が重複しないよう調整
    if not raw_url.endswith('/'):
        raw_url += '/'
    clickable_url = raw_url + "static/"

    print("-" * 60)
    print(f"Meshcat Server is running!")
    print(f"Click here to open: {clickable_url}")
    print("-" * 60)

    # 3. パーツの定義（形状と色）
    # 土台 (Base)
    base_geo = g.Box([0.2, 0.2, 0.1])
    base_mat = g.MeshPhongMaterial(color=0x333333) # 濃いグレー
    
    # 第1リンク (下腕: Lower Arm)
    link1_geo = g.Box([0.05, 0.05, 0.3])
    link1_mat = g.MeshPhongMaterial(color=0x00ff00) # 緑
    
    # 第2リンク (上腕: Upper Arm)
    link2_geo = g.Box([0.04, 0.04, 0.25])
    link2_mat = g.MeshPhongMaterial(color=0x0000ff) # 青

    # 4. パーツの配置（親子関係の構築）
    # 土台を配置
    vis["robot/base"].set_object(base_geo, base_mat)
    
    # 第1リンクを配置（土台の子にする）
    # パスを "robot/base/link1" とすることで親子関係になります
    vis["robot/base/link1"].set_object(link1_geo, link1_mat)
    # 子の初期位置を調整（土台の上に載せる。高さは土台の半分+リンクの半分 = 0.05 + 0.15）
    vis["robot/base/link1"].set_transform(tf.translation_matrix([0, 0, 0.2]))

    # 第2リンクを配置（第1リンクの子にする）
    # パスをさらに深くします
    vis["robot/base/link1/link2"].set_object(link2_geo, link2_mat)
    # 第1リンクの先端に配置（第1リンクの長さ 0.3 の半分 + 第2リンクの半分 0.125 = 0.275）
    vis["robot/base/link1/link2"].set_transform(tf.translation_matrix([0, 0, 0.275]))

    # 5. アニメーション（親子関係の確認）
    print("Moving the shoulder (base)... Notice how the entire arm follows.")
    
    angle = 0
    try:
        while True:
            angle += 0.05
            
            # 土台（親）を回転させる
            # これを動かすだけで、子であるlink1も孫であるlink2も一緒に回転します
            vis["robot/base"].set_transform(tf.rotation_matrix(angle, [0, 0, 1]))
            
            # 第2リンク（孫）だけを独自に回転させる（第1リンクに対する相対的な動き）
            vis["robot/base/link1/link2"].set_transform(
                tf.concatenate_matrices(
                    tf.translation_matrix([0, 0, 0.275]), # 位置は固定
                    tf.rotation_matrix(np.sin(angle)*1.5, [0, 1, 0]) # Y軸周りにユラユラ
                )
            )
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()