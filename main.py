import pybullet as p
import meshcat
import meshcat.transformations as tf
import numpy as np
import asyncio
import json
import re
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from sse_starlette.sse import EventSourceResponse

app = FastAPI()

# 1. MeshcatとPyBulletの初期化
vis = meshcat.Visualizer()
p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)

# Meshcatが実際に使用しているポート番号を抽出 (例: "7001")
m = re.search(r":(\d+)/", vis.url())
meshcat_port = m.group(1) if m else "7000"

# ロボットの読み込み
robot_id = p.loadURDF("simple_arm.urdf", useFixedBase=True)
num_joints = p.getNumJoints(robot_id)

visual_offsets = {}
current_step = 0
current_reward = 0.0

def setup_meshcat_robot():
    visual_data = p.getVisualShapeData(robot_id)
    for data in visual_data:
        link_id, geom_type, size, v_pos, v_quat, rgba = data[1], data[2], data[3], data[5], data[6], data[7]
        path = f"robot/link_{link_id}"
        if geom_type == p.GEOM_BOX:
            obj = meshcat.geometry.Box(size); geom_transform = tf.identity_matrix()
        elif geom_type == p.GEOM_CYLINDER:
            obj = meshcat.geometry.Cylinder(size[0], size[1]); geom_transform = tf.rotation_matrix(np.pi/2, [1, 0, 0])
        else: continue
        vis[path].set_object(obj, meshcat.geometry.MeshPhongMaterial(color=int(rgba[0]*255)<<16 | int(rgba[1]*255)<<8 | int(rgba[2]*255)))
        visual_offsets[link_id] = tf.translation_matrix(v_pos) @ tf.quaternion_matrix([v_quat[3], v_quat[0], v_quat[1], v_quat[2]]) @ geom_transform

async def simulation_loop():
    global current_step, current_reward
    setup_meshcat_robot()
    t = 0
    while True:
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=np.sin(t))
        p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=np.cos(t))
        p.stepSimulation()
        for i in range(-1, num_joints):
            pos, quat = (p.getBasePositionAndOrientation(robot_id)) if i == -1 else (p.getLinkState(robot_id, i)[4:6])
            m = tf.translation_matrix(pos) @ tf.quaternion_matrix([quat[3], quat[0], quat[1], quat[2]])
            if i in visual_offsets: vis[f"robot/link_{i}"].set_transform(m @ visual_offsets[i])
        current_step += 1
        current_reward = round(np.sin(t) * 10, 2)
        t += 0.05
        await asyncio.sleep(0.05)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(simulation_loop())

@app.get("/")
async def get_index():
    with open("index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

@app.get("/stream")
async def message_stream(request: Request):
    async def event_generator():
        while True:
            if await request.is_disconnected(): break
            # ポート番号も含めて送信
            yield json.dumps({
                "step": current_step, 
                "reward": current_reward, 
                "port": meshcat_port,
                "status": "Running"
            })
            await asyncio.sleep(0.5)
    return EventSourceResponse(event_generator())

if __name__ == "__main__":
    import uvicorn
    print(f"\n--- Meshcat is using port: {meshcat_port} ---")
    print(f"--- Access the Web App on port 8000 ---\n")
    uvicorn.run(app, host="0.0.0.0", port=8000)