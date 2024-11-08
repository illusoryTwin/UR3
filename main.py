import mujoco
from mujoco import viewer
import time
import numpy as np

# model_path = "ur_description/urdf/universalUR3_motor1.urdf"
# model_path = "ur_description/urdf/universalUR3.urdf"
model_path = "mjmodel_without_inertia.xml"

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
print("len(data.qpos)", len(data.qpos))

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 25:
        step_start = time.time()
        # for i in range(-120, 0, 2):
        #     data.qpos = np.deg2rad(i)
        # data.qpos = 0 * len(data.qpos)
        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
