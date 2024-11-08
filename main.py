# import mujoco
# from mujoco import viewer
# import time

# print("Im ok")
# model_path = "ur_description/urdf/universalUR3.urdf"

# model = mujoco.MjModel.from_xml_path(model_path)
# data = mujoco.MjData(model)

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < 30:
#         step_start = time.time()
#         mujoco.mj_step(model, data)

#         viewer.sync()

#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)


# ====================
# START HERE 


import mujoco
from mujoco import viewer
import time
import numpy as np

# model_path = "ur_description/urdf/universalUR3.urdf"
model_path = "ur_description/urdf/universalUR3_motor4.urdf"


model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
print("len(data.qpos)", len(data.qpos))

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 55:
        step_start = time.time()

        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)







# import mujoco
# from mujoco import viewer
# import time
# import numpy as np

# model_path = "ur_description/urdf/universalUR3.urdf"

# model = mujoco.MjModel.from_xml_path(model_path)
# data = mujoco.MjData(model)
# print("len(data.qpos)", len(data.qpos))
# k = 10

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < 55:
#         step_start = time.time()
        
#         # for i in range(-60, 60):
#         #     data.qpos[3] = i
#         data.qpos = 0
#         # data.qpos[0] = 90
#         data.qpos[0] = k
#         k += 0.02
#         # k += 0.3
#         # data.qpos = 0 * len(data.qpos)
#         mujoco.mj_step(model, data)

#         viewer.sync()

#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)









# import mujoco
# from mujoco import viewer
# import time
# import numpy as np

# model_path = "ur_description/urdf/universalUR3.urdf"

# model = mujoco.MjModel.from_xml_path(model_path)
# data = mujoco.MjData(model)
# print("len(data.qpos)", len(data.qpos))

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < 55:
#         step_start = time.time()
#         for i in range(-20, 70, 2):
#             data.qpos[0] = np.deg2rad(-120)
#             data.qpos[1:5] = 0 * len(data.qpos[1:5])
#             data.qpos[5] = np.deg2rad(i)
#             data.qpos[6:] = 0 * len(data.qpos[6:])

#             # data.qpos[0] = np.deg2rad(-70)
#             # data.qpos[1:4] = 0 * len(data.qpos[1:4])
#             # data.qpos[4] = np.deg2rad(i)
#             # data.qpos[5:] = 0 * len(data.qpos[5:])

#             mujoco.mj_step(model, data)

#             viewer.sync()

#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
