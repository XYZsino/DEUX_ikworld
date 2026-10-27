"""ZED 2i simulated registered RGB-D; provisional mount, no object truth input."""
import numpy as np
import mujoco


class HeadRgbd:
    def __init__(self, sim, width=None, height=None):
        self.sim = sim
        width=sim.zed_camera_config['sim_width'] if width is None else width
        height=sim.zed_camera_config['sim_height'] if height is None else height
        self.width, self.height = width, height
        self.frame_id = 'zed_left_camera_frame_optical'
        self.camera = sim.model.camera('zed2i_left').id
        self.renderer = mujoco.Renderer(sim.model, height, width)
        focal = .5*height/np.tan(np.deg2rad(sim.model.cam_fovy[self.camera])/2)
        self.k = np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])

    def capture(self):
        self.renderer.update_scene(self.sim.data, camera=self.camera)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        try: depth = self.renderer.render().copy()
        finally: self.renderer.disable_depth_rendering()
        depth[(~np.isfinite(depth)) | (depth <= 0) | (depth > 3.)] = np.nan
        optical = np.eye(4)
        optical[:3,:3] = self.sim.data.cam_xmat[self.camera].reshape(3,3) @ np.diag([1.,-1.,-1.])
        optical[:3,3] = self.sim.data.cam_xpos[self.camera]
        return rgb, depth.astype(np.float32), self.k.copy(), optical

    def close(self): self.renderer.close()
