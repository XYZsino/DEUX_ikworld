"""Executor-thread scene preview. HTTP serves only an immutable JPEG copy."""
from io import BytesIO
import mujoco
from PIL import Image


class SimWebView:
    def __init__(self,sim,gui):
        self.sim,self.gui=sim,gui
        self.camera=mujoco.MjvCamera()
        self.camera.lookat[:]=[-.3,0.,.05]
        self.camera.distance=2.;self.camera.azimuth=130.;self.camera.elevation=-25.

    def render(self):
        self.sim.scene_renderer.update_scene(self.sim.data,camera=self.camera)
        rgb=self.sim.scene_renderer.render()
        image=Image.fromarray(rgb);image.thumbnail((960,540))
        out=BytesIO();image.save(out,format='JPEG',quality=80)
        self.gui.set_frame(out.getvalue(),float(self.sim.data.time))
