"""Render the exact library ready snapshot using local CAD mesh convex hulls."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/deux-grid-matplotlib')
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
from build_right_grasp_grid import Builder, ROOT
from deux_task_core.model import origin


def stl_vertices(path):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    if len(data) == 84+50*count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        return np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    return np.array([[float(x) for x in line.split()[1:]] for line in data.decode().splitlines()
                     if line.strip().startswith('vertex ')])


def main():
    b = Builder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    transforms = b.robot.fk(b.ready)
    model = ET.parse(b.config['robot']['urdf_path']).getroot()
    meshes = []; missing = []
    for link in model.findall('link'):
        name = link.attrib['name']
        if name not in transforms: continue
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None: continue
            filename = mesh.attrib['filename'].replace('package://deux_description/', '')
            path = Path('/home/sino/git/DEUX/src/deux_description')/filename
            if not path.exists(): missing.append(str(path)); continue
            points = stl_vertices(path)*np.array(mesh.get('scale', '1 1 1').split(), float)
            hull = ConvexHull(points)
            faces = points[hull.simplices]
            t = transforms[name]@origin(visual.find('origin'))
            faces = faces@t[:3, :3].T+t[:3, 3]
            color = '#ec862f' if name.startswith('right') else '#6b96ae' if name.startswith('left') else '#a6afb5'
            meshes.append((faces, color))
    if missing: raise ValueError('missing CAD meshes: '+str(missing))
    fig = plt.figure(figsize=(14, 7), facecolor='white')
    ee = b.robot.ee_fk('right', b.ready)[:3, 3]
    for index, (elev, azim, title) in enumerate([(20, -48, 'Front-right view'), (90, -90, 'Top view')]):
        ax = fig.add_subplot(1, 2, index+1, projection='3d')
        for faces, color in meshes:
            ax.add_collection3d(Poly3DCollection(faces, facecolor=color, edgecolor='none', alpha=.95))
        ax.scatter(*ee, s=65, c='#d02244', depthshade=False)
        ax.text(*ee, '  Right grasp\n  (0.588, -0.143, 1.046) m', color='#a40d2f', fontsize=9)
        xx, yy = np.meshgrid([-.2, .9], [-.65, .65]); zz = np.full_like(xx, .97)
        ax.plot_surface(xx, yy, zz, color='#39b1b9', alpha=.12, shade=False)
        ax.set(xlim=(-.4, 1.), ylim=(-.7, .7), zlim=(0, 1.65), xlabel='base X (m)', ylabel='base Y (m)', zlabel='base Z (m)', title=title)
        ax.set_box_aspect((1.4, 1.4, 1.65)); ax.view_init(elev=elev, azim=azim)
    fig.suptitle('Grid trajectory initial ready pose | installation.yaml home_joints\nOrange: right arm / Blue: left arm / Cyan plane: grasp Z = 0.97 m', fontsize=13)
    fig.text(.5, .015, 'Measured-lift URDF + CAD convex envelopes. Offline model preview; robot has not moved.', ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .04, 1, .90))
    out = ROOT/'artifacts/right_grasp_grid/ready_pose.png'; fig.savefig(out, dpi=150)
    print(out)
    print('Right arm degrees:', np.degrees(b.ready[14:21]).round(2).tolist())


from pathlib import Path
if __name__ == '__main__': main()
