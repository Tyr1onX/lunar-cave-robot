"""Render the synthetic blue-block scene used by tzb_sim.

Run with Blender, for example:
    blender -b --python tools/render_blue_block.py -- \
        config/blue_block_scene.png
"""

import math
import os
import sys

import bpy
from mathutils import Matrix, Vector


def _output_path() -> str:
    argv = sys.argv
    if '--' in argv:
        args = argv[argv.index('--') + 1:]
        if args:
            return os.path.abspath(args[0])
    return os.path.abspath('config/blue_block_scene.png')


def _look_at(obj, target):
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def main() -> None:
    out = _output_path()
    os.makedirs(os.path.dirname(out), exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    # Camera pose matches tzb_sim/blue_block_scene.py: at (0, 0, 0.60) in the
    # robot base frame, optical +X forward, tilted 42 degrees down.
    camera_pos = Vector((0.0, 0.0, 0.60))
    pitch = math.radians(52.13)
    yaw = math.radians(26.57)
    x_axis = Vector((0.0, -1.0, 0.0))
    z_axis = Vector((math.cos(pitch), 0.0, -math.sin(pitch)))
    y_axis = z_axis.cross(x_axis)
    rot_z = Matrix.Rotation(yaw, 4, 'Z')
    r_opt = rot_z.to_3x3() @ Matrix((
        (x_axis.x, y_axis.x, z_axis.x),
        (x_axis.y, y_axis.y, z_axis.y),
        (x_axis.z, y_axis.z, z_axis.z),
    ))
    r_blender = r_opt @ Matrix.Diagonal((1.0, -1.0, -1.0))

    cam_data = bpy.data.cameras.new('Camera')
    cam_data.lens = 800.0 * 36.0 / 640.0
    cam_data.sensor_width = 36.0
    cam_data.sensor_fit = 'HORIZONTAL'
    cam_data.clip_start = 0.01
    cam = bpy.data.objects.new('Camera', cam_data)
    scene.collection.objects.link(cam)
    cam.location = camera_pos
    cam.rotation_mode = 'QUATERNION'
    cam.rotation_quaternion = r_blender.to_quaternion()
    scene.camera = cam

    # Table.
    bpy.ops.mesh.primitive_plane_add(size=4.0, location=(0.4, 0.1, 0.0))
    table = bpy.context.object
    table.name = 'Table'
    table_mat = bpy.data.materials.new('TableMat')
    table_mat.use_nodes = True
    bsdf = table_mat.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Base Color'].default_value = (0.16, 0.15, 0.13, 1.0)
    bsdf.inputs['Roughness'].default_value = 0.75
    table.data.materials.append(table_mat)

    # Blue block, 50 x 50 x 50 mm, centered at (0.40, 0.12, 0.025).
    size = 0.05
    center = (0.40, 0.20, size / 2.0)
    bpy.ops.mesh.primitive_cube_add(size=size, location=center)
    cube = bpy.context.object
    cube.name = 'BlueBlock'
    bevel = cube.modifiers.new('Bevel', 'BEVEL')
    bevel.width = 0.0012
    bevel.segments = 3
    bpy.ops.object.modifier_apply(modifier=bevel.name)

    cube_mat = bpy.data.materials.new('BlueBlockMat')
    cube_mat.use_nodes = True
    bsdf = cube_mat.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Base Color'].default_value = (0.004, 0.045, 0.55, 1.0)
    bsdf.inputs['Roughness'].default_value = 0.38
    bsdf.inputs['Metallic'].default_value = 0.0
    cube.data.materials.append(cube_mat)

    # Lights.
    key_data = bpy.data.lights.new('Key', 'AREA')
    key_data.energy = 45.0
    key_data.shape = 'DISK'
    key_data.size = 0.7
    key = bpy.data.objects.new('Key', key_data)
    scene.collection.objects.link(key)
    key.location = (0.55, -0.25, 0.95)
    _look_at(key, center)

    fill_data = bpy.data.lights.new('Fill', 'AREA')
    fill_data.energy = 14.0
    fill_data.size = 0.9
    fill = bpy.data.objects.new('Fill', fill_data)
    scene.collection.objects.link(fill)
    fill.location = (0.0, 0.65, 0.55)
    _look_at(fill, center)

    world = bpy.data.worlds.new('World')
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs['Color'].default_value = (
        0.55, 0.58, 0.62, 1.0)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.30
    scene.world = world

    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 96
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 640
    scene.render.resolution_y = 480
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = out
    scene.view_settings.view_transform = 'Standard'
    scene.view_settings.look = 'None'
    scene.view_settings.exposure = -0.2

    bpy.ops.render.render(write_still=True)
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
