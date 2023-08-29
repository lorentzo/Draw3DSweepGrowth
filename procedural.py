
import bpy
import mathutils
import bmesh

# TODO
# Add version with spheres
# Clean the code
# Cool animated scene!

# Interpolate [a,b] using factor t.
def lerp(t, a, b):
    return (1.0 - t) * a + t * b

def create_collection_if_not_exists(collection_name):
    if collection_name not in bpy.data.collections:
        new_collection = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(new_collection) #Creates a new collection

def add_object_to_collection(base_object, collection_name="collection"):
    create_collection_if_not_exists(collection_name)
    bpy.data.collections[collection_name].objects.link(base_object)

# Based on: https://blog.federicopepe.com/en/2020/05/create-random-palettes-of-colors-that-will-go-well-together/
def generate_5_random_colors_that_fit():
    hue = int(mathutils.noise.random() * 360.0) # Random between [0,360]
    hue_op = int(mathutils.noise.random() * 180.0) # Random between [0,180]
    hues = [
        hue,
        hue - hue_op,
        hue + hue_op,
        hue - 2 * hue_op,
        hue + 2 * hue_op]
    rand_cols = []
    for i in range (5):
        col = mathutils.Color()
        col.hsv = (hues[i]/360.0, mathutils.noise.random(), mathutils.noise.random())
        rand_cols.append(col)
    return rand_cols

def generate_n_gradient_colors_with_same_random_hue(n=10):
    hue = mathutils.noise.random()
    rand_cols = []
    for i in range(n):
        col = mathutils.Color()
        col.hsv = (hue, mathutils.noise.random(), mathutils.noise.random())
        rand_cols.append(col)
    return rand_cols

def create_material(mat_id, mat_type, color=mathutils.Color((1.0, 0.5, 0.1))):

    mat = bpy.data.materials.get(mat_id)

    if mat is None:
        mat = bpy.data.materials.new(name=mat_id)

    mat.use_nodes = True

    if mat.node_tree:
        mat.node_tree.links.clear()
        mat.node_tree.nodes.clear()

    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')

    if mat_type == "diffuse":
        shader = nodes.new(type='ShaderNodeBsdfDiffuse')
        nodes["Diffuse BSDF"].inputs[0].default_value = color[:] + (1.0,)

    elif mat_type == "emission":
        shader = nodes.new(type='ShaderNodeEmission')
        nodes["Emission"].inputs[0].default_value = color[:] + (1.0,)
        nodes["Emission"].inputs[1].default_value = 1

    elif mat_type == "glossy":
        shader = nodes.new(type='ShaderNodeBsdfGlossy')
        nodes["Glossy BSDF"].inputs[0].default_value = color[:] + (1.0,)
        nodes["Glossy BSDF"].inputs[1].default_value = 0

    links.new(shader.outputs[0], output.inputs[0])

    return mat

def create_cube(size=2, location=mathutils.Vector((0,0,0)), collection_name=None):
    # Make a new BMesh.
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=size, matrix=mathutils.Matrix.Identity(4), calc_uvs=False)
    # Transform.
    bmesh.ops.translate(
        bm,
        verts=bm.verts,
        vec=location)
    # Write the bmesh into a new mesh.
    me = bpy.data.meshes.new("Mesh")
    bm.to_mesh(me)
    bm.free()
    # Add the mesh to the scene.
    obj = bpy.data.objects.new("Object", me)
    if collection_name == None:
        bpy.context.collection.objects.link(obj)
    else:
        add_object_to_collection(obj, collection_name)
    return obj

def copy_obj(obj, collection_name=None):
    obj_cpy = obj.copy()
    obj_cpy.data = obj.data.copy()
    obj_cpy.animation_data_clear()
    if collection_name == None:
        bpy.context.collection.objects.link(obj_cpy)
    else:
        add_object_to_collection(obj_cpy, collection_name)
    return obj_cpy

def create_instance(base_obj, pos_vec=mathutils.Vector((0,0,0)), rot_vec=mathutils.Vector((0,0,0)), collection_name=None):
    # Create instance.
    inst_obj = bpy.data.objects.new(base_obj.name+"_inst", base_obj.data)
    # Transform.
    transform_obj_rot_pos(inst_obj, pos_vec, rot_vec)
    # Store.
    if collection_name == None:
        bpy.context.collection.objects.link(inst_obj)
    else:
        create_collection_if_not_exists(collection_name)
        bpy.data.collections[collection_name].objects.link(inst_obj)
    return inst_obj

def instance_on_path(base_obj=None, scale_range=[1,2], curve_path=None, dt=0.01, frame_start=1, frame_end=200, collection_name=None):
    """
        Use Blender's "FOLLOW PATH" constraint to find position on given curve (sketch).
        Position is controlled with t in [0,1]
    """
    instances = []
    if not base_obj:
        return instances
    t = 0
    while t <= 1.0:
        # Create instance in local space. Configure its rotation.
        inst = create_instance(base_obj, rot_vec=mathutils.Vector((3,3,0)), collection_name=collection_name)
        # Animate scale.
        inst.scale = (0,0,0)
        inst.keyframe_insert(data_path="scale", frame=frame_start)
        adaptive_frame_start = lerp(t, frame_start, frame_end)
        inst.keyframe_insert(data_path="scale", frame=adaptive_frame_start)
        inst_scale = lerp(1.0-t, scale_range[0], scale_range[1])
        print(inst_scale)
        inst.scale = mathutils.Vector((inst_scale, inst_scale, inst_scale))
        inst.keyframe_insert(data_path="scale", frame=frame_end)
        # Position on path using "FOLLOW PATH" constraint
        follow_path_constraint = inst.constraints.new("FOLLOW_PATH")
        follow_path_constraint.target = curve_path
        follow_path_constraint.use_fixed_location = True
        follow_path_constraint.offset_factor = t
        #set_animation_fcurve(inst, "LINEAR")
        t += dt
        instances.append(inst)
    return instances

# https://graphics.pixar.com/library/OrthonormalB/paper.pdf
# NOTE: n must be normalized!
def pixar_onb(_n):
    n = _n.normalized()
    t = mathutils.Vector((0,0,0))
    b = mathutils.Vector((0,0,0))
    if(n[2] < 0.0):
        a = 1.0 / (1.0 - n[2])
        b = n[0] * n[1] * a
        t = mathutils.Vector((1.0 - n[0] * n[0] * a, -b, n[0]))
        b = mathutils.Vector((b, n[1] * n[1] * a - 1.0, -n[1]))
    else:
        a = 1.0 / (1.0 + n[2])
        b = -n[0] * n[1] * a
        t = mathutils.Vector((1.0 - n[0] * n[0] * a, b, -n[0]))
        b = mathutils.Vector((b, 1 - n[1] * n[1] * a, -n[1]))
    return t, b

def transform_obj_rot_pos(base_obj, pos_vec=mathutils.Vector((0,0,0)), rot_vec=mathutils.Vector((0,0,0))):
    # Extract position and scale.
    #original_scale_vec = base_obj.matrix_world.to_scale()
    #original_mat_scale = mathutils.Matrix.Scale(0.5, 4, (0.0, 0.0, 1.0))
    #original_pos_vec = base_obj.matrix_world.to_translation()
    #original_mat_trans = mathutils.Matrix.Translation(original_pos_vec)
    # 
    # zero out curr rotation matrix first: https://blender.stackexchange.com/a/159992
    curr_rot_mat_inv = base_obj.matrix_basis.to_3x3().transposed().to_4x4()
    base_obj.matrix_basis = base_obj.matrix_basis @ curr_rot_mat_inv
    # Orient object using vector.
    new_rot_z = rot_vec.normalized()
    new_rot_x, new_rot_y = pixar_onb(new_rot_z)
    rot_basis = mathutils.Matrix((new_rot_x, new_rot_y, new_rot_z))
    rot_basis = rot_basis.transposed()
    rot_basis.resize_4x4()
    rot_mat = rot_basis.to_euler().to_matrix().to_4x4() # extract only rotation!
    base_obj.matrix_basis = base_obj.matrix_basis @ rot_mat # https://blender.stackexchange.com/questions/35125/what-is-matrix-basis
    # Transform object using vector.
    new_pos = pos_vec
    trans_mat = mathutils.Matrix.Translation(new_pos)
    base_obj.matrix_basis = trans_mat @ base_obj.matrix_basis

# https://behreajj.medium.com/scripting-curves-in-blender-with-python-c487097efd13
def set_animation_fcurve(base_object, option='ELASTIC'):
    fcurves = base_object.data.animation_data.action.fcurves
    for fcurve in fcurves:
        for kf in fcurve.keyframe_points:
            # Options: ['CONSTANT', 'LINEAR', 'BEZIER', 'SINE',
            # 'QUAD', 'CUBIC', 'QUART', 'QUINT', 'EXPO', 'CIRC',
            # 'BACK', 'BOUNCE', 'ELASTIC']
            kf.interpolation = option
            # Options: ['AUTO', 'EASE_IN', 'EASE_OUT', 'EASE_IN_OUT']
            kf.easing = 'AUTO'

def main():
    sketch_collection_name = "sketch_collection"
    working_collection_name = "working_collection"

    # Create rigid body world.
    if bpy.context.scene.rigidbody_world == None:
        bpy.ops.rigidbody.world_add()
        bpy.context.scene.rigidbody_world.enabled = True
        bpy.context.scene.rigidbody_world.collection = bpy.data.collections.new("RigidBodyCubeCollection")
    bpy.context.scene.use_gravity = True

    base_cube = create_cube(size=1, location=mathutils.Vector((0,0,0)), collection_name=working_collection_name)
    for sketch in bpy.data.collections[sketch_collection_name].all_objects:
        instances = instance_on_path(base_obj=base_cube, scale_range=[0.1,0.9], curve_path=sketch, dt=0.005, frame_start=1, frame_end=200, collection_name=working_collection_name)
        for inst in instances:
            bpy.context.scene.rigidbody_world.collection.objects.link(inst)
            inst.rigid_body.mass = 30.0
            inst.rigid_body.friction = 0.5
            inst.rigid_body.restitution = 0.1
            inst.rigid_body.enabled = True
            #inst.rigid_body.kinematic = True
            inst.rigid_body.linear_damping = 1.0
            #inst.rigid_body.type = 'PASSIVE'




#
# Script entry point.
#
if __name__ == "__main__":
    main()

