bl_info = {
    "name": "MMD 控制器 ",
    "author": "蛙灾 ",
    "version": (10, 4, 4),
    "blender": (5, 0, 0),
    "location": "3D Viewport > Sidebar > MMD IK/FK",
    "description": "控制器 + 表情面板",
    "category": "Rigging",
}

import bpy
import json
import os
from bpy.types import Panel, Operator
from mathutils import Matrix, Vector
from math import radians, pi, cos, sin
from bpy.props import FloatProperty, PointerProperty, EnumProperty, BoolProperty, StringProperty

# ============================================================
# 骨骼配置
# ============================================================
MMD_BONE_MAP = {
    "arm_L": {"shoulder": "肩.L", "upper": "腕.L", "twist": "腕捩.L", "lower": "ひじ.L", "end": "手首.L", "chain_count": 3},
    "arm_R": {"shoulder": "肩.R", "upper": "腕.R", "twist": "腕捩.R", "lower": "ひじ.R", "end": "手首.R", "chain_count": 3},
    "leg_L": {"upper": "足.L", "lower": "ひざ.L", "end": "足首.L", "toe_ex": "足先EX.L", "chain_count": 2},
    "leg_R": {"upper": "足.R", "lower": "ひざ.R", "end": "足首.R", "toe_ex": "足先EX.R", "chain_count": 2},
}

FK_PREFIX = "FK_"
IK_PREFIX = "IK_"
IK_TARGET_PREFIX = "IKT_"
IK_POLE_PREFIX = "IKP_"
CTRL_PREFIX = "CTRL_"
BEND_PREFIX = "BEND_"
AUTO_PREFIX = "AUTO_"  # 手指自动驱动层：由 BEND 控制，CTRL 作为其子骨骼用于手动微调
BIND_PREFIX = "BIND_"  # 绑定辅助层：按原骨静止位创建、挂在弯过的 IK_ 骨下，原骨 FOLLOW_IK 的实际目标

# 骨骼集合（Bone Collections）名称
COLL_HIDDEN = "MMD_Hidden_Bones"
COLL_BODY = "MMD_Body_Controllers"   # 总控、躯干、头颈、眼部
COLL_LIMB = "MMD_Limb_Controllers"   # 手指、手捩等通用四肢控制器
COLL_ARM_FK = "MMD_Arm_FK_Controllers"
COLL_ARM_IK = "MMD_Arm_IK_Controllers"
COLL_LEG_FK = "MMD_Leg_FK_Controllers"
COLL_LEG_IK = "MMD_Leg_IK_Controllers"
COLL_LEGACY_CTRL = "MMD_Controllers"  # 旧版本统一集合，保留用于迁移

CONTROL_COLLECTIONS = (
    COLL_BODY, COLL_LIMB,
    COLL_ARM_FK, COLL_ARM_IK,
    COLL_LEG_FK, COLL_LEG_IK,
    COLL_LEGACY_CTRL,
)

CONTROL_GROUP_COLLECTION = {
    "body": COLL_BODY,
    "limb": COLL_LIMB,
    "arm_fk": COLL_ARM_FK,
    "arm_ik": COLL_ARM_IK,
    "leg_fk": COLL_LEG_FK,
    "leg_ik": COLL_LEG_IK,
}

EXTRA_CONTROL_BONES = [
    "全ての親",  # 新增总控骨骼
    "センター", "センター2", "下半身", "上半身", "上半身2",
    "首", "頭",
    "肩P.L", "肩P.R",   # 耸肩骨：VPD/VMD 常给它姿势，必须有控制器可见可清
    "手捩.L", "手捩.R",
]

# 中心类骨骼：双圈形状、允许位移
CENTER_CONTROL_BONES = ("センター", "センター2", "全ての親")

# MMD 原生足部 IK 目标骨名（全/半角变体），生成系统时自动关闭其 IK 约束
MMD_NATIVE_IK_BONES = ("足ＩＫ", "つま先ＩＫ", "足IK", "つま先IK")

FINGER_CHAINS = {
    "親指":  ["親指０", "親指１", "親指２"],
    "人指":  ["人指１", "人指２", "人指３"],
    "中指":  ["中指１", "中指２", "中指３"],
    "薬指":  ["薬指１", "薬指２", "薬指３"],
    "小指":  ["小指１", "小指２", "小指３"],
}

LIMB_ITEMS = [
    ('arm_L', "左腕", ""), ('arm_R', "右腕", ""),
    ('leg_L', "左足", ""), ('leg_R', "右足", ""),
]

PROP_TO_LIMB = {
    "arm_l_ikfk": "arm_L", "arm_r_ikfk": "arm_R",
    "leg_l_ikfk": "leg_L", "leg_r_ikfk": "leg_R",
}
LIMB_TO_PROP = {v: k for k, v in PROP_TO_LIMB.items()}

def get_pose_bone(armature, name):
    return armature.pose.bones.get(name)

# ============================================================
# 自定义形状创建
# ============================================================
SHAPE_COLL = "MMD_控制器形状"


def _move_to_shape_collection(obj):
    """形状物体统一收进专属集合，不散在大纲里"""
    coll = bpy.data.collections.get(SHAPE_COLL)
    if coll is None:
        coll = bpy.data.collections.new(SHAPE_COLL)
    scene = bpy.context.scene
    if coll.name not in {c.name for c in scene.collection.children}:
        try:
            scene.collection.children.link(coll)
        except Exception:
            pass
    coll.hide_viewport = True
    coll.hide_render = True
    for uc in list(obj.users_collection):
        if uc != coll:
            uc.objects.unlink(obj)
    if obj.name not in coll.objects:
        coll.objects.link(obj)


def create_custom_shape(name, shape_type):
    if name in bpy.data.objects:
        obj = bpy.data.objects[name]
        _move_to_shape_collection(obj)
        return obj
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    _move_to_shape_collection(obj)
    obj.display_type = 'WIRE'
    obj.hide_viewport = True
    obj.hide_render = True

    if shape_type == 'SPHERE':
        verts, edges = [], []
        seg = 16
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * 0.5, sin(a) * 0.5, 0))
            edges.append((i, (i + 1) % seg))
        o1 = len(verts)
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * 0.5, 0, sin(a) * 0.5))
            edges.append((o1 + i, o1 + (i + 1) % seg))
        o2 = len(verts)
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((0, cos(a) * 0.5, sin(a) * 0.5))
            edges.append((o2 + i, o2 + (i + 1) % seg))
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'CIRCLE':
        verts, edges = [], []
        seg = 24
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * 0.5, 0, sin(a) * 0.5))
            if i > 0: edges.append((i - 1, i))
        edges.append((seg - 1, 0))
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'DOUBLE_CIRCLE':
        verts, edges = [], []
        seg = 24
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * 0.5, 0, sin(a) * 0.5))
            if i > 0: edges.append((i - 1, i))
        edges.append((seg - 1, 0))
        o = len(verts)
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * 0.35, 0, sin(a) * 0.35))
            if i > 0: edges.append((o + i - 1, o + i))
        edges.append((o + seg - 1, o))
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'CUBE_FRAME':
        s = 0.5
        verts = [(s, s, s), (s, s, -s), (s, -s, s), (s, -s, -s), (-s, s, s), (-s, s, -s), (-s, -s, s), (-s, -s, -s)]
        edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'FOOT_SHAPE':
        verts = [(-0.4, -0.3, 0), (0.4, -0.3, 0), (-0.4, 0.7, 0), (0.4, 0.7, 0), (-0.3, 0.8, 0), (0.3, 0.8, 0), (-0.4, -0.3, 0.3), (0.4, -0.3, 0.3)]
        edges = [(0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (4, 5), (0, 6), (1, 7), (6, 7)]
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'TOE_SHAPE':
        s = 0.4
        verts = [(-s, 0, 0), (s, 0, 0), (0, s * 1.5, 0), (-s * 0.6, 0, 0.15), (s * 0.6, 0, 0.15), (0, s, 0.15)]
        edges = [(0, 1), (1, 2), (2, 0), (3, 4), (4, 5), (5, 3), (0, 3), (1, 4), (2, 5)]
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'EYE_RING':
        verts, edges = [], []
        seg = 24
        r = 1.0
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * r, 0, sin(a) * r))
            edges.append((i, (i + 1) % seg))
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'EYES_OVAL':
        verts, edges = [], []
        seg = 36
        rx = 20.0  
        rz = 10.0  
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * rx, 0, sin(a) * rz))
            edges.append((i, (i + 1) % seg))
        mesh.from_pydata(verts, edges, [])

    elif shape_type == 'FINGER_BEND':
        verts, edges = [], []
        seg = 24
        rx = 0.5
        rz = 0.18
        for i in range(seg):
            a = (i / seg) * 2 * pi
            verts.append((cos(a) * rx, 0, sin(a) * rz))
            edges.append((i, (i + 1) % seg))
        mesh.from_pydata(verts, edges, [])

    return obj

def hide_all_collections_except(armature_obj, keep_names):
    armature = armature_obj.data
    if not hasattr(armature, "collections"): return
    for coll in armature.collections:
        coll.is_visible = coll.name in keep_names

# ============================================================
# 骨骼层级/集合管理 
# ============================================================
def set_bone_layer_hidden(armature_obj, bone_name):
    armature = armature_obj.data
    b = armature.bones.get(bone_name)
    if not b: return
    b.hide = True
    
    if hasattr(armature, "collections"): 
        coll = armature.collections.get(COLL_HIDDEN)
        if not coll:
            coll = armature.collections.new(COLL_HIDDEN)
            coll.is_visible = False
        coll.assign(b)
        
        # 从所有控制器集合中移除（包括旧版统一集合）
        for cname in CONTROL_COLLECTIONS:
            other = armature.collections.get(cname)
            if other and b.name in other.bones:
                other.unassign(b)
    else: 
        for i in range(32): b.layers[i] = (i == 31)
        armature.layers[31] = False

def set_bone_layer_control(armature_obj, bone_name, group="limb"):
    """把骨骼放进控制器集合。
    group 可为 body、limb、arm_fk、arm_ik、leg_fk、leg_ik。
    """
    armature = armature_obj.data
    b = armature.bones.get(bone_name)
    if not b: return

    target_name = CONTROL_GROUP_COLLECTION.get(group, COLL_LIMB)

    if hasattr(armature, "collections"): 
        coll = armature.collections.get(target_name)
        if not coll:
            coll = armature.collections.new(target_name)
            coll.is_visible = True
        coll.assign(b)
        
        # 从其他控制器/隐藏集合中移除（包括旧版统一集合）
        for cname in (COLL_HIDDEN, *CONTROL_COLLECTIONS):
            if cname == target_name:
                continue
            other = armature.collections.get(cname)
            if other and b.name in other.bones:
                other.unassign(b)
    else: 
        for i in range(32): b.layers[i] = (i == 0)
        armature.layers[0] = True

def update_limb_visibility(armature_obj, limb_key):
    limb_data = MMD_BONE_MAP[limb_key]
    prop_name = LIMB_TO_PROP[limb_key]
    is_ik = getattr(armature_obj.mmikfk_props, prop_name) >= 0.5
    armature = armature_obj.data

    names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
    if limb_data.get("twist"): names.append(limb_data["twist"])
    if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])

    for name in names:
        fk_b = armature.bones.get(FK_PREFIX + name)
        if fk_b: fk_b.hide = is_ik

    ikt = armature.bones.get(IK_TARGET_PREFIX + limb_data["end"])
    if ikt: ikt.hide = not is_ik

    ikp = armature.bones.get(IK_POLE_PREFIX + limb_data["lower"])
    if ikp: ikp.hide = not is_ik

    # 脚尖 IK 控制器跟随脚踝开关
    toe_ex_name = limb_data.get("toe_ex")
    if toe_ex_name:
        ikt_toe = armature.bones.get(IK_TARGET_PREFIX + toe_ex_name)
        if ikt_toe: ikt_toe.hide = not is_ik
        # FK 模式下显示 FK 脚尖；IK 模式下隐藏 FK 脚尖（避免和 IKT 重叠）
        fk_toe = armature.bones.get(FK_PREFIX + toe_ex_name)
        if fk_toe: fk_toe.hide = is_ik

def hide_all_internal_bones(armature_obj):
    for limb_key, limb_data in MMD_BONE_MAP.items():
        prop_name = LIMB_TO_PROP[limb_key]
        is_ik = getattr(armature_obj.mmikfk_props, prop_name) >= 0.5

        is_arm = limb_key.startswith("arm")
        fk_group = "arm_fk" if is_arm else "leg_fk"
        ik_group = "arm_ik" if is_arm else "leg_ik"

        names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
        if limb_data.get("twist"): names.append(limb_data["twist"])
        if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])

        for name in names:
            set_bone_layer_hidden(armature_obj, name)
            set_bone_layer_hidden(armature_obj, IK_PREFIX + name)
            set_bone_layer_hidden(armature_obj, BIND_PREFIX + name)

            set_bone_layer_control(armature_obj, FK_PREFIX + name, group=fk_group)
            fk_b = armature_obj.data.bones.get(FK_PREFIX + name)
            if fk_b: fk_b.hide = is_ik

        toe_ex_name = limb_data.get("toe_ex")
        if toe_ex_name:
            set_bone_layer_hidden(armature_obj, toe_ex_name)
            set_bone_layer_hidden(armature_obj, IK_PREFIX + toe_ex_name)
            set_bone_layer_control(armature_obj, FK_PREFIX + toe_ex_name, group=fk_group)
            fk_toe = armature_obj.data.bones.get(FK_PREFIX + toe_ex_name)
            # FK 模式下显示 FK 脚尖；IK 模式下隐藏（IKT 接管）
            if fk_toe: fk_toe.hide = is_ik

            # 脚尖 IK 控制器
            ikt_toe_name = IK_TARGET_PREFIX + toe_ex_name
            set_bone_layer_control(armature_obj, ikt_toe_name, group=ik_group)
            ikt_toe = armature_obj.data.bones.get(ikt_toe_name)
            if ikt_toe: ikt_toe.hide = not is_ik

        ikt_name = IK_TARGET_PREFIX + limb_data["end"]
        ikp_name = IK_POLE_PREFIX + limb_data["lower"]
        set_bone_layer_control(armature_obj, ikt_name, group=ik_group)
        set_bone_layer_control(armature_obj, ikp_name, group=ik_group)

        ikt = armature_obj.data.bones.get(ikt_name)
        if ikt: ikt.hide = not is_ik
        ikp = armature_obj.data.bones.get(ikp_name)
        if ikp: ikp.hide = not is_ik


def ensure_limb_controller_collections(armature_obj):
    """把旧版已生成的四肢控制器迁移到手/脚 IK/FK 独立集合。"""
    armature = armature_obj.data
    if not hasattr(armature, "collections"):
        return

    assignments = []
    for limb_key, limb_data in MMD_BONE_MAP.items():
        is_arm = limb_key.startswith("arm")
        fk_group = "arm_fk" if is_arm else "leg_fk"
        ik_group = "arm_ik" if is_arm else "leg_ik"
        names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
        if limb_data.get("twist"):
            names.append(limb_data["twist"])
        if limb_data.get("shoulder"):
            names.insert(0, limb_data["shoulder"])
        if limb_data.get("toe_ex"):
            names.append(limb_data["toe_ex"])
        assignments.extend((FK_PREFIX + name, fk_group) for name in names)
        assignments.append((IK_TARGET_PREFIX + limb_data["end"], ik_group))
        assignments.append((IK_POLE_PREFIX + limb_data["lower"], ik_group))
        if limb_data.get("toe_ex"):
            assignments.append((IK_TARGET_PREFIX + limb_data["toe_ex"], ik_group))

    existing = [(name, group) for name, group in assignments if armature.bones.get(name)]
    if not existing:
        return

    needs_migration = any(
        not armature.collections.get(CONTROL_GROUP_COLLECTION[group])
        or name not in armature.collections[CONTROL_GROUP_COLLECTION[group]].bones
        for name, group in existing
    )
    if not needs_migration:
        return

    for name, group in existing:
        set_bone_layer_control(armature_obj, name, group=group)

def remove_constraint_if_exists(pb, constraint_name):
    if not pb:
        return
    c = pb.constraints.get(constraint_name)
    if c:
        pb.constraints.remove(c)

def set_mmd_native_leg_ik(armature_obj, enabled):
    """开/关 MMD 原生足部 IK（足ＩＫ/つま先ＩＫ）。
    生成系统时自动关闭（不再需要手动去 MMD 插件里关），移除系统时恢复。
    返回改动的约束数量。"""
    count = 0
    for pb in armature_obj.pose.bones:
        for con in pb.constraints:
            if con.type != 'IK':
                continue
            sub = getattr(con, "subtarget", "") or ""
            if any(key in sub for key in MMD_NATIVE_IK_BONES):
                con.influence = 1.0 if enabled else 0.0
                count += 1
    return count

def _managed_pairs():
    """(原骨名, 控制骨名) 列表 + 只跟随旋转的原骨集合（脚尖）。
    控制骨都是原骨的静止位副本，姿势数值可直接互抄。"""
    pairs = []
    rot_only = set()
    for limb_data in MMD_BONE_MAP.values():
        names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
        if limb_data.get("twist"): names.append(limb_data["twist"])
        if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])
        pairs += [(n, FK_PREFIX + n) for n in names]
        if limb_data.get("toe_ex"):
            pairs.append((limb_data["toe_ex"], FK_PREFIX + limb_data["toe_ex"]))
            rot_only.add(limb_data["toe_ex"])
    pairs += [(n, CTRL_PREFIX + n) for n in EXTRA_CONTROL_BONES]
    for side in ("L", "R"):
        for seg_names in FINGER_CHAINS.values():
            pairs += [(f"{s}.{side}", CTRL_PREFIX + f"{s}.{side}") for s in seg_names]
    return pairs, rot_only


def _apply_local(pb, loc, rot, write_loc):
    """把本地空间的位移/旋转写进骨骼通道（按各自旋转模式）"""
    if write_loc and not all(pb.lock_location):
        pb.location = loc
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = rot
    elif pb.rotation_mode == 'AXIS_ANGLE':
        axis, angle = rot.to_axis_angle()
        pb.rotation_axis_angle = (angle, axis.x, axis.y, axis.z)
    else:
        pb.rotation_euler = rot.to_euler(pb.rotation_mode)


def _key_pb(pb, frame):
    pb.keyframe_insert("location", frame=frame)
    if pb.rotation_mode == 'QUATERNION':
        pb.keyframe_insert("rotation_quaternion", frame=frame)
    elif pb.rotation_mode == 'AXIS_ANGLE':
        pb.keyframe_insert("rotation_axis_angle", frame=frame)
    else:
        pb.keyframe_insert("rotation_euler", frame=frame)


def _strip_bone_curves(armature_obj, names, also_native_ik=False):
    """删掉动作里指定骨骼的曲线（已被烘焙消化的原骨/原生IK骨曲线）"""
    ad = armature_obj.animation_data
    if not ad or not ad.action:
        return 0
    targets = set(names)
    removed = 0
    for fc in list(ad.action.fcurves):
        dp = fc.data_path
        if not dp.startswith('pose.bones["'):
            continue
        bname = dp.split('"')[1]
        if bname in targets or (also_native_ik and any(k in bname for k in MMD_NATIVE_IK_BONES)):
            ad.action.fcurves.remove(fc)
            removed += 1
    return removed


def _writeback_native_ik(context, armature_obj, frame=None):
    """把当前 足首/足先EX 的视觉位置反推成 足ＩＫ/つま先ＩＫ 的位移值。
    frame 给定时顺便 K 位置帧（VMD 烘焙用）"""
    written = 0
    for pb in armature_obj.pose.bones:
        nm = pb.name
        side = "L" if nm.endswith(".L") else ("R" if nm.endswith(".R") else None)
        if side is None:
            continue
        if "つま先" in nm and ("ＩＫ" in nm or "IK" in nm):
            tgt = get_pose_bone(armature_obj, f"足先EX.{side}")
        elif ("足ＩＫ" in nm or "足IK" in nm) and "親" not in nm:
            tgt = get_pose_bone(armature_obj, f"足首.{side}")
        else:
            continue
        if tgt is None:
            continue
        m = pb.matrix.copy()
        m.translation = tgt.matrix.translation.copy()
        m_local = armature_obj.convert_space(pose_bone=pb, matrix=m,
                                             from_space='POSE', to_space='LOCAL')
        pb.location = m_local.decompose()[0]
        if frame is not None:
            pb.keyframe_insert("location", frame=frame)
        written += 1
    return written


def _reset_bend_bones(armature_obj):
    """BEND 球归零（旋转清零、缩放回 1）。
    吸附姿势/动作时用：BEND 的贡献已经烘进吸附值里，留着会双重叠加"""
    n = 0
    for side in ("L", "R"):
        for finger_name in FINGER_CHAINS:
            bp = get_pose_bone(armature_obj, BEND_PREFIX + f"{finger_name}.{side}")
            if bp:
                bp.rotation_euler = (0.0, 0.0, 0.0)
                bp.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                bp.scale = (1.0, 1.0, 1.0)
                n += 1
    return n


def _mmd_root_of(obj):
    """顺着父级找 mmd_tools 的模型根（十字空物体）"""
    while obj:
        if getattr(obj, "mmd_type", "NONE") == 'ROOT':
            return obj
        obj = obj.parent
    return None


def _mmd_tools_ready(opname):
    ops = getattr(bpy.ops, "mmd_tools", None)
    if ops is None:
        return False
    try:
        return hasattr(ops, opname)
    except Exception:
        return False


def _filtered_op_kwargs(op, kwargs):
    """按对方操作符实际存在的参数过滤，跨版本不炸"""
    avail = op.get_rna_type().properties.keys()
    return {k: v for k, v in kwargs.items() if k in avail}


def _with_mmd_selection(context, armature_obj, fn):
    """临时选中 骨架+模型根（表情才会一起处理），active=根；跑完还原"""
    prev_sel = list(context.selected_objects)
    prev_active = context.view_layer.objects.active
    root = _mmd_root_of(armature_obj)
    for o in context.view_layer.objects:
        try:
            o.select_set(False)
        except RuntimeError:
            pass
    for o in (armature_obj, root):
        if o:
            try:
                o.select_set(True)
            except RuntimeError:
                pass
    context.view_layer.objects.active = root or armature_obj
    try:
        return fn()
    finally:
        for o in context.view_layer.objects:
            try:
                o.select_set(False)
            except RuntimeError:
                pass
        for o in prev_sel:
            try:
                o.select_set(True)
            except RuntimeError:
                pass
        context.view_layer.objects.active = prev_active


_D_PIN_BASES = ("足首", "ひざ")


def _pin_d_bones(armature_obj):
    """足首D/ひざD 这类 D 变形骨：位置钉回本尊。
    预弯曲让 IK 链段长与原骨有微差，末端闭合误差会跑到没人钉的 D 链上，
    表现为下蹲时脚部轻微平移"""
    n = 0
    for side in ("L", "R"):
        for base in _D_PIN_BASES:
            d = get_pose_bone(armature_obj, f"{base}D.{side}")
            src = get_pose_bone(armature_obj, f"{base}.{side}")
            if not (d and src):
                continue
            remove_constraint_if_exists(d, "MMIKFK_D_PIN")
            con = d.constraints.new('COPY_LOCATION')
            con.name = "MMIKFK_D_PIN"
            con.target = armature_obj
            con.subtarget = src.name
            n += 1
    return n


def _unpin_d_bones(armature_obj):
    for side in ("L", "R"):
        for base in _D_PIN_BASES:
            remove_constraint_if_exists(
                get_pose_bone(armature_obj, f"{base}D.{side}"), "MMIKFK_D_PIN")


def guess_bend_dir(ebs, limb_key, limb_data):
    """解剖学判弯向（骨架空间单位向量，垂直于肢体主轴）。
    手臂：肘尖凸向拇指反方向；腿：膝盖凸向脚尖方向。判不出返回 None。"""
    upper = ebs.get(limb_data["upper"])
    end = ebs.get(limb_data["end"])
    if not (upper and end):
        return None
    axis = end.head - upper.head
    if axis.length < 1e-6:
        return None
    axis = axis.normalized()

    side = limb_data["upper"].split(".")[-1]
    if "arm" in limb_key:
        start = ebs.get(f"親指０.{side}") or ebs.get(f"親指１.{side}")
        tip = ebs.get(f"親指２.{side}") or ebs.get(f"親指１.{side}")
        if not (start and tip):
            return None
        hint = -(tip.tail - start.head)
    else:
        ankle = end
        toe = ebs.get(limb_data.get("toe_ex") or "")
        if toe:
            hint = toe.head - ankle.head
            if hint.length < 1e-6:
                hint = toe.tail - ankle.head
        else:
            hint = ankle.tail - ankle.head

    if hint.length < 1e-6:
        return None
    hint = hint.normalized()
    bend = hint - axis * hint.dot(axis)
    # 提示方向与主轴太接近时视为判不出
    if bend.length < 0.2:
        return None
    return bend.normalized()

def solve_pole_angle(context, armature_obj, ik_lower_name, ikp_name):
    """扫描 pole_angle，使 IK 弯曲平面对准极向量。"""
    ik_lower_pb = get_pose_bone(armature_obj, ik_lower_name)
    ikp_pb = get_pose_bone(armature_obj, ikp_name)
    if not (ik_lower_pb and ikp_pb):
        return
    ik_con = ik_lower_pb.constraints.get("IK_SOLVER")
    if not ik_con:
        return
    best_angle = 0
    best_dist = float('inf')
    for test_angle in range(-180, 180, 1):
        ik_con.pole_angle = radians(test_angle)
        context.view_layer.update()
        dist = (ik_lower_pb.head - ikp_pb.head).length
        if dist < best_dist:
            best_dist = dist
            best_angle = test_angle
    ik_con.pole_angle = radians(best_angle)

def remove_rotation_drivers(pb):
    if not pb:
        return
    for idx in range(3):
        try:
            pb.driver_remove("rotation_euler", idx)
        except Exception:
            pass

def add_driver_to_constraint(armature_obj, bone_name, constraint_name, prop_name, expression="ikfk"):
    pb = get_pose_bone(armature_obj, bone_name)
    if not pb: return
    con = pb.constraints.get(constraint_name)
    if not con: return
    try:
        con.driver_remove("influence")
    except Exception:
        pass
    fcurve = con.driver_add("influence")
    driver = fcurve.driver
    driver.type = 'SCRIPTED'
    for v in list(driver.variables):
        driver.variables.remove(v)
    var = driver.variables.new()
    var.name = "ikfk"
    var.type = 'SINGLE_PROP'
    target = var.targets[0]
    target.id_type = 'OBJECT'
    target.id = armature_obj
    target.data_path = f'mmikfk_props.{prop_name}'
    driver.expression = expression

def add_ikfk_follow_drivers(armature_obj, bone_name, prop_name):
    # IK/FK 互补驱动：IK=ikfk，FK=1-ikfk，避免滑条中间值时双重叠加。
    add_driver_to_constraint(armature_obj, bone_name, "FOLLOW_IK", prop_name, "ikfk")
    add_driver_to_constraint(armature_obj, bone_name, "FOLLOW_FK", prop_name, "1 - ikfk")

FINGER_SPREAD_WEIGHTS = {"人指": 1.0, "中指": 0.33, "薬指": -0.33, "小指": -1.0}

def _spread_axis_for(bend_axis):
    if bend_axis == 'X': return 'Z'
    if bend_axis == 'Z': return 'X'
    return 'Z'

def _rebuild_finger_drivers(armature_obj):
    """重建手指自动层驱动。

    专业版层级：
        BEND_手指  -> 驱动 AUTO_每节手指
        CTRL_每节手指 -> 作为 AUTO_子骨骼，给用户自由多轴微调
        原始手指骨骼 -> Copy Transforms CTRL_每节手指

    这样批量弯曲/张开不会直接占用 CTRL 的 rotation_euler，
    用户仍可在 CTRL 上做 X/Y/Z 多方向手动旋转。
    """
    if not armature_obj or armature_obj.type != 'ARMATURE':
        return
    props = armature_obj.mmikfk_props
    axis_to_idx = {'X': 0, 'Y': 1, 'Z': 2}
    transform_map = {'X': 'ROT_X', 'Y': 'ROT_Y', 'Z': 'ROT_Z'}
    weights = [1.0, 1.0, 1.0]

    # 清理 CTRL/AUTO 上的旧驱动，保证重复生成不堆叠
    for side in ("L", "R"):
        for finger_name, seg_names in FINGER_CHAINS.items():
            for seg in seg_names:
                orig = f"{seg}.{side}"
                remove_rotation_drivers(get_pose_bone(armature_obj, CTRL_PREFIX + orig))
                remove_rotation_drivers(get_pose_bone(armature_obj, AUTO_PREFIX + orig))

    for side in ("L", "R"):
        axis_normal = props.finger_bend_axis_l if side == "L" else props.finger_bend_axis_r
        sign_normal = props.finger_bend_sign_l if side == "L" else props.finger_bend_sign_r
        axis_thumb = props.thumb_bend_axis_l if side == "L" else props.thumb_bend_axis_r
        sign_thumb = props.thumb_bend_sign_l if side == "L" else props.thumb_bend_sign_r
        spread_sign = props.finger_spread_sign_l if side == "L" else props.finger_spread_sign_r

        for finger_name, seg_names in FINGER_CHAINS.items():
            full_names = [f"{n}.{side}" for n in seg_names]
            bend_name = BEND_PREFIX + f"{finger_name}.{side}"
            if not get_pose_bone(armature_obj, bend_name):
                continue

            if finger_name == "親指":
                axis = axis_thumb
                sign = sign_thumb
            else:
                axis = axis_normal
                sign = sign_normal
            tgt_idx = axis_to_idx[axis]

            for i, orig in enumerate(full_names):
                auto_pb = get_pose_bone(armature_obj, AUTO_PREFIX + orig)
                ctrl_pb = get_pose_bone(armature_obj, CTRL_PREFIX + orig)
                if not auto_pb:
                    continue

                auto_pb.rotation_mode = 'XYZ'
                if ctrl_pb:
                    ctrl_pb.rotation_mode = 'XYZ'
                    remove_rotation_drivers(ctrl_pb)  # CTRL 永远保留给用户手动控制

                fcurve = auto_pb.driver_add("rotation_euler", tgt_idx)
                drv = fcurve.driver
                drv.type = 'SCRIPTED'
                for v in list(drv.variables):
                    drv.variables.remove(v)
                var = drv.variables.new()
                var.name = "bend"
                var.type = 'TRANSFORMS'
                tgt = var.targets[0]
                tgt.id = armature_obj
                tgt.bone_target = bend_name
                tgt.transform_type = transform_map[axis]
                tgt.transform_space = 'LOCAL_SPACE'
                drv.expression = f"bend * {sign * weights[i]:.3f}"

                if finger_name != "親指" and i == 0 and finger_name in FINGER_SPREAD_WEIGHTS:
                    spread_axis = _spread_axis_for(axis)
                    spread_idx = axis_to_idx[spread_axis]
                    if spread_idx == tgt_idx:
                        continue
                    base_w = FINGER_SPREAD_WEIGHTS[finger_name]
                    amplitude = 0.52
                    coeff = base_w * amplitude * spread_sign
                    # 张开并拢并进 BEND 球：选中 BEND 按 S 缩放，>1 张开 <1 并拢
                    fcurve2 = auto_pb.driver_add("rotation_euler", spread_idx)
                    drv2 = fcurve2.driver
                    drv2.type = 'SCRIPTED'
                    for v in list(drv2.variables):
                        drv2.variables.remove(v)
                    var2 = drv2.variables.new()
                    var2.name = "s"
                    var2.type = 'TRANSFORMS'
                    t2 = var2.targets[0]
                    t2.id = armature_obj
                    t2.bone_target = bend_name
                    t2.transform_type = 'SCALE_AVG'
                    t2.transform_space = 'LOCAL_SPACE'
                    drv2.expression = f"(s - 1.0) * {coeff:.4f}"

def _finger_axis_update(self, context):
    obj = context.active_object
    if obj and obj.type == 'ARMATURE': _rebuild_finger_drivers(obj)

class MMIKFK_Properties(bpy.types.PropertyGroup):
    arm_l_ikfk: FloatProperty(name="左腕 IK/FK", default=0.0, min=0.0, max=1.0)
    arm_r_ikfk: FloatProperty(name="右腕 IK/FK", default=0.0, min=0.0, max=1.0)
    leg_l_ikfk: FloatProperty(name="左足 IK/FK", default=0.0, min=0.0, max=1.0)
    leg_r_ikfk: FloatProperty(name="右足 IK/FK", default=0.0, min=0.0, max=1.0)

    finger_bend_axis_l: EnumProperty(
        name="左手弯曲轴", description="手指绕该局部轴弯曲",
        items=[('X', "X 轴", ""), ('Y', "Y 轴", ""), ('Z', "Z 轴", "")], default='X', update=_finger_axis_update,
    )
    finger_bend_axis_r: EnumProperty(
        name="右手弯曲轴", description="手指绕该局部轴弯曲",
        items=[('X', "X 轴", ""), ('Y', "Y 轴", ""), ('Z', "Z 轴", "")], default='X', update=_finger_axis_update,
    )
    finger_bend_sign_l: FloatProperty(name="左手弯曲方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)
    finger_bend_sign_r: FloatProperty(name="右手弯曲方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)

    thumb_bend_axis_l: EnumProperty(
        name="左拇指弯曲轴", description="左拇指绕该局部轴弯曲",
        items=[('X', "X 轴", ""), ('Y', "Y 轴", ""), ('Z', "Z 轴", "")], default='X', update=_finger_axis_update,
    )
    thumb_bend_axis_r: EnumProperty(
        name="右拇指弯曲轴", description="右拇指绕该局部轴弯曲",
        items=[('X', "X 轴", ""), ('Y', "Y 轴", ""), ('Z', "Z 轴", "")], default='X', update=_finger_axis_update,
    )
    thumb_bend_sign_l: FloatProperty(name="左拇指弯曲方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)
    thumb_bend_sign_r: FloatProperty(name="右拇指弯曲方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)

    finger_spread_sign_l: FloatProperty(name="左手张开方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)
    finger_spread_sign_r: FloatProperty(name="右手张开方向", default=1.0, min=-1.0, max=1.0, update=_finger_axis_update)

    ui_show_limbs: BoolProperty(default=False)
    ui_show_finger_bend: BoolProperty(default=False)
    ui_show_prebend: BoolProperty(default=False)

    # 预弯曲参数：生成系统时自动用于 IK 控制链；「预弯曲手肘膝盖」按钮直接弯模型骨骼
    prebend_amount: FloatProperty(
        name="预弯曲量",
        description="ひじ/ひざ 沿 Y 轴的弯曲偏移。生成系统时自动内置到 IK 控制链"
                    "（不改模型骨架，已有弯曲时跳过）；「预弯曲手肘膝盖」按钮则直接"
                    "弯模型骨骼。0.01~0.03 通常足够。",
        default=0.02, min=0.0, max=0.2, step=0.1, precision=3,
    )
    prebend_invert: BoolProperty(
        name="反向（面朝 +Y 模型）",
        description="默认假设角色面朝 -Y（标准 MMD 朝向）。"
                    "如果模型面朝 +Y，则勾选此项以反转弯曲方向。",
        default=False,
    )

# ============================================================
# 核心构建系统
# ============================================================
class MMIKFK_OT_Setup(bpy.types.Operator):
    bl_idname = "mmikfk.setup"
    bl_label = "生成骨骼系统"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context): return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        props = armature_obj.mmikfk_props

        bpy.ops.object.mode_set(mode='EDIT')

        def copy_bone(orig_name, new_name):
            if new_name in armature.edit_bones: return armature.edit_bones[new_name]
            orig = armature.edit_bones.get(orig_name)
            if not orig: return None
            b = armature.edit_bones.new(new_name)
            b.head = orig.head.copy()
            b.tail = orig.tail.copy()
            b.roll = orig.roll
            b.use_deform = False
            return b

        bend_dirs = {}

        for limb_key, limb_data in MMD_BONE_MAP.items():
            names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
            if limb_data.get("twist"): names.insert(1, limb_data["twist"])
            if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])

            if not all(n in armature.edit_bones for n in names): continue

            orig_upper = armature.edit_bones[limb_data["upper"]]
            orig_lower = armature.edit_bones[limb_data["lower"]]
            orig_end = armature.edit_bones[limb_data["end"]]
            orig_shoulder = armature.edit_bones.get(limb_data.get("shoulder")) if limb_data.get("shoulder") else None

            fk_bones = {}; ik_bones = {}
            for name in names:
                fk_bones[name] = copy_bone(name, FK_PREFIX + name)
                ik_bones[name] = copy_bone(name, IK_PREFIX + name)

            if orig_shoulder:
                fk_bones[limb_data["shoulder"]].parent = orig_shoulder.parent
                ik_bones[limb_data["shoulder"]].parent = orig_shoulder.parent
                fk_bones[limb_data["upper"]].parent = fk_bones[limb_data["shoulder"]]
                ik_bones[limb_data["upper"]].parent = ik_bones[limb_data["shoulder"]]
            else:
                fk_bones[limb_data["upper"]].parent = orig_upper.parent
                ik_bones[limb_data["upper"]].parent = orig_upper.parent

            if limb_data.get("twist"):
                twist_name = limb_data["twist"]
                fk_bones[twist_name].parent = fk_bones[limb_data["upper"]]
                ik_bones[twist_name].parent = ik_bones[limb_data["upper"]]
                fk_bones[limb_data["lower"]].parent = fk_bones[twist_name]
                ik_bones[limb_data["lower"]].parent = ik_bones[twist_name]
            else:
                fk_bones[limb_data["lower"]].parent = fk_bones[limb_data["upper"]]
                ik_bones[limb_data["lower"]].parent = ik_bones[limb_data["upper"]]

            fk_bones[limb_data["end"]].parent = fk_bones[limb_data["lower"]]
            ik_bones[limb_data["end"]].parent = ik_bones[limb_data["lower"]]

            # IK 层内置预弯曲：只弯 IK_ 控制链；已有明显弯曲时跳过，不叠加
            ik_upper = ik_bones[limb_data["upper"]]
            ik_lower = ik_bones[limb_data["lower"]]
            ik_end_eb = ik_bones[limb_data["end"]]
            seg1 = ik_lower.head - ik_upper.head
            seg2 = ik_end_eb.head - ik_lower.head
            already_bent = False
            if seg1.length > 1e-6 and seg2.length > 1e-6:
                already_bent = seg1.angle(seg2) > radians(5.0)
            is_arm_limb = "arm" in limb_key
            if already_bent:
                # 记录既有弯向，供极向量摆放和笔直退化兜底
                axis_v = ik_end_eb.head - ik_upper.head
                if axis_v.length > 1e-6:
                    an = axis_v.normalized()
                    ul = ik_lower.head - ik_upper.head
                    ex_bend = ul - an * ul.dot(an)
                    if ex_bend.length > 1e-9:
                        bend_dirs[limb_key] = list(ex_bend.normalized())
            elif props.prebend_amount > 0.0:
                # 弯向：手指/脚尖解剖学判向，判不出退回世界方向（手肘 +Y、膝盖 -Y）
                bend_dir = guess_bend_dir(armature.edit_bones, limb_key, limb_data)
                if bend_dir is None:
                    bend_dir = Vector((0.0, 1.0, 0.0)) if is_arm_limb else Vector((0.0, -1.0, 0.0))
                if props.prebend_invert:
                    bend_dir = -bend_dir
                ik_lower.head = ik_lower.head + bend_dir * props.prebend_amount
                ik_upper.tail = ik_lower.head.copy()
                bend_dirs[limb_key] = list(bend_dir)

            # BIND_ 辅助骨：按原骨静止位创建、挂在 IK_ 骨下，静止时与原骨重合
            for bind_src in (limb_data["upper"], limb_data["lower"]):
                bind_name = BIND_PREFIX + bind_src
                if bind_name not in armature.edit_bones:
                    orig_eb = armature.edit_bones[bind_src]
                    bb = armature.edit_bones.new(bind_name)
                    bb.head = orig_eb.head.copy()
                    bb.tail = orig_eb.tail.copy()
                    bb.roll = orig_eb.roll
                    bb.use_deform = False
                armature.edit_bones[bind_name].parent = ik_bones[bind_src]

            toe_ex_name = limb_data.get("toe_ex")
            if toe_ex_name and toe_ex_name in armature.edit_bones:
                fk_toe = copy_bone(toe_ex_name, FK_PREFIX + toe_ex_name)
                ik_toe = copy_bone(toe_ex_name, IK_PREFIX + toe_ex_name)
                if fk_toe: fk_toe.parent = fk_bones[limb_data["end"]]
                if ik_toe: ik_toe.parent = ik_bones[limb_data["end"]]

            ikt_name = IK_TARGET_PREFIX + limb_data["end"]
            ikt = copy_bone(limb_data["end"], ikt_name)
            # 世界空间 IK 目标也必须跟随角色总控。CTRL_全ての親稍后通过
            # FOLLOW_CTRL 驱动原始「全ての親」，因此挂在原骨上既能继承
            # 总控变换，又保留手腕/脚腕 IK 自身的独立位移与旋转。
            ikt.parent = armature.edit_bones.get("全ての親")

            # 脚尖控制器 IKT_足先EX：挂在脚踝 IKT 下自动跟随
            if toe_ex_name and toe_ex_name in armature.edit_bones:
                ikt_toe_name = IK_TARGET_PREFIX + toe_ex_name
                ikt_toe = copy_bone(toe_ex_name, ikt_toe_name)
                if ikt_toe:
                    ikt_toe.parent = ikt  # 跟随脚踝 IK 目标

            ikp_name = IK_POLE_PREFIX + limb_data["lower"]
            ikp = armature.edit_bones.get(ikp_name)
            if not ikp:
                ikp = armature.edit_bones.new(ikp_name)
                l_pos = orig_lower.head
                if limb_key in bend_dirs:
                    pole_dir = Vector(bend_dirs[limb_key])
                else:
                    pole_dir = Vector((0, 1, 0)) if "arm" in limb_key else Vector((0, -1, 0))
                offset = 0.4 if "arm" in limb_key else 0.5
                ikp.head = l_pos + pole_dir * offset
                ikp.tail = ikp.head + pole_dir * 0.06
                ikp.use_deform = False

            # 肘/膝极向量跟随对应的手腕/脚踝 IK 目标，同时保留局部偏移，
            # 移动末端控制器时弯曲方向控制器会一起移动。
            ikp.parent = ikt

        armature["mmikfk_bend_dirs"] = bend_dirs

        bpy.ops.object.mode_set(mode='POSE')

        fk_shape = create_custom_shape("MMD_FK_Shape_Circle", 'CIRCLE')
        fk_end_shape = create_custom_shape("MMD_FK_End_Shape", 'DOUBLE_CIRCLE')
        fk_hand_shape = create_custom_shape("MMD_FK_Hand_Shape", 'CIRCLE')
        fk_arm_shape = create_custom_shape("MMD_FK_Arm_Shape", 'CUBE_FRAME')
        arm_target_shape = create_custom_shape("MMD_IK_Arm_Target", 'CUBE_FRAME')
        foot_target_shape = create_custom_shape("MMD_IK_Foot_Target", 'FOOT_SHAPE')
        pole_shape = create_custom_shape("MMD_IK_Shape_Sphere", 'SPHERE')
        toe_shape = create_custom_shape("MMD_Toe_Shape", 'TOE_SHAPE')

        for limb_key, limb_data in MMD_BONE_MAP.items():
            upper_name = limb_data["upper"]
            lower_name = limb_data["lower"]
            end_name = limb_data["end"]
            twist_name = limb_data.get("twist")
            shoulder_name = limb_data.get("shoulder")
            toe_ex_name = limb_data.get("toe_ex")
            prop_name = LIMB_TO_PROP[limb_key]
            is_arm = "arm" in limb_key

            names_to_decorate = [upper_name, lower_name, end_name, twist_name]
            if shoulder_name: names_to_decorate.insert(0, shoulder_name)

            for name in names_to_decorate:
                if not name: continue
                fk_b = get_pose_bone(armature_obj, FK_PREFIX + name)
                if fk_b:
                    if name == end_name:
                        if is_arm:
                            fk_b.custom_shape = fk_hand_shape
                            fk_b.custom_shape_scale_xyz = (1.5, 1.5, 1.5)
                        else:
                            fk_b.custom_shape = fk_end_shape
                            fk_b.custom_shape_scale_xyz = (0.8, 0.8, 0.8)
                    elif name == shoulder_name:
                        fk_b.custom_shape = fk_shape
                        fk_b.custom_shape_scale_xyz = (1.0, 1.0, 1.0)
                    elif is_arm and name in (upper_name, lower_name):
                        if name == upper_name:
                            fk_b.custom_shape = fk_shape
                            fk_b.custom_shape_scale_xyz = (0.8, 0.8, 0.8)
                        else:
                            fk_b.custom_shape = fk_arm_shape
                            fk_b.custom_shape_scale_xyz = (0.45, 0.45, 0.45)
                    else:
                        fk_b.custom_shape = fk_shape
                        if is_arm: fk_b.custom_shape_scale_xyz = (1.3, 1.3, 1.3)
                        else: fk_b.custom_shape_scale_xyz = (0.7, 0.7, 0.7)
                    fk_b.color.palette = 'THEME04'
                    fk_b.lock_location = (True, True, True)

            if toe_ex_name:
                fk_toe_pb = get_pose_bone(armature_obj, FK_PREFIX + toe_ex_name)
                if fk_toe_pb:
                    fk_toe_pb.custom_shape = toe_shape
                    fk_toe_pb.custom_shape_scale_xyz = (0.6, 0.6, 0.6)
                    fk_toe_pb.color.palette = 'THEME04'
                    fk_toe_pb.lock_location = (True, True, True)

            if twist_name:
                fk_twist_pb = get_pose_bone(armature_obj, FK_PREFIX + twist_name)
                if fk_twist_pb:
                    fk_twist_pb.rotation_mode = 'XYZ'
                    fk_twist_pb.lock_rotation = (True, False, True)
                    fk_twist_pb.custom_shape_scale_xyz = (0.9, 0.9, 0.9)

            ik_lower_pb = get_pose_bone(armature_obj, IK_PREFIX + lower_name)
            remove_constraint_if_exists(ik_lower_pb, "IK_SOLVER")
            ik_con = ik_lower_pb.constraints.new('IK')
            ik_con.name = "IK_SOLVER"
            ik_con.target = armature_obj
            ik_con.subtarget = IK_TARGET_PREFIX + end_name
            ik_con.chain_count = limb_data["chain_count"]
            ik_con.pole_target = armature_obj
            ik_con.pole_subtarget = IK_POLE_PREFIX + lower_name

            if twist_name:
                ik_twist_pb = get_pose_bone(armature_obj, IK_PREFIX + twist_name)
                if ik_twist_pb:
                    ik_twist_pb.lock_ik_x, ik_twist_pb.lock_ik_y, ik_twist_pb.lock_ik_z = True, True, True

            solve_pole_angle(context, armature_obj, IK_PREFIX + lower_name, IK_POLE_PREFIX + lower_name)
            ikp_pb = get_pose_bone(armature_obj, IK_POLE_PREFIX + lower_name)

            ik_end_pb = get_pose_bone(armature_obj, IK_PREFIX + end_name)
            remove_constraint_if_exists(ik_end_pb, "IK_ROT")
            rot_con = ik_end_pb.constraints.new('COPY_ROTATION')
            rot_con.name = "IK_ROT"
            rot_con.target = armature_obj
            rot_con.subtarget = IK_TARGET_PREFIX + end_name

            ikt_pb = get_pose_bone(armature_obj, IK_TARGET_PREFIX + end_name)
            if is_arm:
                ikt_pb.custom_shape = arm_target_shape
                ikt_pb.custom_shape_scale_xyz = (1.2, 1.2, 1.2)
            else:
                ikt_pb.custom_shape = foot_target_shape
                ikt_pb.custom_shape_scale_xyz = (0.8, 0.8, 0.8)
            ikt_pb.color.palette = 'THEME01'

            ikp_pb.custom_shape = pole_shape
            ikp_pb.color.palette = 'THEME03'
            ikp_pb.custom_shape_scale_xyz = (1.25, 1.25, 1.25)

            # 脚尖控制器：纯旋转手柄，位置由脚踝层级决定
            if toe_ex_name:
                ik_toe_pb = get_pose_bone(armature_obj, IK_PREFIX + toe_ex_name)
                ikt_toe_pb = get_pose_bone(armature_obj, IK_TARGET_PREFIX + toe_ex_name)
                if ik_toe_pb and ikt_toe_pb:
                    remove_constraint_if_exists(ik_toe_pb, "IK_SOLVER_TOE")  # 清理旧版遗留
                    remove_constraint_if_exists(ik_toe_pb, "TOE_ROT")
                    toe_rot_con = ik_toe_pb.constraints.new('COPY_ROTATION')
                    toe_rot_con.name = "TOE_ROT"
                    toe_rot_con.target = armature_obj
                    toe_rot_con.subtarget = IK_TARGET_PREFIX + toe_ex_name

                    # 控制器视觉；位置锁死
                    ikt_toe_pb.custom_shape = toe_shape
                    ikt_toe_pb.custom_shape_scale_xyz = (0.8, 0.8, 0.8)
                    ikt_toe_pb.color.palette = 'THEME01'
                    ikt_toe_pb.lock_location = (True, True, True)

            names_to_bind = [upper_name, lower_name, end_name]
            if twist_name: names_to_bind.append(twist_name)
            if shoulder_name: names_to_bind.insert(0, shoulder_name)

            for name in names_to_bind:
                pb = get_pose_bone(armature_obj, name)
                if not pb:
                    continue
                remove_constraint_if_exists(pb, "FOLLOW_FK")
                remove_constraint_if_exists(pb, "FOLLOW_IK")

                fk_follow = pb.constraints.new('COPY_TRANSFORMS')
                fk_follow.name = "FOLLOW_FK"
                fk_follow.target = armature_obj
                fk_follow.subtarget = FK_PREFIX + name
                fk_follow.influence = 1.0

                ik_follow = pb.constraints.new('COPY_TRANSFORMS')
                ik_follow.name = "FOLLOW_IK"
                ik_follow.target = armature_obj
                # upper/lower 经 BIND_ 辅助骨跟随 IK 层
                bind_name = BIND_PREFIX + name
                if get_pose_bone(armature_obj, bind_name):
                    ik_follow.subtarget = bind_name
                else:
                    ik_follow.subtarget = IK_PREFIX + name
                ik_follow.influence = 0.0

                add_ikfk_follow_drivers(armature_obj, name, prop_name)

            if toe_ex_name:
                toe_pb = get_pose_bone(armature_obj, toe_ex_name)
                if toe_pb:
                    remove_constraint_if_exists(toe_pb, "FOLLOW_FK")
                    remove_constraint_if_exists(toe_pb, "FOLLOW_IK")

                    # 脚尖只复制旋转
                    fk_follow = toe_pb.constraints.new('COPY_ROTATION')
                    fk_follow.name = "FOLLOW_FK"
                    fk_follow.target = armature_obj
                    fk_follow.subtarget = FK_PREFIX + toe_ex_name
                    fk_follow.influence = 1.0

                    ik_follow = toe_pb.constraints.new('COPY_ROTATION')
                    ik_follow.name = "FOLLOW_IK"
                    ik_follow.target = armature_obj
                    ik_follow.subtarget = IK_PREFIX + toe_ex_name
                    ik_follow.influence = 0.0

                    add_ikfk_follow_drivers(armature_obj, toe_ex_name, prop_name)

        self._setup_extra_controllers(context, armature_obj)
        self._setup_eye_controllers(context, armature_obj)
        self._setup_finger_controllers(context, armature_obj)

        # 自动关闭 MMD 原生足部 IK，移除系统时恢复
        native_ik_count = set_mmd_native_leg_ik(armature_obj, False)
        # D 变形骨末端钉回本尊，防下蹲时脚部滑移
        _pin_d_bones(armature_obj)

        hide_all_internal_bones(armature_obj)
        # 显示全部控制器集合，隐藏其他无关集合
        hide_all_collections_except(armature_obj, {
            COLL_BODY, COLL_LIMB,
            COLL_ARM_FK, COLL_ARM_IK,
            COLL_LEG_FK, COLL_LEG_IK,
        })

        # 清理旧版统一集合（如果存在且为空）
        if hasattr(armature_obj.data, "collections"):
            legacy = armature_obj.data.collections.get(COLL_LEGACY_CTRL)
            if legacy and len(legacy.bones) == 0:
                armature_obj.data.collections.remove(legacy)

        msg = "系统构建完毕！"
        if native_ik_count:
            msg += f"（已自动关闭 {native_ik_count} 个 MMD 原生足 IK 约束）"
        self.report({'INFO'}, msg)
        return {'FINISHED'}

    def _setup_extra_controllers(self, context, armature_obj):
        armature = armature_obj.data
        bpy.ops.object.mode_set(mode='EDIT')

        created = []  
        for name in EXTRA_CONTROL_BONES:
            orig = armature.edit_bones.get(name)
            if not orig: continue
            ctrl_name = CTRL_PREFIX + name
            if ctrl_name in armature.edit_bones:
                created.append(name)
                continue
            cb = armature.edit_bones.new(ctrl_name)
            cb.head = orig.head.copy()
            cb.tail = orig.tail.copy()
            cb.roll = orig.roll
            cb.use_deform = False
            created.append(name)

        for name in created:
            orig = armature.edit_bones.get(name)
            cb = armature.edit_bones.get(CTRL_PREFIX + name)
            if not orig or not cb: continue
            parent = orig.parent
            if parent and (CTRL_PREFIX + parent.name) in armature.edit_bones:
                cb.parent = armature.edit_bones[CTRL_PREFIX + parent.name]
            else:
                cb.parent = parent

        bpy.ops.object.mode_set(mode='POSE')
        ctrl_shape = create_custom_shape("MMD_FK_Shape_Circle", 'CIRCLE')
        center_shape = create_custom_shape("MMD_FK_End_Shape", 'DOUBLE_CIRCLE')

        # === 修改区：总控大小配置 ===
        shape_scale = {
            "全ての親": 1.5,
            "センター": 0.8, "センター2": 0.8, "下半身": 1.6, "上半身": 1.5, "上半身2": 1.4,
            "首": 5.0, "頭": 1.8, "手捩.L": 0.85, "手捩.R": 0.85,
        }

        for name in created:
            ctrl_pb = get_pose_bone(armature_obj, CTRL_PREFIX + name)
            orig_pb = get_pose_bone(armature_obj, name)
            if not ctrl_pb or not orig_pb: continue

            if name in CENTER_CONTROL_BONES: ctrl_pb.custom_shape = center_shape
            else: ctrl_pb.custom_shape = ctrl_shape
            
            s = shape_scale.get(name, 1.0)
            ctrl_pb.custom_shape_scale_xyz = (s, s, s)
            
            # === 修改区：总控颜色与位移锁定配置 ===
            if name in ("手捩.L", "手捩.R"):
                ctrl_pb.color.palette = 'THEME04'
            elif name == "全ての親":
                ctrl_pb.color.palette = 'THEME01'  
            else:
                ctrl_pb.color.palette = 'THEME09' 

            shape_offset = {
                "センター": (0, -0.1, 0), "センター2": (0, -0.1, 0), "上半身2": (0, 0.05, 0),
                "上半身": (0, 0.05, 0), "下半身": (0, 0.05, 0), "頭": (0, 0.1, 0),
            }
            if name in shape_offset: ctrl_pb.custom_shape_translation = shape_offset[name]

            if name in ("手捩.L", "手捩.R"):
                ctrl_pb.rotation_mode = 'XYZ'
                ctrl_pb.lock_rotation = (True, False, True)

            if name not in CENTER_CONTROL_BONES:
                ctrl_pb.lock_location = (True, True, True)

            existing = orig_pb.constraints.get("FOLLOW_CTRL")
            if existing: orig_pb.constraints.remove(existing)
            con = orig_pb.constraints.new('COPY_TRANSFORMS')
            con.name = "FOLLOW_CTRL"
            con.target = armature_obj
            con.subtarget = CTRL_PREFIX + name
            con.influence = 1.0

            set_bone_layer_hidden(armature_obj, name)
            # 手捩属于手臂，归到四肢集合；其余总控、躯干、头颈归到身体集合
            ctrl_group = "limb" if name in ("手捩.L", "手捩.R") else "body"
            set_bone_layer_control(armature_obj, CTRL_PREFIX + name, group=ctrl_group)

    # ------------------------------------------------------------
    # 完美眼部注视系统修复版 (解决斗鸡眼问题)
    # ------------------------------------------------------------
    def _setup_eye_controllers(self, context, armature_obj):
        armature = armature_obj.data

        eye_l = armature.bones.get("目.L")
        eye_r = armature.bones.get("目.R")
        if not (eye_l and eye_r): return  

        bpy.ops.object.mode_set(mode='EDIT')
        ebs = armature.edit_bones

        el = ebs.get("目.L")
        er = ebs.get("目.R")
        if not (el and er):
            bpy.ops.object.mode_set(mode='POSE')
            return

        mid_head = (el.head + er.head) * 0.5
        eye_gap = (el.head - er.head).length

        # 1. 计算距离
        head_bone = ebs.get("頭")
        if head_bone:
            dist = max((head_bone.tail - head_bone.head).length * 1.5, eye_gap * 4.0, 0.2)
        else:
            dist = max(eye_gap * 4.0, 0.2)

        # 2. 提取正前方向量
        dir_l = (el.tail - el.head)
        if dir_l.length < 0.001: dir_l = Vector((0, -1, 0))
        dir_l = dir_l.normalized()

        dir_r = (er.tail - er.head)
        if dir_r.length < 0.001: dir_r = Vector((0, -1, 0))
        dir_r = dir_r.normalized()

        dir_main = (dir_l + dir_r).normalized()

        l_ctrl_head = el.head + dir_l * dist
        r_ctrl_head = er.head + dir_r * dist
        main_ctrl_head = mid_head + dir_main * dist

        bone_len = max(eye_gap * 0.5, 0.05)

        # 3A. 创建主控制器 (用户唯一可见的椭圆大框)
        main_name = CTRL_PREFIX + "両目"
        if main_name not in ebs:
            mb = ebs.new(main_name)
            mb.head = main_ctrl_head
            mb.tail = main_ctrl_head + dir_main * bone_len
            mb.roll = 0  
            mb.use_deform = False
            
            parent_name = CTRL_PREFIX + "頭"
            if parent_name in ebs: mb.parent = ebs[parent_name]
            elif head_bone: mb.parent = head_bone

        # 3B. 创建隐藏的左右平行子目标 (解决斗鸡眼)
        tgt_l_name = IK_TARGET_PREFIX + "目.L"
        tgt_r_name = IK_TARGET_PREFIX + "目.R"
        
        if tgt_l_name not in ebs:
            tl = ebs.new(tgt_l_name)
            tl.head = l_ctrl_head
            tl.tail = l_ctrl_head + dir_l * bone_len
            tl.use_deform = False
            tl.parent = ebs[main_name]  

        if tgt_r_name not in ebs:
            tr = ebs.new(tgt_r_name)
            tr.head = r_ctrl_head
            tr.tail = r_ctrl_head + dir_r * bone_len
            tr.use_deform = False
            tr.parent = ebs[main_name]  

        bpy.ops.object.mode_set(mode='POSE')

        # 4. 设置美化外观
        eyes_oval_shape = create_custom_shape("MMD_Eyes_Oval", 'EYES_OVAL')
        main_scale = max(eye_gap * 0.8, 0.08) 
        
        main_pb = get_pose_bone(armature_obj, main_name)
        if main_pb:
            main_pb.custom_shape = eyes_oval_shape
            main_pb.custom_shape_scale_xyz = (main_scale, main_scale, main_scale)
            main_pb.color.palette = 'THEME09'

        # 5. 生成精准追踪约束（分别追踪各自的隐藏平行目标）
        for orig_name, tgt_name in [("目.L", tgt_l_name), ("目.R", tgt_r_name)]:
            orig_pb = get_pose_bone(armature_obj, orig_name)
            if not orig_pb: continue
            
            for c in orig_pb.constraints:
                c.mute = True

            existing = orig_pb.constraints.get("EYE_TRACK")
            if existing: orig_pb.constraints.remove(existing)
                
            con = orig_pb.constraints.new('DAMPED_TRACK')
            con.name = "EYE_TRACK"
            con.target = armature_obj
            con.subtarget = tgt_name  
            con.track_axis = 'TRACK_Y'  
            con.influence = 1.0

            set_bone_layer_hidden(armature_obj, tgt_name)

        for n in ("両目",):
            if armature.bones.get(n): set_bone_layer_hidden(armature_obj, n)
        for n in ("目.L", "目.R"):
            b = armature.bones.get(n)
            if b: b.hide = True

        set_bone_layer_control(armature_obj, main_name, group="body")

    def _setup_finger_controllers(self, context, armature_obj):
        armature = armature_obj.data
        bpy.ops.object.mode_set(mode='EDIT')
        ebs = armature.edit_bones
        finger_info = {}

        for side in ("L", "R"):
            for finger_name, seg_names in FINGER_CHAINS.items():
                full_names = [f"{n}.{side}" for n in seg_names]
                if not all(n in ebs for n in full_names): continue

                # AUTO 层负责接受 BEND 批量驱动；CTRL 层负责用户手动多轴微调。
                # 层级：parent -> AUTO1 -> CTRL1 -> AUTO2 -> CTRL2 -> AUTO3 -> CTRL3
                for orig in full_names:
                    ob = ebs[orig]

                    aname = AUTO_PREFIX + orig
                    if aname not in ebs:
                        ab = ebs.new(aname)
                        ab.head = ob.head.copy()
                        ab.tail = ob.tail.copy()
                        ab.roll = ob.roll
                        ab.use_deform = False

                    cname = CTRL_PREFIX + orig
                    if cname not in ebs:
                        cb = ebs.new(cname)
                        cb.head = ob.head.copy()
                        cb.tail = ob.tail.copy()
                        cb.roll = ob.roll
                        cb.use_deform = False

                for i, orig in enumerate(full_names):
                    ab = ebs[AUTO_PREFIX + orig]
                    cb = ebs[CTRL_PREFIX + orig]
                    if i == 0:
                        ab.parent = ebs[orig].parent
                    else:
                        ab.parent = ebs[CTRL_PREFIX + full_names[i - 1]]
                    cb.parent = ab

                seg1 = ebs[full_names[0]]
                seg2 = ebs[full_names[1]]
                dir1 = (seg1.tail - seg1.head).normalized()
                dir2 = (seg2.tail - seg2.head).normalized()
                world_axis = dir1.cross(dir2)
                
                if world_axis.length < 0.01:
                    parent_bone = seg1.parent
                    if parent_bone:
                        parent_dir = (parent_bone.tail - parent_bone.head).normalized()
                        world_axis = dir1.cross(parent_dir)
                if world_axis.length < 0.01: world_axis = Vector((1, 0, 0))
                world_axis = world_axis.normalized()

                local_axis = seg1.matrix.to_3x3().inverted() @ world_axis
                abs_comp = [abs(local_axis.x), abs(local_axis.y), abs(local_axis.z)]
                axis_idx = abs_comp.index(max(abs_comp))
                axis_names = ['X', 'Y', 'Z']
                bend_axis = axis_names[axis_idx]
                bend_sign = 1 if local_axis[axis_idx] > 0 else -1

                bend_name = BEND_PREFIX + f"{finger_name}.{side}"
                root_bone = ebs[full_names[0]]
                tip_bone = ebs[full_names[-1]]
                finger_len = sum((ebs[n].tail - ebs[n].head).length for n in full_names)

                hand_end = "手首." + side
                hand_bone = ebs.get(hand_end)

                if bend_name not in ebs:
                    # BEND 沿各自手指根→尖方向放在指尖外侧（世界空间，不依赖手腕骨朝向）
                    finger_dir_world = tip_bone.tail - root_bone.head
                    fdl = finger_dir_world.length
                    if fdl > 1e-6:
                        finger_dir_world = finger_dir_world / fdl
                    else:
                        finger_dir_world = Vector((0, 1, 0))  # 极端兜底

                    offset_factor = 0.15 if finger_name == "親指" else 0.25
                    head_pos = tip_bone.tail + finger_dir_world * (finger_len * offset_factor)
                    bend_dir = finger_dir_world

                    bend_len = finger_len * 0.15
                    bb = ebs.new(bend_name)
                    bb.head = head_pos
                    bb.tail = head_pos + bend_dir * bend_len
                    bb.use_deform = False

                # BEND 挂在原始手腕上（原骨跟随当前 FK/IK 结果），局部旋转仍供驱动器读取
                bb = ebs.get(bend_name)
                if bb:
                    bb.use_connect = False
                    if hand_bone:
                        bb.parent = hand_bone
                    else:
                        bb.parent = root_bone.parent

                finger_info[(finger_name, side)] = {
                    "segs": full_names,
                    "bend_axis": bend_axis,
                    "bend_sign": bend_sign,
                }

        bpy.ops.object.mode_set(mode='POSE')
        ctrl_shape = create_custom_shape("MMD_FK_Shape_Circle", 'CIRCLE')
        bend_shape = create_custom_shape("MMD_IK_Shape_Sphere", 'SPHERE')  

        for (finger_name, side), info in finger_info.items():
            segs = info["segs"]
            bend_axis = info["bend_axis"]
            bend_sign = info["bend_sign"]
            bend_name = BEND_PREFIX + f"{finger_name}.{side}"

            for orig in segs:
                auto_pb = get_pose_bone(armature_obj, AUTO_PREFIX + orig)
                ctrl_pb = get_pose_bone(armature_obj, CTRL_PREFIX + orig)
                orig_pb = get_pose_bone(armature_obj, orig)
                if not (auto_pb and ctrl_pb and orig_pb):
                    continue

                auto_pb.rotation_mode = 'XYZ'
                auto_pb.lock_location = (True, True, True)
                auto_pb.lock_scale = (True, True, True)
                auto_pb.custom_shape = None

                ctrl_pb.rotation_mode = 'XYZ'
                ctrl_pb.custom_shape = ctrl_shape
                ctrl_pb.custom_shape_scale_xyz = (0.7, 0.7, 0.7)
                ctrl_pb.color.palette = 'THEME04'
                ctrl_pb.lock_location = (True, True, True)
                ctrl_pb.lock_scale = (True, True, True)
                remove_rotation_drivers(ctrl_pb)
                
                remove_constraint_if_exists(orig_pb, "FOLLOW_CTRL")
                con = orig_pb.constraints.new('COPY_TRANSFORMS')
                con.name = "FOLLOW_CTRL"
                con.target = armature_obj
                con.subtarget = CTRL_PREFIX + orig
                con.influence = 1.0

                set_bone_layer_hidden(armature_obj, orig)
                set_bone_layer_hidden(armature_obj, AUTO_PREFIX + orig)
                set_bone_layer_control(armature_obj, CTRL_PREFIX + orig)

            bend_pb = get_pose_bone(armature_obj, bend_name)
            if not bend_pb: continue
            bend_pb.custom_shape = bend_shape
            bend_pb.custom_shape_scale_xyz = (0.96, 0.96, 0.96)  
            bend_pb.color.palette = 'THEME11'  
            bend_pb.rotation_mode = 'XYZ'
            bend_pb.lock_location = (True, True, True)
            bend_pb.lock_rotation = (False, False, False)
            set_bone_layer_control(armature_obj, bend_name)

            bend_pb["_bend_axis"] = bend_axis
            bend_pb["_bend_sign"] = bend_sign

            for orig in segs:
                ap = get_pose_bone(armature_obj, AUTO_PREFIX + orig)
                cp = get_pose_bone(armature_obj, CTRL_PREFIX + orig)
                if ap: ap.rotation_mode = 'XYZ'
                if cp: cp.rotation_mode = 'XYZ'

        props = armature_obj.mmikfk_props
        for side in ("L", "R"):
            info = finger_info.get(("親指", side))
            if not info: continue
            if side == "L":
                props.thumb_bend_axis_l = info["bend_axis"]
                props.thumb_bend_sign_l = float(info["bend_sign"])
            else:
                props.thumb_bend_axis_r = info["bend_axis"]
                props.thumb_bend_sign_r = float(info["bend_sign"])

        _rebuild_finger_drivers(armature_obj)

class MMIKFK_OT_SnapIKtoFK(bpy.types.Operator):
    bl_idname = "mmikfk.snap_ik_to_fk"
    bl_label = "IK 对齐 FK"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS)

    def execute(self, context):
        armature_obj = context.active_object
        limb_data = MMD_BONE_MAP[self.limb]

        if limb_data.get("shoulder"):
            ik_sh = get_pose_bone(armature_obj, IK_PREFIX + limb_data["shoulder"])
            fk_sh = get_pose_bone(armature_obj, FK_PREFIX + limb_data["shoulder"])
            if ik_sh and fk_sh: ik_sh.matrix = fk_sh.matrix.copy()

        end_pb = get_pose_bone(armature_obj, limb_data["end"])
        ik_target = get_pose_bone(armature_obj, IK_TARGET_PREFIX + limb_data["end"])
        if end_pb and ik_target: ik_target.matrix = end_pb.matrix.copy()

        # 同步脚尖 IK 控制器到当前脚尖姿态
        toe_ex_name = limb_data.get("toe_ex")
        if toe_ex_name:
            toe_pb = get_pose_bone(armature_obj, toe_ex_name)
            ikt_toe = get_pose_bone(armature_obj, IK_TARGET_PREFIX + toe_ex_name)
            if toe_pb and ikt_toe:
                # 只对齐旋转，位置保持跟随脚踝
                ikt_toe.matrix = toe_pb.matrix.copy()
                ikt_toe.location = (0.0, 0.0, 0.0)

        upper_pb = get_pose_bone(armature_obj, limb_data["upper"])
        lower_pb = get_pose_bone(armature_obj, limb_data["lower"])
        end_pb = get_pose_bone(armature_obj, limb_data["end"])
        ik_pole = get_pose_bone(armature_obj, IK_POLE_PREFIX + limb_data["lower"])

        if upper_pb and lower_pb and end_pb and ik_pole:
            u_head = upper_pb.matrix.translation
            l_head = lower_pb.matrix.translation
            e_head = end_pb.matrix.translation

            offset = 0.4 if "arm" in self.limb else 0.5

            # 在当前 FK 弯曲平面内，沿肘/膝凸出的方向放置极向量
            # 这样 IK 解算器才能用 setup 阶段固定下来的 pole_angle 还原出与 FK 一致的肘/膝位置
            pole_dir = None
            chain = e_head - u_head
            chain_len = chain.length
            if chain_len > 1e-6:
                chain_n = chain / chain_len
                ul = l_head - u_head
                # 把 lower 偏离 upper→end 主轴的分量取出来——就是当前弯曲方向
                bend = ul - chain_n * ul.dot(chain_n)
                # 模型自带的固有弯（静止姿势就有的，比如被旧版预弯过）不算用户姿势：
                # 当前弯≈固有弯时交给下面的存储弯向兜底，否则校准结果会被固有弯覆盖
                arm_bones = armature_obj.data.bones
                ub = arm_bones[limb_data["upper"]].head_local
                lb = arm_bones[limb_data["lower"]].head_local
                ebn = arm_bones[limb_data["end"]].head_local
                rest_chain = ebn - ub
                rest_bend = Vector((0.0, 0.0, 0.0))
                if rest_chain.length > 1e-6:
                    rn = rest_chain.normalized()
                    rul = lb - ub
                    rest_bend = rul - rn * rul.dot(rn)
                if bend.length > 1e-5 and \
                        (bend - rest_bend).length > max(rest_bend.length * 0.5, 0.005):
                    pole_dir = bend.normalized()

            # 链条几乎笔直时弯曲方向不唯一，优先用生成/校准时记录的弯向兜底
            if pole_dir is None:
                stored = armature_obj.data.get("mmikfk_bend_dirs")
                if stored and self.limb in stored.keys():
                    pole_dir = Vector(stored[self.limb]).normalized()
                else:
                    sign = 1.0 if not armature_obj.mmikfk_props.prebend_invert else -1.0
                    pole_dir = Vector((0, sign, 0)) if "arm" in self.limb else Vector((0, -sign, 0))

            ik_pole.matrix = Matrix.Translation(l_head + pole_dir * offset)

        context.view_layer.update()
        return {'FINISHED'}

class MMIKFK_OT_SnapFKtoIK(bpy.types.Operator):
    bl_idname = "mmikfk.snap_fk_to_ik"
    bl_label = "FK 对齐 IK"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS)

    def execute(self, context):
        armature_obj = context.active_object
        limb_data = MMD_BONE_MAP[self.limb]
        twist_name = limb_data.get("twist")

        context.view_layer.update()

        names = []
        if limb_data.get("shoulder"): names.append(limb_data["shoulder"])
        names.append(limb_data["upper"])
        if twist_name: names.append(twist_name)
        names.append(limb_data["lower"])
        names.append(limb_data["end"])

        target_matrices = {}
        for name in names:
            if name == twist_name: continue
            orig_pb = get_pose_bone(armature_obj, name)
            if orig_pb: target_matrices[name] = orig_pb.matrix.copy()
            else:
                ik_pb = get_pose_bone(armature_obj, IK_PREFIX + name)
                if ik_pb: target_matrices[name] = ik_pb.matrix.copy()

        for name in names:
            fk_pb = get_pose_bone(armature_obj, FK_PREFIX + name)
            if not fk_pb: continue
            if name == twist_name:
                fk_pb.rotation_quaternion = (1, 0, 0, 0)
                fk_pb.rotation_euler = (0, 0, 0)
            elif name in target_matrices:
                fk_pb.matrix = target_matrices[name]
            context.view_layer.update()

        end_name = limb_data["end"]
        fk_end = get_pose_bone(armature_obj, FK_PREFIX + end_name)
        if fk_end and end_name in target_matrices:
            fk_end.matrix = target_matrices[end_name]
            context.view_layer.update()

        # 同步 FK 脚尖到当前脚尖姿态
        toe_ex_name = limb_data.get("toe_ex")
        if toe_ex_name:
            toe_pb = get_pose_bone(armature_obj, toe_ex_name)
            fk_toe = get_pose_bone(armature_obj, FK_PREFIX + toe_ex_name)
            if toe_pb and fk_toe:
                toe_matrix = toe_pb.matrix.copy()
                fk_toe.matrix = toe_matrix
                context.view_layer.update()

        return {'FINISHED'}

class MMIKFK_OT_SwitchToIK(bpy.types.Operator):
    bl_idname = "mmikfk.switch_to_ik"
    bl_label = "切换到 IK"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS)

    def execute(self, context):
        bpy.ops.mmikfk.snap_ik_to_fk(limb=self.limb)
        setattr(context.active_object.mmikfk_props, LIMB_TO_PROP[self.limb], 1.0)
        update_limb_visibility(context.active_object, self.limb)
        return {'FINISHED'}

class MMIKFK_OT_SwitchToFK(bpy.types.Operator):
    bl_idname = "mmikfk.switch_to_fk"
    bl_label = "切换到 FK"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS)

    def execute(self, context):
        bpy.ops.mmikfk.snap_fk_to_ik(limb=self.limb)
        setattr(context.active_object.mmikfk_props, LIMB_TO_PROP[self.limb], 0.0)
        update_limb_visibility(context.active_object, self.limb)
        return {'FINISHED'}

class MMIKFK_OT_ToggleIKFK(bpy.types.Operator):
    """IK/FK 一键切换：自动对齐当前姿势再换模式，不跳变"""
    bl_idname = "mmikfk.toggle_ikfk"
    bl_label = "一键切换"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS + [('ALL', "全部", "")])

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        props = context.active_object.mmikfk_props
        limbs = list(MMD_BONE_MAP.keys()) if self.limb == 'ALL' else [self.limb]
        for limb_key in limbs:
            is_ik = getattr(props, LIMB_TO_PROP[limb_key]) >= 0.5
            if is_ik:
                bpy.ops.mmikfk.switch_to_fk(limb=limb_key)
            else:
                bpy.ops.mmikfk.switch_to_ik(limb=limb_key)
        if self.limb == 'ALL':
            self.report({'INFO'}, "四肢已各自切到另一侧")
        return {'FINISHED'}


class MMIKFK_OT_KeyframeAll(bpy.types.Operator):
    bl_idname = "mmikfk.keyframe_all"
    bl_label = "K 帧当前模式"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS)

    def execute(self, context):
        armature_obj = context.active_object
        prop_name = LIMB_TO_PROP[self.limb]
        frame = context.scene.frame_current
        is_ik = getattr(armature_obj.mmikfk_props, prop_name) >= 0.5
        limb_data = MMD_BONE_MAP[self.limb]

        armature_obj.keyframe_insert(data_path=f'mmikfk_props.{prop_name}', frame=frame)

        if is_ik:
            ik_controllers = [IK_TARGET_PREFIX + limb_data["end"], IK_POLE_PREFIX + limb_data["lower"]]
            if limb_data.get("shoulder"): ik_controllers.append(IK_PREFIX + limb_data["shoulder"])
            for ik_name in ik_controllers:
                pb = get_pose_bone(armature_obj, ik_name)
                if pb:
                    pb.keyframe_insert(data_path="location", frame=frame)
                    pb.keyframe_insert(data_path="rotation_quaternion" if pb.rotation_mode == 'QUATERNION' else "rotation_euler", frame=frame)
            # 脚尖控制器只 K 旋转
            if limb_data.get("toe_ex"):
                pb = get_pose_bone(armature_obj, IK_TARGET_PREFIX + limb_data["toe_ex"])
                if pb:
                    pb.keyframe_insert(data_path="rotation_quaternion" if pb.rotation_mode == 'QUATERNION' else "rotation_euler", frame=frame)
        else:
            names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
            if limb_data.get("twist"): names.append(limb_data["twist"])
            if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])
            for name in names:
                pb = get_pose_bone(armature_obj, FK_PREFIX + name)
                if pb: pb.keyframe_insert(data_path="rotation_quaternion" if pb.rotation_mode == 'QUATERNION' else "rotation_euler", frame=frame)

        # FK 脚尖在 FK 模式下 K（IK 模式已在上面 K 了 IKT 脚尖）
        toe_ex_name = limb_data.get("toe_ex")
        if toe_ex_name and not is_ik:
            fk_toe = get_pose_bone(armature_obj, FK_PREFIX + toe_ex_name)
            if fk_toe: fk_toe.keyframe_insert(data_path="rotation_quaternion" if fk_toe.rotation_mode == 'QUATERNION' else "rotation_euler", frame=frame)

        self.report({'INFO'}, "已 K 帧")
        return {'FINISHED'}

class MMIKFK_OT_ResetAllPose(bpy.types.Operator):
    """姿势全部归零：控制器、原骨、BEND 一键回默认姿势"""
    bl_idname = "mmikfk.reset_all_pose"
    bl_label = "姿势全部归零"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        for pb in armature_obj.pose.bones:
            pb.location = (0.0, 0.0, 0.0)
            pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
            pb.rotation_euler = (0.0, 0.0, 0.0)
            pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
            pb.scale = (1.0, 1.0, 1.0)
        context.view_layer.update()
        self.report({'INFO'}, "全部归零，回到默认姿势")
        return {'FINISHED'}


class MMIKFK_OT_AbsorbPose(bpy.types.Operator):
    """把原骨上的姿势吸附到控制器（配合 mmd_tools 的 VPD 导入使用）。
先用 mmd_tools 导入 VPD（姿势会被约束压住、看起来没反应），
再点此按钮，姿势就转移到 FK/CTRL 控制器上，可继续编辑。
建议在控制器中立状态下使用；眼睛注视由控制器接管，VPD 的眼骨值不吸附"""
    bl_idname = "mmikfk.absorb_pose"
    bl_label = "吸附姿势到控制器"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        props = armature_obj.mmikfk_props
        if not any(b.name.startswith(FK_PREFIX) for b in armature.bones):
            self.report({'ERROR'}, "请先生成骨骼系统")
            return {'CANCELLED'}

        pairs, rot_only = _managed_pairs()

        # 临时静音 FOLLOW，让原骨显出 mmd_tools 导入的真实姿势
        # （含它自家约束的效果），再按视觉局部值采样——不是抄裸数值
        muted = []
        for orig_name, _c in pairs:
            pb = get_pose_bone(armature_obj, orig_name)
            if pb:
                for con in pb.constraints:
                    if con.name in ("FOLLOW_FK", "FOLLOW_IK", "FOLLOW_CTRL"):
                        muted.append((con, con.mute))
                        con.mute = True
        # VPD 的腿部姿势藏在 足ＩＫ 骨的位置里：临时恢复原生足 IK 参与解算
        set_mmd_native_leg_ik(armature_obj, True)
        context.view_layer.update()

        eps = 1e-5
        moved = 0
        try:
            # 先整体采样再写入：写入会归零原骨，边采边写会污染子骨的换算
            sampled = {}
            for orig_name, ctrl_name in pairs:
                orig_pb = get_pose_bone(armature_obj, orig_name)
                ctrl_pb = get_pose_bone(armature_obj, ctrl_name)
                if not (orig_pb and ctrl_pb):
                    continue
                m_local = armature_obj.convert_space(pose_bone=orig_pb, matrix=orig_pb.matrix,
                                                     from_space='POSE', to_space='LOCAL')
                loc, rot, _ = m_local.decompose()
                if loc.length < eps and abs(rot.w - 1.0) < eps and \
                        abs(rot.x) < eps and abs(rot.y) < eps and abs(rot.z) < eps:
                    continue
                sampled[orig_name] = (ctrl_name, loc, rot)

            for orig_name, (ctrl_name, loc, rot) in sampled.items():
                orig_pb = get_pose_bone(armature_obj, orig_name)
                ctrl_pb = get_pose_bone(armature_obj, ctrl_name)
                _apply_local(ctrl_pb, loc, rot, orig_name not in rot_only)
                # 原骨归零（它由约束驱动，残值只会捣乱）
                orig_pb.location = (0.0, 0.0, 0.0)
                orig_pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                orig_pb.rotation_euler = (0.0, 0.0, 0.0)
                moved += 1

            # 消费掉原生 IK 骨（足ＩＫ/つま先ＩＫ/足IK親）的残值：腿型已烘进 FK，
            # 留着会在导出时双重。肩P/上半身3 这类祖先骨的残值保留——
            # 它们不在采样的局部值里，留着才能继续贡献姿势
            all_prefixes = (FK_PREFIX, IK_PREFIX, IK_TARGET_PREFIX, IK_POLE_PREFIX,
                            CTRL_PREFIX, BEND_PREFIX, AUTO_PREFIX, BIND_PREFIX)
            for pb in armature_obj.pose.bones:
                nm = pb.name
                if nm.startswith(all_prefixes):
                    continue
                if "IK親" in nm or any(k in nm for k in MMD_NATIVE_IK_BONES):
                    pb.location = (0.0, 0.0, 0.0)
                    pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                    pb.rotation_euler = (0.0, 0.0, 0.0)
        finally:
            set_mmd_native_leg_ik(armature_obj, False)
            for con, m in muted:
                con.mute = m

        if not moved:
            self.report({'WARNING'}, "原骨上没有待吸附的姿势（先用 mmd_tools 导入 VPD）")
            return {'CANCELLED'}

        # 吸附值里已含 BEND 的贡献，归零防止双重叠加
        _reset_bend_bones(armature_obj)

        context.view_layer.update()

        # IK 模式的肢体：临时切 FK 对齐 IK 控制器再切回，姿势保持不变
        for limb_key, prop_name in LIMB_TO_PROP.items():
            if getattr(props, prop_name) >= 0.5:
                setattr(props, prop_name, 0.0)
                context.view_layer.update()
                bpy.ops.mmikfk.snap_ik_to_fk(limb=limb_key)
                setattr(props, prop_name, 1.0)
        context.view_layer.update()

        self.report({'INFO'}, f"已吸附 {moved} 根骨骼的姿势到控制器")
        return {'FINISHED'}


class MMIKFK_OT_WritebackPose(bpy.types.Operator):
    """把控制器摆好的当前姿势写回原骨数值（导出 VPD 前点一下）。
画面不会有任何变化；写回后直接用 mmd_tools 导出 VPD 即可"""
    bl_idname = "mmikfk.writeback_pose"
    bl_label = "回写姿势到原骨"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        if not any(b.name.startswith(FK_PREFIX) for b in armature.bones):
            self.report({'ERROR'}, "请先生成骨骼系统")
            return {'CANCELLED'}

        pairs, rot_only = _managed_pairs()

        context.view_layer.update()
        n = 0
        for name, _ctrl in pairs:
            pb = get_pose_bone(armature_obj, name)
            if not pb:
                continue
            # 当前评估结果（约束驱动的最终姿势）→ 本地空间数值
            m_local = armature_obj.convert_space(pose_bone=pb, matrix=pb.matrix,
                                                 from_space='POSE', to_space='LOCAL')
            loc, rot, _ = m_local.decompose()
            _apply_local(pb, loc, rot, name not in rot_only)
            n += 1

        # MMD 生态的腿型靠 足ＩＫ：把当前脚踝/脚尖位置反推写进原生 IK 骨，
        # 否则导出的姿势在 MMD 里会被引擎的足 IK 拉回直立
        context.view_layer.update()
        _writeback_native_ik(context, armature_obj)

        self.report({'INFO'}, f"已把当前姿势写回 {n} 根原骨，可以用 mmd_tools 导出 VPD 了")
        return {'FINISHED'}


class MMIKFK_OT_AbsorbMotion(bpy.types.Operator):
    """把 mmd_tools 导入的 VMD 动作逐帧烘焙到控制器上（腿部 IK 动作一并转换成 FK）。
「导入 VMD 动作」按钮会自动调用，一般不用手动点"""
    bl_idname = "mmikfk.absorb_motion"
    bl_label = "吸附动作到控制器"
    bl_options = {'REGISTER', 'UNDO'}
    frame_start: bpy.props.IntProperty(name="起始帧", default=1)
    frame_end: bpy.props.IntProperty(name="结束帧", default=250)

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        self.frame_start = context.scene.frame_start
        self.frame_end = context.scene.frame_end
        return self.execute(context)

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        props = armature_obj.mmikfk_props
        scene = context.scene
        if not any(b.name.startswith(FK_PREFIX) for b in armature.bones):
            self.report({'ERROR'}, "请先生成骨骼系统")
            return {'CANCELLED'}

        pairs, rot_only = _managed_pairs()
        # 逐帧采样会把 BEND 的贡献烘进 CTRL 曲线，先归零防叠加
        _reset_bend_bones(armature_obj)

        # 临时放开接管：静音 FOLLOW 约束、恢复原生足 IK，让 VMD 动作原生驱动模型
        muted = []
        for orig_name, _ in pairs:
            pb = get_pose_bone(armature_obj, orig_name)
            if not pb:
                continue
            for con in pb.constraints:
                if con.name in ("FOLLOW_FK", "FOLLOW_IK", "FOLLOW_CTRL"):
                    muted.append((con, con.mute))
                    con.mute = True
        set_mmd_native_leg_ik(armature_obj, True)

        cur = scene.frame_current
        baked = 0
        try:
            for f in range(self.frame_start, self.frame_end + 1):
                scene.frame_set(f)
                for orig_name, ctrl_name in pairs:
                    orig_pb = get_pose_bone(armature_obj, orig_name)
                    ctrl_pb = get_pose_bone(armature_obj, ctrl_name)
                    if not (orig_pb and ctrl_pb):
                        continue
                    m_local = armature_obj.convert_space(pose_bone=orig_pb, matrix=orig_pb.matrix,
                                                         from_space='POSE', to_space='LOCAL')
                    loc, rot, _ = m_local.decompose()
                    _apply_local(ctrl_pb, loc, rot, orig_name not in rot_only)
                    _key_pb(ctrl_pb, f)
                baked += 1
        finally:
            for con, m in muted:
                con.mute = m
            set_mmd_native_leg_ik(armature_obj, False)
            scene.frame_set(cur)

        # 已消化的原骨/原生IK骨曲线清掉，原骨归零
        _strip_bone_curves(armature_obj, [n for n, _ in pairs], also_native_ik=True)
        for orig_name, _ in pairs:
            pb = get_pose_bone(armature_obj, orig_name)
            if pb:
                pb.location = (0.0, 0.0, 0.0)
                pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                pb.rotation_euler = (0.0, 0.0, 0.0)

        # 动作烘在 FK 层上，四肢统一切 FK
        for prop_name in PROP_TO_LIMB:
            setattr(props, prop_name, 0.0)
        for limb_key in MMD_BONE_MAP:
            update_limb_visibility(armature_obj, limb_key)

        self.report({'INFO'}, f"动作已吸附到控制器：{baked} 帧（四肢已切 FK）")
        return {'FINISHED'}


class MMIKFK_OT_BakeMotionToOrig(bpy.types.Operator):
    """把控制器做的动画逐帧烘焙回原骨关键帧（导出 VMD 前用，画面不变）。
「导出 VMD 动作」按钮会自动调用，一般不用手动点"""
    bl_idname = "mmikfk.bake_motion"
    bl_label = "烘焙动画到原骨"
    bl_options = {'REGISTER', 'UNDO'}
    frame_start: bpy.props.IntProperty(name="起始帧", default=1)
    frame_end: bpy.props.IntProperty(name="结束帧", default=250)

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        self.frame_start = context.scene.frame_start
        self.frame_end = context.scene.frame_end
        return self.execute(context)

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        scene = context.scene
        if not any(b.name.startswith(FK_PREFIX) for b in armature.bones):
            self.report({'ERROR'}, "请先生成骨骼系统")
            return {'CANCELLED'}

        pairs, rot_only = _managed_pairs()
        cur = scene.frame_current
        try:
            for f in range(self.frame_start, self.frame_end + 1):
                scene.frame_set(f)
                for name, _ctrl in pairs:
                    pb = get_pose_bone(armature_obj, name)
                    if not pb:
                        continue
                    m_local = armature_obj.convert_space(pose_bone=pb, matrix=pb.matrix,
                                                         from_space='POSE', to_space='LOCAL')
                    loc, rot, _ = m_local.decompose()
                    _apply_local(pb, loc, rot, name not in rot_only)
                    _key_pb(pb, f)
                # 腿型同步写进原生 足ＩＫ/つま先ＩＫ（MMD 引擎按它算腿）
                _writeback_native_ik(context, armature_obj, frame=f)
        finally:
            scene.frame_set(cur)

        self.report({'INFO'}, f"已烘焙 {self.frame_end - self.frame_start + 1} 帧到原骨，可以导出 VMD 了")
        return {'FINISHED'}


# ── VPD / VMD 一条龙（内部调用 mmd_tools，不用来回切面板） ──

class MMIKFK_OT_ImportVPDFile(bpy.types.Operator):
    """导入 VPD 姿势并自动吸附到控制器（内部调用 mmd_tools）"""
    bl_idname = "mmikfk.import_vpd_file"
    bl_label = "导入 VPD 姿势"
    bl_options = {'REGISTER', 'UNDO'}
    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.vpd", options={'HIDDEN'})
    scale: FloatProperty(name="缩放", default=0.08, description="与模型导入时的缩放一致")

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not _mmd_tools_ready("import_vpd"):
            self.report({'ERROR'}, "没找到 mmd_tools 插件（VPD 解析靠它）")
            return {'CANCELLED'}
        armature_obj = context.active_object
        d, n = os.path.split(self.filepath)

        def run():
            kwargs = _filtered_op_kwargs(bpy.ops.mmd_tools.import_vpd, dict(
                files=[{"name": n}], directory=d,
                bone_mapper='PMX', scale=self.scale))
            return bpy.ops.mmd_tools.import_vpd(**kwargs)

        try:
            _with_mmd_selection(context, armature_obj, run)
        except Exception as e:
            self.report({'ERROR'}, f"mmd_tools 导入失败：{e}")
            return {'CANCELLED'}
        bpy.ops.mmikfk.absorb_pose()
        return {'FINISHED'}


class MMIKFK_OT_ExportVPDFile(bpy.types.Operator):
    """把当前姿势导出为 VPD（自动回写原骨后调用 mmd_tools）"""
    bl_idname = "mmikfk.export_vpd_file"
    bl_label = "导出 VPD 姿势"
    bl_options = {'REGISTER'}
    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.vpd", options={'HIDDEN'})
    scale: FloatProperty(name="缩放", default=12.5, description="0.08 导入的模型对应 12.5")

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = "pose.vpd"
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not _mmd_tools_ready("export_vpd"):
            self.report({'ERROR'}, "没找到 mmd_tools 插件（VPD 写出靠它）")
            return {'CANCELLED'}
        armature_obj = context.active_object
        r = bpy.ops.mmikfk.writeback_pose()
        if r != {'FINISHED'}:
            return {'CANCELLED'}
        fp = self.filepath if self.filepath.lower().endswith(".vpd") else self.filepath + ".vpd"

        def run():
            kwargs = _filtered_op_kwargs(bpy.ops.mmd_tools.export_vpd, dict(
                filepath=fp, scale=self.scale, pose_type='CURRENT'))
            return bpy.ops.mmd_tools.export_vpd(**kwargs)

        try:
            _with_mmd_selection(context, armature_obj, run)
        except Exception as e:
            self.report({'ERROR'}, f"mmd_tools 导出失败：{e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"VPD 已导出：{fp}")
        return {'FINISHED'}


class MMIKFK_OT_ImportVMDFile(bpy.types.Operator):
    """导入 VMD 动作并自动吸附到控制器（内部调用 mmd_tools；表情动画直接生效）"""
    bl_idname = "mmikfk.import_vmd_file"
    bl_label = "导入 VMD 动作"
    bl_options = {'REGISTER', 'UNDO'}
    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.vmd", options={'HIDDEN'})
    scale: FloatProperty(name="缩放", default=0.08, description="与模型导入时的缩放一致")
    update_scene: BoolProperty(name="按动作更新帧范围和帧率(30fps)", default=True)

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not _mmd_tools_ready("import_vmd"):
            self.report({'ERROR'}, "没找到 mmd_tools 插件（VMD 解析靠它）")
            return {'CANCELLED'}
        armature_obj = context.active_object

        def run():
            kwargs = _filtered_op_kwargs(bpy.ops.mmd_tools.import_vmd, dict(
                filepath=self.filepath, bone_mapper='PMX', scale=self.scale,
                margin=0, create_new_action=False,
                update_scene_settings=self.update_scene))
            return bpy.ops.mmd_tools.import_vmd(**kwargs)

        try:
            _with_mmd_selection(context, armature_obj, run)
        except Exception as e:
            self.report({'ERROR'}, f"mmd_tools 导入失败：{e}")
            return {'CANCELLED'}
        scene = context.scene
        return bpy.ops.mmikfk.absorb_motion(frame_start=scene.frame_start,
                                            frame_end=scene.frame_end)


class MMIKFK_OT_ExportVMDFile(bpy.types.Operator):
    """把控制器做的动画导出为 VMD（自动烘焙原骨后调用 mmd_tools；含表情动画）"""
    bl_idname = "mmikfk.export_vmd_file"
    bl_label = "导出 VMD 动作"
    bl_options = {'REGISTER'}
    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.vmd", options={'HIDDEN'})
    scale: FloatProperty(name="缩放", default=12.5, description="0.08 导入的模型对应 12.5")
    clean_after: BoolProperty(name="导出后清理原骨关键帧", default=True,
                              description="烘焙用的原骨关键帧导出后删掉，保持动作干净")

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = "motion.vmd"
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not _mmd_tools_ready("export_vmd"):
            self.report({'ERROR'}, "没找到 mmd_tools 插件（VMD 写出靠它）")
            return {'CANCELLED'}
        armature_obj = context.active_object
        scene = context.scene
        r = bpy.ops.mmikfk.bake_motion(frame_start=scene.frame_start,
                                       frame_end=scene.frame_end)
        if r != {'FINISHED'}:
            return {'CANCELLED'}
        fp = self.filepath if self.filepath.lower().endswith(".vmd") else self.filepath + ".vmd"

        def run():
            kwargs = _filtered_op_kwargs(bpy.ops.mmd_tools.export_vmd, dict(
                filepath=fp, scale=self.scale, use_frame_range=True))
            return bpy.ops.mmd_tools.export_vmd(**kwargs)

        try:
            _with_mmd_selection(context, armature_obj, run)
        except Exception as e:
            self.report({'ERROR'}, f"mmd_tools 导出失败：{e}")
            return {'CANCELLED'}
        if self.clean_after:
            pairs, _ = _managed_pairs()
            _strip_bone_curves(armature_obj, [n for n, _ in pairs])
            for name, _ctrl in pairs:
                pb = get_pose_bone(armature_obj, name)
                if pb:
                    pb.location = (0.0, 0.0, 0.0)
                    pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                    pb.rotation_euler = (0.0, 0.0, 0.0)
        self.report({'INFO'}, f"VMD 已导出：{fp}")
        return {'FINISHED'}


class MMIKFK_OT_AutoBendFK(bpy.types.Operator):
    """按自动判定的弯向，把 FK 的手肘/膝盖各折一个小角度。
弯向对不对一眼看穿：对了直接校准，不对就手动掰对再校准。不碰模型骨架"""
    bl_idname = "mmikfk.auto_bend_fk"
    bl_label = "自动弯一点 FK"
    bl_options = {'REGISTER', 'UNDO'}
    angle: FloatProperty(name="弯曲角度", default=15.0, min=2.0, max=60.0)

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        if not any(b.name.startswith(FK_PREFIX) for b in armature.bones):
            self.report({'ERROR'}, "请先生成骨骼系统")
            return {'CANCELLED'}

        stored = armature.get("mmikfk_bend_dirs")
        context.view_layer.update()
        bent = 0
        for limb_key, limb_data in MMD_BONE_MAP.items():
            fk_lower = get_pose_bone(armature_obj, FK_PREFIX + limb_data["lower"])
            fk_end = get_pose_bone(armature_obj, FK_PREFIX + limb_data["end"])
            if not (fk_lower and fk_end):
                continue
            if stored and limb_key in stored.keys():
                bend_dir = Vector(stored[limb_key]).normalized()
            else:
                bend_dir = Vector((0, 1, 0)) if "arm" in limb_key else Vector((0, -1, 0))

            pivot = fk_lower.matrix.translation.copy()
            forearm = fk_end.matrix.translation - pivot
            if forearm.length < 1e-6:
                continue
            # 让末端朝弯向的反方向折，肘/膝就凸向弯向
            axis = forearm.normalized().cross(-bend_dir)
            if axis.length < 1e-6:
                continue
            rot = Matrix.Rotation(radians(self.angle), 4, axis.normalized())
            fk_lower.matrix = (Matrix.Translation(pivot) @ rot @
                               Matrix.Translation(-pivot) @ fk_lower.matrix)
            context.view_layer.update()
            bent += 1

        self.report({'INFO'}, f"已给 {bent} 条肢体的 FK 折了 {self.angle:.0f}°，"
                              f"弯向对就点校准，不对就掰到对再校准")
        return {'FINISHED'}


class MMIKFK_OT_CalibrateBend(bpy.types.Operator):
    """自动弯向判错时用：先用 FK 把手肘/膝盖朝正确方向掰一点，
    再点此按钮，按当前摆出的弯曲平面重建 IK 弯向、极向量和 pole_angle。
    全程不修改模型骨架。"""
    bl_idname = "mmikfk.calibrate_bend"
    bl_label = "按当前姿势校准弯向"
    bl_options = {'REGISTER', 'UNDO'}
    limb: EnumProperty(items=LIMB_ITEMS + [('ALL', "全部", "")])
    reset_fk: BoolProperty(name="校准后控制器归位", default=True,
                           description="校准完把为演示方向掰出来的 FK 姿势自动清零")

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        armature_obj = context.active_object
        armature = armature_obj.data
        props = armature_obj.mmikfk_props

        limbs = list(MMD_BONE_MAP.keys()) if self.limb == 'ALL' else [self.limb]
        amount = props.prebend_amount if props.prebend_amount > 0.0 else 0.02

        bpy.ops.object.mode_set(mode='POSE')

        # 从当前姿势读每条肢体的弯曲平面
        new_dirs = {}
        skipped = []
        for limb_key in limbs:
            limb_data = MMD_BONE_MAP[limb_key]
            upper_pb = get_pose_bone(armature_obj, limb_data["upper"])
            lower_pb = get_pose_bone(armature_obj, limb_data["lower"])
            end_pb = get_pose_bone(armature_obj, limb_data["end"])
            ik_lower_b = armature.bones.get(IK_PREFIX + limb_data["lower"])
            if not (upper_pb and lower_pb and end_pb and ik_lower_b):
                skipped.append(limb_key)
                continue
            u = upper_pb.matrix.translation
            l = lower_pb.matrix.translation
            e = end_pb.matrix.translation
            chain = e - u
            if chain.length < 1e-6:
                skipped.append(limb_key)
                continue
            cn = chain / chain.length
            ul = l - u
            bend = ul - cn * ul.dot(cn)
            if bend.length < chain.length * 0.01:
                skipped.append(limb_key)
                continue
            new_dirs[limb_key] = bend.normalized()

        if not new_dirs:
            self.report({'WARNING'}, "没检测到弯曲：请旋转 FK_ひじ/FK_ひざ（手肘/膝盖那节），"
                                     "让关节折出角度再校准。转大臂/大腿没有用")
            return {'CANCELLED'}

        # 重弯 IK 层 + 重摆极向量。肘/膝先投影回肩→腕弦线再朝校准方向偏移：
        # 模型自带预弯（如被旧版物理预弯过）也会被校准方向完全接管，不被压制
        bend_dirs = dict(armature.get("mmikfk_bend_dirs", {}))
        bpy.ops.object.mode_set(mode='EDIT')
        ebs = armature.edit_bones
        for limb_key, bend_dir in new_dirs.items():
            limb_data = MMD_BONE_MAP[limb_key]
            orig_upper = ebs.get(limb_data["upper"])
            orig_lower = ebs.get(limb_data["lower"])
            orig_end = ebs.get(limb_data["end"])
            ik_upper = ebs.get(IK_PREFIX + limb_data["upper"])
            ik_lower = ebs.get(IK_PREFIX + limb_data["lower"])
            if not (orig_upper and orig_lower and orig_end and ik_upper and ik_lower):
                continue
            axis_v = orig_end.head - orig_upper.head
            ul = orig_lower.head - orig_upper.head
            if axis_v.length > 1e-9:
                an = axis_v.normalized()
                foot = orig_upper.head + an * ul.dot(an)
            else:
                foot = orig_lower.head.copy()
            own_bend = (orig_lower.head - foot).length
            amt = max(amount, own_bend)
            new_head = foot + bend_dir * amt
            ik_lower.head = new_head
            ik_upper.tail = new_head.copy()
            ikp = ebs.get(IK_POLE_PREFIX + limb_data["lower"])
            if ikp:
                offset = 0.4 if "arm" in limb_key else 0.5
                ikp.head = foot + bend_dir * offset
                ikp.tail = ikp.head + bend_dir * 0.06
            bend_dirs[limb_key] = list(bend_dir)

        # 重解 pole_angle
        bpy.ops.object.mode_set(mode='POSE')
        for limb_key in new_dirs:
            limb_data = MMD_BONE_MAP[limb_key]
            solve_pole_angle(context, armature_obj,
                             IK_PREFIX + limb_data["lower"],
                             IK_POLE_PREFIX + limb_data["lower"])

        armature["mmikfk_bend_dirs"] = bend_dirs

        # 校准完把演示用的 FK 姿势收走，控制器回归原位
        if self.reset_fk:
            for limb_key in new_dirs:
                limb_data = MMD_BONE_MAP[limb_key]
                names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
                if limb_data.get("twist"): names.append(limb_data["twist"])
                if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])
                if limb_data.get("toe_ex"): names.append(limb_data["toe_ex"])
                for n in names:
                    pb = get_pose_bone(armature_obj, FK_PREFIX + n)
                    if pb:
                        pb.location = (0.0, 0.0, 0.0)
                        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
                        pb.rotation_euler = (0.0, 0.0, 0.0)
            context.view_layer.update()

        msg = f"已按当前姿势校准 {len(new_dirs)} 条肢体的弯向"
        if skipped:
            msg += f"（{len(skipped)} 条太直或缺骨，跳过）"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class MMIKFK_OT_Cleanup(bpy.types.Operator):
    bl_idname = "mmikfk.cleanup"
    bl_label = "移除骨骼系统"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature_obj = context.active_object
        bpy.ops.object.mode_set(mode='POSE')

        for name in EXTRA_CONTROL_BONES:
            orig_pb = get_pose_bone(armature_obj, name)
            if orig_pb:
                con = orig_pb.constraints.get("FOLLOW_CTRL")
                if con: orig_pb.constraints.remove(con)
                orig_pb.bone.hide = False
                if hasattr(armature_obj.data, "collections"):
                    hidden_coll = armature_obj.data.collections.get(COLL_HIDDEN)
                    if hidden_coll and orig_pb.bone.name in hidden_coll.bones:
                        hidden_coll.unassign(orig_pb.bone)

        for name in ("目.L", "目.R", "両目"):
            orig_pb = get_pose_bone(armature_obj, name)
            if orig_pb:
                con = orig_pb.constraints.get("EYE_TRACK")
                if con: orig_pb.constraints.remove(con)
                
                for c in orig_pb.constraints:
                    c.mute = False
                    
                orig_pb.bone.hide = False
                if hasattr(armature_obj.data, "collections"):
                    hidden_coll = armature_obj.data.collections.get(COLL_HIDDEN)
                    if hidden_coll and orig_pb.bone.name in hidden_coll.bones:
                        hidden_coll.unassign(orig_pb.bone)

        for side in ("L", "R"):
            for finger_name, seg_names in FINGER_CHAINS.items():
                for seg in seg_names:
                    orig_name = f"{seg}.{side}"
                    orig_pb = get_pose_bone(armature_obj, orig_name)
                    if not orig_pb: continue
                    con = orig_pb.constraints.get("FOLLOW_CTRL")
                    if con: orig_pb.constraints.remove(con)
                    remove_rotation_drivers(get_pose_bone(armature_obj, CTRL_PREFIX + orig_name))
                    remove_rotation_drivers(get_pose_bone(armature_obj, AUTO_PREFIX + orig_name))
                    orig_pb.bone.hide = False
                    if hasattr(armature_obj.data, "collections"):
                        hidden_coll = armature_obj.data.collections.get(COLL_HIDDEN)
                        if hidden_coll and orig_pb.bone.name in hidden_coll.bones:
                            hidden_coll.unassign(orig_pb.bone)

        for limb_data in MMD_BONE_MAP.values():
            names = [limb_data["upper"], limb_data["lower"], limb_data["end"]]
            if limb_data.get("twist"): names.append(limb_data["twist"])
            if limb_data.get("shoulder"): names.insert(0, limb_data["shoulder"])
            if limb_data.get("toe_ex"): names.append(limb_data["toe_ex"])
            
            for name in names:
                pb = get_pose_bone(armature_obj, name)
                if pb:
                    pb.bone.hide = False
                    pb.custom_shape = None
                    pb.custom_shape_scale_xyz = (1, 1, 1)
                    for c_name in ["FOLLOW_FK", "FOLLOW_IK"]:
                        c = pb.constraints.get(c_name)
                        if c:
                            try:
                                c.driver_remove("influence")
                            except Exception:
                                pass
                            pb.constraints.remove(c)
                    
                    if hasattr(armature_obj.data, "collections"):
                        hidden_coll = armature_obj.data.collections.get(COLL_HIDDEN)
                        if hidden_coll and pb.bone.name in hidden_coll.bones:
                            hidden_coll.unassign(pb.bone)
                    else:
                        for i in range(32): pb.bone.layers[i] = (i == 0)

        bpy.ops.object.mode_set(mode='EDIT')
        for b in list(armature_obj.data.edit_bones):
            if any(b.name.startswith(p) for p in [FK_PREFIX, IK_PREFIX, IK_TARGET_PREFIX, IK_POLE_PREFIX, CTRL_PREFIX, AUTO_PREFIX, BEND_PREFIX, BIND_PREFIX]):
                armature_obj.data.edit_bones.remove(b)

        bpy.ops.object.mode_set(mode='POSE')

        if hasattr(armature_obj.data, "collections"):
            for coll in armature_obj.data.collections:
                if coll.name not in (COLL_HIDDEN, COLL_BODY, COLL_LIMB, COLL_LEGACY_CTRL):
                    coll.is_visible = True

        shape_names = [
            "MMD_FK_Shape_Circle", "MMD_FK_End_Shape", "MMD_FK_Hand_Shape", "MMD_FK_Arm_Shape",
            "MMD_Eye_Ring", "MMD_Eyes_Oval", "MMD_Eyes_Main", "MMD_Finger_Bend", 
            "MMD_IK_Arm_Target", "MMD_IK_Foot_Target",
            "MMD_IK_Shape_Sphere", "MMD_Shoulder_Shape", "MMD_Toe_Shape", "MMD_Empty_Shape",
            "MMD_IK_Shape_Box", "MMD_IK_Shape_Diamond", "MMD_IK_Shape_CrossArrow",
        ]
        for sname in shape_names:
            obj = bpy.data.objects.get(sname)
            if obj: bpy.data.objects.remove(obj, do_unlink=True)
        shape_coll = bpy.data.collections.get(SHAPE_COLL)
        if shape_coll and not shape_coll.objects:
            bpy.data.collections.remove(shape_coll)

        # 恢复 MMD 原生足部 IK，拆掉 D 骨钉子
        set_mmd_native_leg_ik(armature_obj, True)
        _unpin_d_bones(armature_obj)

        self.report({'INFO'}, "已移除系统，原骨骼已恢复正常显示")
        return {'FINISHED'}

# ============================================================
# ============================================================
# 姿态导入 / 导出
# ============================================================
POSE_EXPORT_PREFIXES = (
    FK_PREFIX,
    IK_TARGET_PREFIX,
    IK_POLE_PREFIX,
    CTRL_PREFIX,
    BEND_PREFIX,
)

POSE_PROP_NAMES = (
    "arm_l_ikfk", "arm_r_ikfk", "leg_l_ikfk", "leg_r_ikfk",
    "finger_bend_axis_l", "finger_bend_axis_r",
    "finger_bend_sign_l", "finger_bend_sign_r",
    "thumb_bend_axis_l", "thumb_bend_axis_r",
    "thumb_bend_sign_l", "thumb_bend_sign_r",
    "finger_spread_sign_l", "finger_spread_sign_r",
)


def _is_pose_export_bone(pb):
    """只导出用户真正会操作或会 K 帧的控制骨。AUTO_ 是驱动中间层，不导出。"""
    if not pb:
        return False
    name = pb.name
    if name.startswith(AUTO_PREFIX):
        return False
    if name.startswith(POSE_EXPORT_PREFIXES):
        return True
    return False


def _pose_bone_to_dict(pb):
    rot_mode = pb.rotation_mode
    data = {
        "rotation_mode": rot_mode,
        "location": list(pb.location),
        "scale": list(pb.scale),
    }
    if rot_mode == 'QUATERNION':
        data["rotation_quaternion"] = list(pb.rotation_quaternion)
    elif rot_mode == 'AXIS_ANGLE':
        data["rotation_axis_angle"] = list(pb.rotation_axis_angle)
    else:
        data["rotation_euler"] = list(pb.rotation_euler)
    return data


def _apply_pose_bone_dict(pb, data):
    if not pb or not isinstance(data, dict):
        return False

    rot_mode = data.get("rotation_mode", pb.rotation_mode)
    try:
        pb.rotation_mode = rot_mode
    except Exception:
        pass

    if "location" in data:
        try:
            pb.location = data["location"]
        except Exception:
            pass
    if "scale" in data:
        try:
            pb.scale = data["scale"]
        except Exception:
            pass

    try:
        if pb.rotation_mode == 'QUATERNION' and "rotation_quaternion" in data:
            pb.rotation_quaternion = data["rotation_quaternion"]
        elif pb.rotation_mode == 'AXIS_ANGLE' and "rotation_axis_angle" in data:
            pb.rotation_axis_angle = data["rotation_axis_angle"]
        elif "rotation_euler" in data:
            pb.rotation_euler = data["rotation_euler"]
        elif "rotation_quaternion" in data:
            pb.rotation_mode = 'QUATERNION'
            pb.rotation_quaternion = data["rotation_quaternion"]
    except Exception:
        pass
    return True


class MMIKFK_OT_ExportPose(bpy.types.Operator):
    bl_idname = "mmikfk.export_pose"
    bl_label = "导出姿态"
    bl_options = {'REGISTER'}

    filepath: StringProperty(
        name="保存姿态文件",
        subtype='FILE_PATH',
        default="mmd_pose.json",
    )

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        armature_obj = context.active_object
        props = armature_obj.mmikfk_props

        pose_data = {
            "format": "MMIKFK_POSE_JSON",
            "version": 1,
            "armature_name": armature_obj.name,
            "addon_version": list(bl_info.get("version", (0, 0, 0))),
            "properties": {},
            "bones": {},
        }

        for prop_name in POSE_PROP_NAMES:
            if hasattr(props, prop_name):
                pose_data["properties"][prop_name] = getattr(props, prop_name)

        for pb in armature_obj.pose.bones:
            if _is_pose_export_bone(pb):
                pose_data["bones"][pb.name] = _pose_bone_to_dict(pb)

        try:
            with open(self.filepath, 'w', encoding='utf-8') as f:
                json.dump(pose_data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.report({'ERROR'}, f"导出失败: {e}")
            return {'CANCELLED'}

        self.report({'INFO'}, f"已导出姿态：{len(pose_data['bones'])} 根控制骨")
        return {'FINISHED'}


class MMIKFK_OT_ImportPose(bpy.types.Operator):
    bl_idname = "mmikfk.import_pose"
    bl_label = "导入姿态"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: StringProperty(
        name="读取姿态文件",
        subtype='FILE_PATH',
        default="mmd_pose.json",
    )

    import_properties: BoolProperty(
        name="导入 IK/FK 与手指参数",
        description="同时恢复 IK/FK 开关、手指弯曲轴、张开量等插件属性",
        default=True,
    )

    keyframe_imported: BoolProperty(
        name="导入后自动 K 帧",
        description="在当前帧给导入的控制骨和插件属性插入关键帧",
        default=False,
    )

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "import_properties")
        layout.prop(self, "keyframe_imported")

    def execute(self, context):
        armature_obj = context.active_object
        props = armature_obj.mmikfk_props

        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                pose_data = json.load(f)
        except Exception as e:
            self.report({'ERROR'}, f"导入失败: {e}")
            return {'CANCELLED'}

        if pose_data.get("format") != "MMIKFK_POSE_JSON":
            self.report({'ERROR'}, "这不是 MMIKFK 姿态 JSON 文件")
            return {'CANCELLED'}

        imported_props = 0
        if self.import_properties:
            for prop_name, value in pose_data.get("properties", {}).items():
                if hasattr(props, prop_name):
                    try:
                        setattr(props, prop_name, value)
                        imported_props += 1
                    except Exception:
                        pass

        context.view_layer.update()

        imported_bones = 0
        missing_bones = []
        for bone_name, bone_data in pose_data.get("bones", {}).items():
            pb = armature_obj.pose.bones.get(bone_name)
            if not pb:
                missing_bones.append(bone_name)
                continue
            if _apply_pose_bone_dict(pb, bone_data):
                imported_bones += 1
                if self.keyframe_imported:
                    frame = context.scene.frame_current
                    try:
                        pb.keyframe_insert(data_path="location", frame=frame)
                        if pb.rotation_mode == 'QUATERNION':
                            pb.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                        elif pb.rotation_mode == 'AXIS_ANGLE':
                            pb.keyframe_insert(data_path="rotation_axis_angle", frame=frame)
                        else:
                            pb.keyframe_insert(data_path="rotation_euler", frame=frame)
                        pb.keyframe_insert(data_path="scale", frame=frame)
                    except Exception:
                        pass

        for limb_key in MMD_BONE_MAP.keys():
            try:
                update_limb_visibility(armature_obj, limb_key)
            except Exception:
                pass

        if self.keyframe_imported and self.import_properties:
            frame = context.scene.frame_current
            for prop_name in pose_data.get("properties", {}).keys():
                if hasattr(props, prop_name):
                    try:
                        armature_obj.keyframe_insert(data_path=f"mmikfk_props.{prop_name}", frame=frame)
                    except Exception:
                        pass

        context.view_layer.update()

        msg = f"已导入姿态：{imported_bones} 根控制骨"
        if imported_props:
            msg += f"，{imported_props} 个参数"
        if missing_bones:
            msg += f"；缺失 {len(missing_bones)} 根骨骼"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


# 插件偏好：模块显示开关
# ============================================================
MMIKFK_SECTIONS = (
    ("show_pose_tools", "姿态工具", True),
    ("show_prebend", "预弯曲", True),
    ("show_limbs", "四肢 IK/FK", True),
    ("show_finger_bend", "手指弯曲方向", False),
    ("show_facepanel", "表情面板", False),
)


def _section_enabled(context, key):
    addon = context.preferences.addons.get(__name__)
    if addon:
        return getattr(addon.preferences, key, True)
    defaults = {k: d for k, _l, d in MMIKFK_SECTIONS}
    return defaults.get(key, True)


class MMIKFK_Preferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    def draw(self, context):
        col = self.layout.column(align=True)
        col.label(text="显示哪些模块")
        row = col.grid_flow(columns=3, align=True)
        for key, label, _d in MMIKFK_SECTIONS:
            row.prop(self, key, text=label)


for _key, _label, _default in MMIKFK_SECTIONS:
    MMIKFK_Preferences.__annotations__[_key] = bpy.props.BoolProperty(name=_label, default=_default)


# ============================================================
# UI 面板
# ============================================================
class MMIKFK_PT_MainPanel(bpy.types.Panel):
    bl_label = "MMD IK/FK 切换"
    bl_idname = "MMIKFK_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "MMD IK/FK"

    @classmethod
    def poll(cls, context): 
        return context.active_object and context.active_object.type == 'ARMATURE'

    def draw(self, context):
        layout = self.layout
        props = context.active_object.mmikfk_props

        box = layout.box()
        row = box.row(align=True)
        row.operator("mmikfk.setup", icon='ADD')
        row.operator("mmikfk.cleanup", icon='TRASH')
        # 姿态工具：VPD/VMD 一条龙（吸附/回写是内部工序，要手动用的话 F3 搜）
        if _section_enabled(context, "show_pose_tools"):
            col = box.column(align=True)
            row = col.row(align=True)
            row.operator("mmikfk.import_vpd_file", icon='POSE_HLT')
            row.operator("mmikfk.export_vpd_file", text="导出 VPD")
            row = col.row(align=True)
            row.operator("mmikfk.import_vmd_file", icon='ANIM')
            row.operator("mmikfk.export_vmd_file", text="导出 VMD")
            col.operator("mmikfk.reset_all_pose", icon='LOOP_BACK')

        # === 控制器集合可见性切换 ===
        armature_obj = context.active_object
        ensure_limb_controller_collections(armature_obj)
        arm_data = armature_obj.data
        if hasattr(arm_data, "collections"):
            body_coll = arm_data.collections.get(COLL_BODY)
            limb_coll = arm_data.collections.get(COLL_LIMB)
            arm_fk_coll = arm_data.collections.get(COLL_ARM_FK)
            arm_ik_coll = arm_data.collections.get(COLL_ARM_IK)
            leg_fk_coll = arm_data.collections.get(COLL_LEG_FK)
            leg_ik_coll = arm_data.collections.get(COLL_LEG_IK)
            if any((body_coll, limb_coll, arm_fk_coll, arm_ik_coll,
                    leg_fk_coll, leg_ik_coll)):
                vis_box = layout.box()
                vis_box.label(text="控制器显示", icon='HIDE_OFF')
                vis_row = vis_box.row(align=True)
                vis_row.scale_y = 1.1
                if body_coll:
                    vis_row.prop(body_coll, "is_visible",
                                 text="身体",
                                 icon='HIDE_OFF' if body_coll.is_visible else 'HIDE_ON',
                                 toggle=True)
                if limb_coll:
                    vis_row.prop(limb_coll, "is_visible",
                                 text="手指/手捩",
                                 icon='HIDE_OFF' if limb_coll.is_visible else 'HIDE_ON',
                                 toggle=True)

                vis_row = vis_box.row(align=True)
                vis_row.scale_y = 1.1
                if arm_fk_coll:
                    vis_row.prop(arm_fk_coll, "is_visible",
                                 text="手 FK",
                                 icon='HIDE_OFF' if arm_fk_coll.is_visible else 'HIDE_ON',
                                 toggle=True)
                if arm_ik_coll:
                    vis_row.prop(arm_ik_coll, "is_visible",
                                 text="手 IK",
                                 icon='HIDE_OFF' if arm_ik_coll.is_visible else 'HIDE_ON',
                                 toggle=True)

                vis_row = vis_box.row(align=True)
                vis_row.scale_y = 1.1
                if leg_fk_coll:
                    vis_row.prop(leg_fk_coll, "is_visible",
                                 text="脚 FK",
                                 icon='HIDE_OFF' if leg_fk_coll.is_visible else 'HIDE_ON',
                                 toggle=True)
                if leg_ik_coll:
                    vis_row.prop(leg_ik_coll, "is_visible",
                                 text="脚 IK",
                                 icon='HIDE_OFF' if leg_ik_coll.is_visible else 'HIDE_ON',
                                 toggle=True)

        # === 预弯曲折叠区 ===
        if _section_enabled(context, "show_prebend"):
            box = layout.box()
            is_open_prebend = props.ui_show_prebend
            row = box.row(align=True)
            icon = 'TRIA_DOWN' if is_open_prebend else 'TRIA_RIGHT'
            row.prop(props, "ui_show_prebend", icon=icon,
                     text="预弯曲（修正 IK 极向量方向）", emboss=False)
        else:
            is_open_prebend = False

        if is_open_prebend:
            col = box.column(align=True)
            col.label(text="生成时按模型手指/脚尖自动判弯向；", icon='INFO')
            col.label(text="不放心就先自动弯 FK 看一眼，再校准。")
            col.separator()
            col.operator("mmikfk.auto_bend_fk", icon='BONE_DATA')
            row = col.row(align=True)
            row.operator("mmikfk.calibrate_bend", text="左腕").limb = 'arm_L'
            row.operator("mmikfk.calibrate_bend", text="右腕").limb = 'arm_R'
            row.operator("mmikfk.calibrate_bend", text="左足").limb = 'leg_L'
            row.operator("mmikfk.calibrate_bend", text="右足").limb = 'leg_R'
            col.operator("mmikfk.calibrate_bend", text="校准全部", icon='CON_KINEMATIC').limb = 'ALL'
            col.separator()
            col.prop(props, "prebend_amount", slider=True)
            col.prop(props, "prebend_invert")

        if _section_enabled(context, "show_limbs"):
            box = layout.box()
            is_open_limbs = props.ui_show_limbs
            row = box.row(align=True)
            icon = 'TRIA_DOWN' if is_open_limbs else 'TRIA_RIGHT'
            row.prop(props, "ui_show_limbs", icon=icon, text="四肢 IK/FK 切换", emboss=False)
        else:
            is_open_limbs = False

        if is_open_limbs:
            row = box.row()
            row.scale_y = 1.15
            row.operator("mmikfk.toggle_ikfk", text="一键全部切换", icon='ARROW_LEFTRIGHT').limb = 'ALL'
            # 每肢一行：滑条（拖=渐变，看得出当前模式）+ 切换 + K 帧
            col = box.column(align=True)
            for label, prop_name, limb_key in (
                    ("左腕", "arm_l_ikfk", "arm_L"),
                    ("右腕", "arm_r_ikfk", "arm_R"),
                    ("左足", "leg_l_ikfk", "leg_L"),
                    ("右足", "leg_r_ikfk", "leg_R")):
                row = col.row(align=True)
                row.prop(props, prop_name, slider=True,
                         text=f"{label} {'IK' if getattr(props, prop_name) >= 0.5 else 'FK'}")
                row.operator("mmikfk.toggle_ikfk", text="", icon='ARROW_LEFTRIGHT').limb = limb_key
                row.operator("mmikfk.keyframe_all", text="", icon='KEYTYPE_KEYFRAME_VEC').limb = limb_key

        if _section_enabled(context, "show_finger_bend"):
            box = layout.box()
            is_open_bend = props.ui_show_finger_bend
            row = box.row(align=True)
            icon = 'TRIA_DOWN' if is_open_bend else 'TRIA_RIGHT'
            row.prop(props, "ui_show_finger_bend", icon=icon, text="手指弯曲方向", emboss=False)
        else:
            is_open_bend = False

        if is_open_bend:
            box.separator()
            box.label(text="专业层级：BEND 驱动 AUTO，CTRL 保留手动多轴微调", icon='INFO')
            box.label(text="动画时可先用 BEND 做整体弯曲，再旋转单节 CTRL 修手型")
            
            sub = box.column(align=True)
            sub.label(text="左手四指")
            row = sub.row(align=True)
            row.prop(props, "finger_bend_axis_l", expand=True)
            sub.prop(props, "finger_bend_sign_l", text="方向", slider=True)

            sub = box.column(align=True)
            sub.label(text="左拇指")
            row = sub.row(align=True)
            row.prop(props, "thumb_bend_axis_l", expand=True)
            sub.prop(props, "thumb_bend_sign_l", text="方向", slider=True)

            sub = box.column(align=True)
            sub.label(text="右手四指")
            row = sub.row(align=True)
            row.prop(props, "finger_bend_axis_r", expand=True)
            sub.prop(props, "finger_bend_sign_r", text="方向", slider=True)

            sub = box.column(align=True)
            sub.label(text="右拇指")
            row = sub.row(align=True)
            row.prop(props, "thumb_bend_axis_r", expand=True)
            sub.prop(props, "thumb_bend_sign_r", text="方向", slider=True)

            sub = box.column(align=True)
            sub.label(text="四指张开：选中 BEND 球按 S 缩放", icon='FULLSCREEN_ENTER')
            sub.prop(props, "finger_spread_sign_l", text="左手张开方向", slider=True)
            sub.prop(props, "finger_spread_sign_r", text="右手张开方向", slider=True)

# ============================================================
# ============================================================
#                  MMD 表情面板模块 (FacePanel)
# ============================================================
# 在 IKFK 基础上集成；面板根 Anchor 通过 Bone Parent 挂到
# Armature 的「全ての親」骨骼 -> 角色位移/旋转/缩放时面板跟随。
# 所有内部函数加 _fp 后缀避免与 IKFK 命名冲突。
# ============================================================

FP_PANEL_NAME = "FacePanel"
FP_BOX_SIZE = 0.1
FP_BOX_SPACING = 0.3
FP_CTRL_SIZE = 0.015

# 用于面板挂载的根骨骼优先级（找到第一个就用）
FP_ROOT_BONE_CANDIDATES = ["全ての親", "センター", "上半身", "頭"]

FP_EYE_MAP = {
    "up":    "びっくり",
    "down":  "まばたき",
    "left":  "ウィンク２",
    "right": "ウィンク２右",
}

# 眉毛联动 - 一个方向可对应多个形态键
FP_BROW_LINK = {
    "up":    ["上"],
    "down":  ["下"],
    "left":  ["下左"],
    "right": ["下右"],
}

FP_VOWEL_ANCHORS = {
    "あ": (0.0,  1.0),
    "い": (0.95, 0.31),
    "う": (0.59, -0.81),
    "え": (-0.59, -0.81),
    "お": (-0.95, 0.31),
}


# ------------------------------------------------------------
# 查找工具
# ------------------------------------------------------------

def find_mesh_fp():
    """查找带形态键的 Mesh（优先 active）"""
    obj = bpy.context.active_object
    if obj and obj.type == 'MESH' and obj.data.shape_keys:
        return obj
    # 优先具有日文形态键名特征的 Mesh
    for o in bpy.context.scene.objects:
        if o.type == 'MESH' and o.data.shape_keys:
            keys = {kb.name for kb in o.data.shape_keys.key_blocks}
            if "あ" in keys or "まばたき" in keys:
                return o
    # 退化：任意带形态键的
    for o in bpy.context.scene.objects:
        if o.type == 'MESH' and o.data.shape_keys:
            return o
    return None


def find_armature_fp(mesh=None):
    """
    查找角色 Armature。优先级：
    1. mesh 的 Armature 修改器目标
    2. mesh 的 parent 链
    3. 场景里第一个 Armature
    """
    if mesh:
        for mod in mesh.modifiers:
            if mod.type == 'ARMATURE' and mod.object:
                return mod.object
        p = mesh.parent
        while p:
            if p.type == 'ARMATURE':
                return p
            p = p.parent
    for o in bpy.context.scene.objects:
        if o.type == 'ARMATURE':
            return o
    return None


def find_root_bone_fp(armature):
    """在 Armature 中查找适合挂载的根骨骼"""
    if not armature or armature.type != 'ARMATURE':
        return None
    for name in FP_ROOT_BONE_CANDIDATES:
        if name in armature.data.bones:
            return name
    return None


def get_keys_fp(mesh):
    if not mesh.data.shape_keys:
        return set()
    return {kb.name for kb in mesh.data.shape_keys.key_blocks if kb.name != "Basis"}


def clear_panel_and_drivers_fp(mesh):
    """删除所有面板对象 + 移除形态键 driver"""
    for o in [x for x in bpy.data.objects if x.name.startswith(FP_PANEL_NAME + "_")]:
        bpy.data.objects.remove(o, do_unlink=True)
    if mesh and mesh.data.shape_keys:
        for kb in mesh.data.shape_keys.key_blocks:
            try:
                kb.driver_remove("value")
            except:
                pass


def reset_all_shape_keys_fp(mesh):
    if not mesh or not mesh.data.shape_keys:
        return 0
    count = 0
    for kb in mesh.data.shape_keys.key_blocks:
        if kb.name != "Basis":
            try:
                kb.driver_remove("value")
            except:
                pass
            kb.value = 0.0
            count += 1
    return count


# ------------------------------------------------------------
# 创建辅助物体
# ------------------------------------------------------------

def make_empty_fp(name, parent, local_pos, dtype='SPHERE', size=0.08,
                  rotation=(0, 0, 0)):
    e = bpy.data.objects.new(f"{FP_PANEL_NAME}_{name}", None)
    e.empty_display_type = dtype
    e.empty_display_size = size
    e.show_in_front = True
    bpy.context.collection.objects.link(e)
    if parent:
        e.parent = parent
        e.matrix_parent_inverse.identity()
    e.location = local_pos
    e.rotation_euler = rotation
    return e


def make_square_outline_fp(name, parent, local_pos, size):
    mesh_data = bpy.data.meshes.new(f"{FP_PANEL_NAME}_{name}_mesh")
    verts = [
        (-size, 0, -size), (size, 0, -size),
        (size, 0, size), (-size, 0, size),
    ]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    mesh_data.from_pydata(verts, edges, [])
    mesh_data.update()
    obj = bpy.data.objects.new(f"{FP_PANEL_NAME}_{name}", mesh_data)
    obj.show_in_front = True
    obj.display_type = 'WIRE'
    bpy.context.collection.objects.link(obj)
    if parent:
        obj.parent = parent
        obj.matrix_parent_inverse.identity()
    obj.location = local_pos
    return obj


def add_limit_square_fp(ctrl, size):
    con = ctrl.constraints.new('LIMIT_LOCATION')
    con.use_min_x = con.use_max_x = True
    con.use_min_y = con.use_max_y = True
    con.use_min_z = con.use_max_z = True
    con.min_x, con.max_x = -size, size
    con.min_y = con.max_y = 0
    con.min_z, con.max_z = -size, size
    con.owner_space = 'LOCAL'


def add_driver_simple_fp(mesh, key_name, ctrl, axis, axis_range, invert=False):
    kb = mesh.data.shape_keys.key_blocks.get(key_name)
    if not kb:
        return False
    try:
        kb.driver_remove("value")
    except:
        pass
    fc = kb.driver_add("value")
    d = fc.driver
    d.type = 'SCRIPTED'
    v = d.variables.new()
    v.name = "p"
    v.type = 'TRANSFORMS'
    v.targets[0].id = ctrl
    v.targets[0].transform_type = axis
    v.targets[0].transform_space = 'LOCAL_SPACE'
    sign = "-" if invert else ""
    d.expression = f"max(0.0, min(1.0, {sign}p / {axis_range}))"
    return True


def add_driver_radial_fp(mesh, key_name, ctrl, anchor_x, anchor_z, box_size):
    kb = mesh.data.shape_keys.key_blocks.get(key_name)
    if not kb:
        return False
    try:
        kb.driver_remove("value")
    except:
        pass
    fc = kb.driver_add("value")
    d = fc.driver
    d.type = 'SCRIPTED'

    vx = d.variables.new()
    vx.name = "x"
    vx.type = 'TRANSFORMS'
    vx.targets[0].id = ctrl
    vx.targets[0].transform_type = 'LOC_X'
    vx.targets[0].transform_space = 'LOCAL_SPACE'

    vz = d.variables.new()
    vz.name = "z"
    vz.type = 'TRANSFORMS'
    vz.targets[0].id = ctrl
    vz.targets[0].transform_type = 'LOC_Z'
    vz.targets[0].transform_space = 'LOCAL_SPACE'

    ax = anchor_x * box_size
    az = anchor_z * box_size
    norm = box_size * 1.2
    dead_zone = box_size * 0.15
    d.expression = (
        f"max(0.0, min(1.0, 1.0 - sqrt((x-({ax}))**2 + (z-({az}))**2) / {norm})) "
        f"* max(0.0, min(1.0, (sqrt(x*x + z*z) - {dead_zone}) / {dead_zone}))"
    )
    return True


# ------------------------------------------------------------
# 眼睛十字锁 handler
# ------------------------------------------------------------

def eye_cross_lock_handler_fp(scene, depsgraph=None):
    """每帧检查眼睛控制器：X 和 Z 都非零时把绝对值较小的归零"""
    ctrl = bpy.data.objects.get(f"{FP_PANEL_NAME}_EyeCtrl")
    if not ctrl:
        return
    x = ctrl.location.x
    z = ctrl.location.z
    eps = 1e-5
    if abs(x) > eps and abs(z) > eps:
        if abs(x) >= abs(z):
            ctrl.location.z = 0.0
        else:
            ctrl.location.x = 0.0


def register_cross_lock_fp():
    unregister_cross_lock_fp()
    bpy.app.handlers.depsgraph_update_post.append(eye_cross_lock_handler_fp)


def unregister_cross_lock_fp():
    handlers = bpy.app.handlers.depsgraph_update_post
    to_remove = [h for h in handlers if h.__name__ == "eye_cross_lock_handler_fp"]
    for h in to_remove:
        handlers.remove(h)


# ------------------------------------------------------------
# 构建面板
# ------------------------------------------------------------

def build_eye_box_fp(mesh, parent_anchor, local_pos, available):
    """构建眼睛十字面板。parent_anchor 为根 Anchor (跟随骨骼)"""
    anchor = make_empty_fp("EyeAnchor", parent_anchor, local_pos,
                           'PLAIN_AXES', 0.05)
    make_square_outline_fp("EyeFrame", anchor, (0, 0, 0), FP_BOX_SIZE)

    # 十字辅助线
    cross_mesh = bpy.data.meshes.new(f"{FP_PANEL_NAME}_EyeCross_mesh")
    cross_verts = [
        (-FP_BOX_SIZE, 0, 0), (FP_BOX_SIZE, 0, 0),
        (0, 0, -FP_BOX_SIZE), (0, 0, FP_BOX_SIZE),
    ]
    cross_mesh.from_pydata(cross_verts, [(0, 1), (2, 3)], [])
    cross_obj = bpy.data.objects.new(f"{FP_PANEL_NAME}_EyeCross", cross_mesh)
    cross_obj.display_type = 'WIRE'
    cross_obj.show_in_front = True
    bpy.context.collection.objects.link(cross_obj)
    cross_obj.parent = anchor
    cross_obj.matrix_parent_inverse.identity()

    ctrl = make_empty_fp("EyeCtrl", anchor, (0, 0, 0), 'CUBE', FP_CTRL_SIZE)
    add_limit_square_fp(ctrl, FP_BOX_SIZE)

    eye_count = 0
    for direction, key in FP_EYE_MAP.items():
        if key not in available:
            continue
        axis = 'LOC_Z' if direction in ("up", "down") else 'LOC_X'
        invert = direction in ("down", "left")
        add_driver_simple_fp(mesh, key, ctrl, axis, FP_BOX_SIZE, invert=invert)
        eye_count += 1

    brow_count = 0
    for direction, keys in FP_BROW_LINK.items():
        axis = 'LOC_Z' if direction in ("up", "down") else 'LOC_X'
        invert = direction in ("down", "left")
        for key in keys:
            if not key or key not in available:
                continue
            add_driver_simple_fp(mesh, key, ctrl, axis, FP_BOX_SIZE, invert=invert)
            brow_count += 1

    return eye_count, brow_count


def build_mouth_box_fp(mesh, parent_anchor, local_pos, available):
    """构建嘴巴五角星面板。parent_anchor 为根 Anchor (跟随骨骼)"""
    anchor = make_empty_fp("MouthAnchor", parent_anchor, local_pos,
                           'PLAIN_AXES', 0.1)
    make_square_outline_fp("MouthFrame", anchor, (0, 0, 0), FP_BOX_SIZE)

    star_mesh = bpy.data.meshes.new(f"{FP_PANEL_NAME}_MouthStar_mesh")
    star_verts = [(0, 0, 0)]
    star_edges = []
    for i, (ax, az) in enumerate(FP_VOWEL_ANCHORS.values()):
        star_verts.append((ax * FP_BOX_SIZE, 0, az * FP_BOX_SIZE))
        star_edges.append((0, i + 1))
    star_mesh.from_pydata(star_verts, star_edges, [])
    star_obj = bpy.data.objects.new(f"{FP_PANEL_NAME}_MouthStar", star_mesh)
    star_obj.display_type = 'WIRE'
    star_obj.show_in_front = True
    bpy.context.collection.objects.link(star_obj)
    star_obj.parent = anchor
    star_obj.matrix_parent_inverse.identity()

    for vowel, (ax, az) in FP_VOWEL_ANCHORS.items():
        make_empty_fp(f"MouthAnchor_{vowel}", anchor,
                      (ax * FP_BOX_SIZE, 0, az * FP_BOX_SIZE),
                      'CIRCLE', 0.01)

    ctrl = make_empty_fp("MouthCtrl", anchor, (0, 0, 0), 'CUBE', FP_CTRL_SIZE)
    add_limit_square_fp(ctrl, FP_BOX_SIZE)

    mouth_count = 0
    for vowel, (ax, az) in FP_VOWEL_ANCHORS.items():
        if vowel not in available:
            continue
        add_driver_radial_fp(mesh, vowel, ctrl, ax, az, FP_BOX_SIZE)
        mouth_count += 1
    return mouth_count


def estimate_panel_world_pos_fp(mesh, armature):
    """
    估算面板的【世界坐标】放置位置：角色头部右上方。
    使用 armature.matrix_world @ 头骨的局部头部位置 来得到世界坐标，
    再加上一个右上方的偏移。
    """
    # 默认头高（场景单位）
    head_local = Vector((0.0, 0.0, 1.5))

    if armature and "頭" in armature.data.bones:
        head_bone = armature.data.bones["頭"]
        # head_local 是骨骼在 Armature 静止姿态下的本地坐标
        head_local = head_bone.head_local.copy()

    if armature:
        # 把头骨局部位置 → 世界位置
        head_world = armature.matrix_world @ head_local
    elif mesh:
        head_world = mesh.matrix_world.translation.copy()
        head_world.z += 1.5
    else:
        head_world = Vector((0, 0, 1.5))

    # 在头部右上方放置面板（世界坐标偏移）
    # 角色右侧 = +X，前方 = -Y（MMD 标准朝向），上方 = +Z
    # 偏移量根据角色高度自适应
    scale = max(0.5, head_local.z / 1.5)  # 1.5m 角色为基准
    panel_world = head_world + Vector((0.6 * scale, 0.0, 0.2 * scale))
    return panel_world


def build_root_anchor_fp(mesh, armature, root_bone_name, world_pos):
    """
    创建根 Anchor，放在 world_pos（世界坐标），并通过 Child Of 约束
    跟随根骨骼。Child Of + Set Inverse 自动处理偏移矩阵 ——
    无论骨骼朝向如何，Anchor 的初始世界位置和姿态都保持不变，
    后续骨骼移动/旋转/缩放时 Anchor 跟随。

    返回 (root_anchor, status_msg)
    """
    root = bpy.data.objects.new(f"{FP_PANEL_NAME}_RootAnchor", None)
    root.empty_display_type = 'ARROWS'
    root.empty_display_size = 0.08
    root.show_in_front = True
    bpy.context.collection.objects.link(root)

    # 关键：先放到世界目标位置，确保 Anchor 自身朝向是世界坐标的
    # （旋转保持 0,0,0 → XZ 平面正对 ±Y 方向）
    root.location = world_pos
    root.rotation_euler = (0.0, 0.0, 0.0)

    if armature and root_bone_name and root_bone_name in armature.data.bones:
        # 用 Child Of 约束跟随骨骼。需要先 update view layer 让 Anchor
        # 的世界变换生效，然后用 set_inverse 计算偏移矩阵。
        bpy.context.view_layer.update()

        con = root.constraints.new('CHILD_OF')
        con.name = "FacePanel_FollowBone"
        con.target = armature
        con.subtarget = root_bone_name
        # 全部启用：位置/旋转/缩放都跟随
        con.use_location_x = con.use_location_y = con.use_location_z = True
        con.use_rotation_x = con.use_rotation_y = con.use_rotation_z = True
        con.use_scale_x = con.use_scale_y = con.use_scale_z = True

        # === 关键：计算 inverse matrix ===
        # 这一步等价于在约束面板点 "Set Inverse"，会把约束的偏移矩阵
        # 设为当前骨骼世界变换的逆，使得激活约束后 Anchor 视觉位置不变。
        _set_childof_inverse_fp(root, con, armature, root_bone_name)

        return root, f"Child Of 约束 → 骨骼「{root_bone_name}」"

    elif armature:
        # 退化：Object Parent 到 Armature
        # 先记下世界位置，再设 parent，最后用 matrix_parent_inverse 锁定位置
        world_matrix = root.matrix_world.copy()
        root.parent = armature
        root.matrix_parent_inverse = armature.matrix_world.inverted() @ world_matrix
        return root, f"已挂到 Armature「{armature.name}」"

    elif mesh:
        world_matrix = root.matrix_world.copy()
        root.parent = mesh
        root.matrix_parent_inverse = mesh.matrix_world.inverted() @ world_matrix
        return root, f"未找到 Armature，已挂到 Mesh「{mesh.name}」"

    else:
        return root, "独立放置（不跟随）"


def _set_childof_inverse_fp(owner_obj, constraint, armature, bone_name):
    """
    手动计算并设置 Child Of 约束的 inverse_matrix，
    等价于 UI 上点 "Set Inverse" 按钮。

    inverse_matrix = (target_bone_world_matrix)^-1

    其中 target_bone_world_matrix = armature.matrix_world @ pose_bone.matrix
    """
    pose_bone = armature.pose.bones.get(bone_name)
    if not pose_bone:
        return
    # 骨骼当前的世界变换矩阵
    bone_world = armature.matrix_world @ pose_bone.matrix
    constraint.inverse_matrix = bone_world.inverted()


def build_all_fp():
    """构建表情面板入口。返回 (mesh, msg)"""
    mesh = find_mesh_fp()
    if not mesh:
        return None, "没找到带形态键的 Mesh"

    clear_panel_and_drivers_fp(mesh)

    armature = find_armature_fp(mesh)
    root_bone_name = find_root_bone_fp(armature) if armature else None

    available = get_keys_fp(mesh)
    panel_world_pos = estimate_panel_world_pos_fp(mesh, armature)

    # 创建根 Anchor（Child Of 约束跟随骨骼，初始放在世界目标位置）
    root_anchor, mount_msg = build_root_anchor_fp(
        mesh, armature, root_bone_name, panel_world_pos
    )

    # 在根 Anchor 局部空间下创建眼睛/嘴巴面板
    eye_local = Vector((-FP_BOX_SPACING / 2, 0, 0))
    mouth_local = Vector((FP_BOX_SPACING / 2, 0, 0))

    eye_n, brow_n = build_eye_box_fp(mesh, root_anchor, eye_local, available)
    mouth_n = build_mouth_box_fp(mesh, root_anchor, mouth_local, available)

    register_cross_lock_fp()

    # 锁定非控制器对象的选中
    for obj in bpy.data.objects:
        if obj.name.startswith(FP_PANEL_NAME + "_"):
            if not obj.name.endswith("Ctrl"):
                obj.hide_select = True

    bpy.context.view_layer.update()
    return mesh, f"眼 {eye_n}/4, 眉 {brow_n}, 嘴 {mouth_n}/5  |  {mount_msg}"


# ------------------------------------------------------------
# FacePanel 操作
# ------------------------------------------------------------

class FACEPANEL_OT_add(bpy.types.Operator):
    bl_idname = "facepanel.add"
    bl_label = "添加表情面板"
    bl_description = "生成表情面板，并自动挂载到角色总控骨上以跟随角色位移"

    def execute(self, context):
        mesh, msg = build_all_fp()
        if mesh is None:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        self.report({'INFO'}, f"✅ {msg}")
        return {'FINISHED'}


class FACEPANEL_OT_remove(bpy.types.Operator):
    bl_idname = "facepanel.remove"
    bl_label = "移除表情面板"

    def execute(self, context):
        mesh = find_mesh_fp()
        clear_panel_and_drivers_fp(mesh)
        unregister_cross_lock_fp()
        if mesh:
            reset_all_shape_keys_fp(mesh)
        self.report({'INFO'}, "🗑️ 已移除表情面板")
        return {'FINISHED'}


class FACEPANEL_OT_reset_eye(bpy.types.Operator):
    bl_idname = "facepanel.reset_eye"
    bl_label = "眼部归零"

    def execute(self, context):
        eye_ctrl = bpy.data.objects.get(f"{FP_PANEL_NAME}_EyeCtrl")
        if eye_ctrl:
            eye_ctrl.location = (0, 0, 0)
            bpy.context.view_layer.update()
            self.report({'INFO'}, "✅ 眼部已归零")
        else:
            self.report({'WARNING'}, "未找到眼部控制器")
        return {'FINISHED'}


class FACEPANEL_OT_reset_mouth(bpy.types.Operator):
    bl_idname = "facepanel.reset_mouth"
    bl_label = "嘴巴归零"

    def execute(self, context):
        mouth_ctrl = bpy.data.objects.get(f"{FP_PANEL_NAME}_MouthCtrl")
        if mouth_ctrl:
            mouth_ctrl.location = (0, 0, 0)
            bpy.context.view_layer.update()
            self.report({'INFO'}, "✅ 嘴巴已归零")
        else:
            self.report({'WARNING'}, "未找到嘴巴控制器")
        return {'FINISHED'}


class FACEPANEL_OT_reattach(bpy.types.Operator):
    """重新绑定面板到当前 Armature 的根骨骼（用于切换角色或修复绑定丢失）"""
    bl_idname = "facepanel.reattach"
    bl_label = "重新绑定到骨骼"
    bl_description = "保留当前控制器姿态，只重新挂载面板根 Anchor 到角色总控骨"

    def execute(self, context):
        root = bpy.data.objects.get(f"{FP_PANEL_NAME}_RootAnchor")
        if not root:
            self.report({'WARNING'}, "面板尚未生成")
            return {'CANCELLED'}
        mesh = find_mesh_fp()
        armature = find_armature_fp(mesh)
        root_bone_name = find_root_bone_fp(armature) if armature else None
        panel_world_pos = estimate_panel_world_pos_fp(mesh, armature)

        # === 清掉旧的绑定 ===
        # 移除所有旧的 Child Of 约束
        for c in list(root.constraints):
            if c.type == 'CHILD_OF':
                root.constraints.remove(c)
        # 解除旧 parent
        if root.parent:
            # 保留世界变换（以免视觉跳动）
            world_matrix = root.matrix_world.copy()
            root.parent = None
            root.parent_type = 'OBJECT'
            root.parent_bone = ""
            root.matrix_parent_inverse.identity()
            root.matrix_world = world_matrix

        # === 重新放到目标世界位置 ===
        root.location = panel_world_pos
        root.rotation_euler = (0.0, 0.0, 0.0)
        bpy.context.view_layer.update()

        # === 加新约束/父级 ===
        if armature and root_bone_name:
            con = root.constraints.new('CHILD_OF')
            con.name = "FacePanel_FollowBone"
            con.target = armature
            con.subtarget = root_bone_name
            con.use_location_x = con.use_location_y = con.use_location_z = True
            con.use_rotation_x = con.use_rotation_y = con.use_rotation_z = True
            con.use_scale_x = con.use_scale_y = con.use_scale_z = True
            _set_childof_inverse_fp(root, con, armature, root_bone_name)
            self.report({'INFO'}, f"✅ Child Of → 骨骼「{root_bone_name}」")
        elif armature:
            world_matrix = root.matrix_world.copy()
            root.parent = armature
            root.matrix_parent_inverse = armature.matrix_world.inverted() @ world_matrix
            self.report({'INFO'}, f"✅ 已挂到 Armature「{armature.name}」")
        else:
            self.report({'ERROR'}, "未找到 Armature")
            return {'CANCELLED'}
        bpy.context.view_layer.update()
        return {'FINISHED'}


# ------------------------------------------------------------
# FacePanel UI（独立子面板，与 IKFK 同标签页）
# ------------------------------------------------------------

class MMIKFK_PT_FacePanel(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "MMD IK/FK"
    bl_label = "MMD 表情面板"
    bl_idname = "MMIKFK_PT_facepanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _section_enabled(context, "show_facepanel")

    def draw(self, context):
        layout = self.layout
        mesh = find_mesh_fp()
        armature = find_armature_fp(mesh) if mesh else None
        root_bone = find_root_bone_fp(armature) if armature else None

        # 状态盒
        box = layout.box()
        if mesh:
            box.label(text=f"目标 Mesh: {mesh.name}", icon='MESH_DATA')
            box.label(text=f"形态键: {len(get_keys_fp(mesh))} 个")
        else:
            box.label(text="未找到带形态键的 Mesh", icon='ERROR')

        if armature:
            box.label(text=f"Armature: {armature.name}", icon='ARMATURE_DATA')
            if root_bone:
                box.label(text=f"挂载骨骼: {root_bone}", icon='BONE_DATA')
            else:
                box.label(text="未找到根骨骼，将挂到 Armature 根", icon='INFO')
        else:
            box.label(text="未找到 Armature（面板将挂到 Mesh）", icon='INFO')

        layout.separator()

        col = layout.column(align=True)
        col.scale_y = 1.3
        col.operator("facepanel.add", icon='ADD')
        col.operator("facepanel.remove", icon='X')

        layout.separator()
        row = layout.row(align=True)
        row.scale_y = 1.2
        row.operator("facepanel.reset_eye", icon='LOOP_BACK')
        row.operator("facepanel.reset_mouth", icon='LOOP_BACK')

        # 重新绑定按钮
        row = layout.row(align=True)
        row.operator("facepanel.reattach", icon='CONSTRAINT_BONE')

        # 运行状态
        panel_exists = any(o.name.startswith(FP_PANEL_NAME + "_")
                           for o in bpy.data.objects)
        status = layout.box()
        if panel_exists:
            status.label(text="✅ 面板已生成", icon='CHECKMARK')
            status.label(text="🔒 眼睛十字锁已激活")
            root = bpy.data.objects.get(f"{FP_PANEL_NAME}_RootAnchor")
            if root:
                # 检查 Child Of 约束
                childof = next(
                    (c for c in root.constraints if c.type == 'CHILD_OF'),
                    None
                )
                if childof and childof.target and childof.subtarget:
                    status.label(
                        text=f"📌 跟随骨骼: {childof.subtarget}",
                        icon='LINKED'
                    )
                elif root.parent:
                    status.label(
                        text=f"📌 跟随对象: {root.parent.name}",
                        icon='LINKED'
                    )
                else:
                    status.label(text="⚠️ 未挂载父级", icon='UNLINKED')
        else:
            status.label(text="⚪ 未生成", icon='RADIOBUT_OFF')


# ============================================================
# 注册（合并 IKFK + FacePanel）
# ============================================================

ikfk_classes = (
    MMIKFK_Preferences,
    MMIKFK_Properties,
    MMIKFK_OT_Setup,
    MMIKFK_OT_SnapIKtoFK,
    MMIKFK_OT_SnapFKtoIK,
    MMIKFK_OT_SwitchToIK,
    MMIKFK_OT_SwitchToFK,
    MMIKFK_OT_ToggleIKFK,
    MMIKFK_OT_KeyframeAll,
    MMIKFK_OT_ResetAllPose,
    MMIKFK_OT_AbsorbPose,
    MMIKFK_OT_WritebackPose,
    MMIKFK_OT_AbsorbMotion,
    MMIKFK_OT_BakeMotionToOrig,
    MMIKFK_OT_ImportVPDFile,
    MMIKFK_OT_ExportVPDFile,
    MMIKFK_OT_ImportVMDFile,
    MMIKFK_OT_ExportVMDFile,
    MMIKFK_OT_AutoBendFK,
    MMIKFK_OT_CalibrateBend,
    MMIKFK_OT_ExportPose,
    MMIKFK_OT_ImportPose,
    MMIKFK_OT_Cleanup,
    MMIKFK_PT_MainPanel,
)

facepanel_classes = (
    FACEPANEL_OT_add,
    FACEPANEL_OT_remove,
    FACEPANEL_OT_reset_eye,
    FACEPANEL_OT_reset_mouth,
    FACEPANEL_OT_reattach,
    MMIKFK_PT_FacePanel,
)

classes = ikfk_classes + facepanel_classes


def register():
    for cls in classes:
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass
        bpy.utils.register_class(cls)
    bpy.types.Object.mmikfk_props = PointerProperty(type=MMIKFK_Properties)


def unregister():
    # 清理 FacePanel handler
    unregister_cross_lock_fp()
    try:
        del bpy.types.Object.mmikfk_props
    except:
        pass
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass


if __name__ == "__main__":
    register()
    print("\n✅ MMD IK/FK + 表情面板 (合并版 v10.0) 已加载")
    print("   按 N 键 → MMD IK/FK 标签页")
