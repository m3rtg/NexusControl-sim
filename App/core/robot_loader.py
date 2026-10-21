import pybullet as p
import os
import re
import math
import numpy as np

class RobotLoader:
    def __init__(self, sim_engine, kinematics_engine):
        self.sim_engine = sim_engine
        self.kinematics_engine = kinematics_engine
        self.known_packages = {}

    def find_package_dir(self, pkg_name, context_path=None):
        if not pkg_name:
            return None

        cache_key = (pkg_name.lower(), str(context_path))
        if cache_key in self.known_packages:
            return self.known_packages[cache_key]

        app_dir = os.path.dirname(os.path.abspath(__file__))
        sim_root = os.path.normpath(os.path.join(app_dir, "..", ".."))
        downloads_dir = os.path.expanduser("~/Downloads")

        if context_path and os.path.exists(context_path):
            curr = os.path.abspath(context_path) if os.path.isdir(context_path) else os.path.dirname(os.path.abspath(context_path))
            for _ in range(6):
                if os.path.basename(curr).lower() == pkg_name.lower():
                    self.known_packages[cache_key] = curr
                    return curr
                cand = os.path.join(curr, pkg_name)
                if os.path.isdir(cand):
                    self.known_packages[cache_key] = cand
                    return cand
                pkg_xml = os.path.join(curr, "package.xml")
                if os.path.isfile(pkg_xml):
                    try:
                        with open(pkg_xml, 'r', encoding='utf-8', errors='ignore') as f:
                            if f"<name>{pkg_name}</name>" in f.read():
                                self.known_packages[cache_key] = curr
                                return curr
                    except Exception:
                        pass
                parent = os.path.dirname(curr)
                if os.path.isdir(parent):
                    for entry in os.listdir(parent):
                        cand_sib = os.path.join(parent, entry)
                        if os.path.isdir(cand_sib) and entry.lower() == pkg_name.lower():
                            self.known_packages[cache_key] = cand_sib
                            return cand_sib
                if parent == curr:
                    break
                curr = parent

        for root_dir in [downloads_dir, os.path.join(sim_root, "resources"), sim_root]:
            if not os.path.isdir(root_dir):
                continue
            cand = os.path.join(root_dir, pkg_name)
            if os.path.isdir(cand):
                self.known_packages[cache_key] = cand
                return cand
            try:
                for entry in os.listdir(root_dir):
                    sub = os.path.join(root_dir, entry)
                    if os.path.isdir(sub):
                        if entry.lower() == pkg_name.lower():
                            self.known_packages[cache_key] = sub
                            return sub
                        cand_sub = os.path.join(sub, pkg_name)
                        if os.path.isdir(cand_sub):
                            self.known_packages[cache_key] = cand_sub
                            return cand_sub
            except Exception:
                pass

        self.known_packages[cache_key] = None
        return None

    def resolve_mesh_path(self, urdf_dir, mesh_filename, context_dir=None):
        if not mesh_filename:
            return None
        if isinstance(mesh_filename, bytes):
            mesh_filename = mesh_filename.decode('utf-8')
        if os.path.isabs(mesh_filename) and os.path.exists(mesh_filename):
            return os.path.abspath(mesh_filename).replace('\\', '/')

        clean_path = mesh_filename
        pkg_name = ''
        for prefix in ["package://", "model://", "file://"]:
            if clean_path.startswith(prefix):
                clean_path = clean_path[len(prefix):]
                parts = clean_path.replace("\\", "/").split("/")
                if len(parts) > 1:
                    pkg_name = parts[0]
                    clean_path = "/".join(parts[1:])
                break

        if pkg_name:
            pkg_dir = self.find_package_dir(pkg_name, context_path=context_dir or urdf_dir)
            if pkg_dir:
                p_cand = os.path.normpath(os.path.join(pkg_dir, clean_path))
                if os.path.isfile(p_cand):
                    return os.path.abspath(p_cand).replace('\\', '/')
                base_name = os.path.basename(clean_path)
                for root, dirs, files in os.walk(pkg_dir):
                    if base_name in files:
                        return os.path.abspath(os.path.join(root, base_name)).replace('\\', '/')

        search_dirs = []
        if context_dir and os.path.isdir(context_dir):
            search_dirs += [
                context_dir,
                os.path.normpath(os.path.join(context_dir, "..")),
                os.path.normpath(os.path.join(context_dir, "meshes")),
                os.path.normpath(os.path.join(context_dir, "..", "meshes")),
                os.path.normpath(os.path.join(context_dir, "..", "..")),
            ]
        if urdf_dir and os.path.isdir(urdf_dir):
            search_dirs += [
                urdf_dir,
                os.path.normpath(os.path.join(urdf_dir, "..")),
                os.path.normpath(os.path.join(urdf_dir, "meshes")),
                os.path.normpath(os.path.join(urdf_dir, "..", "meshes")),
                os.path.normpath(os.path.join(urdf_dir, "..", "..")),
            ]

        for cdir in search_dirs:
            p_check = os.path.normpath(os.path.join(cdir, clean_path))
            if os.path.isfile(p_check):
                return os.path.abspath(p_check).replace('\\', '/')

        if not pkg_name and context_dir and os.path.isdir(context_dir):
            base_name = os.path.basename(mesh_filename)
            for root, dirs, files in os.walk(context_dir):
                if base_name in files:
                    return os.path.abspath(os.path.join(root, base_name)).replace('\\', '/')

        return None

    def sanitize_and_resolve_urdf(self, urdf_path, context_dir=None):
        try:
            import xml.etree.ElementTree as ET
            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, "..", ".."))
            cache_dir = os.path.join(sim_root, ".cache")
            os.makedirs(cache_dir, exist_ok=True)

            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            for geom in root.iter("geometry"):
                mesh = geom.find("mesh")
                if mesh is not None:
                    fn = mesh.attrib.get("filename", "")
                    resolved = self.resolve_mesh_path(urdf_dir, fn, context_dir=context_dir)
                    if resolved and os.path.isfile(resolved):
                        mesh.attrib["filename"] = os.path.abspath(resolved).replace("\\", "/")
                    else:
                        geom.remove(mesh)
                        cyl = ET.SubElement(geom, "cylinder")
                        cyl.attrib["radius"] = "0.04"
                        cyl.attrib["length"] = "0.18"

            base_name = os.path.splitext(os.path.basename(urdf_path))[0]
            if base_name.startswith("."): base_name = base_name[1:]
            sanitized_path = os.path.join(cache_dir, f"sanitized_{base_name}.urdf")
            ET.indent(tree, space="  ")
            tree.write(sanitized_path, encoding="utf-8")
            return sanitized_path
        except Exception as e:
            print(f"[URDF Sanitize] Hata: {e}")
            return urdf_path

    def convert_xacro_to_urdf(self, xacro_path):
        try:
            import xacro
        except ImportError:
            return None, "HATA: 'xacro' modülü yüklü değil! pip install xacro yapın."

        try:
            import sys
            import types

            def local_find_package_dir(pkg_name):
                found = self.find_package_dir(pkg_name, context_path=xacro_path)
                if found: return found
                base_dir = os.path.dirname(os.path.abspath(xacro_path))
                return os.path.dirname(base_dir) if os.path.isdir(os.path.dirname(base_dir)) else base_dir

            if "ament_index_python" not in sys.modules:
                ament_mod = types.ModuleType("ament_index_python")
                pkg_mod = types.ModuleType("ament_index_python.packages")
                ament_mod.packages = pkg_mod
                sys.modules["ament_index_python"] = ament_mod
                sys.modules["ament_index_python.packages"] = pkg_mod
            sys.modules["ament_index_python.packages"].get_package_share_directory = local_find_package_dir

            try:
                import xacro.substitution_args as sa
                sa._eval_find = local_find_package_dir
                if hasattr(sa, "_resolve_args"):
                    orig_resolve = sa._resolve_args
                    def patched_resolve_args(arg_str, context, commands):
                        if "$(" in arg_str:
                            arg_str = re.sub(r"\$\(find-pkg-share\s+([^)]+)\)", r"$(find \1)", arg_str)
                            arg_str = re.sub(r"\$\(find-pkg-prefix\s+([^)]+)\)", r"$(find \1)", arg_str)
                        return orig_resolve(arg_str, context, commands)
                    sa._resolve_args = patched_resolve_args
            except Exception:
                pass

            try:
                orig_abs = xacro.abs_filename_spec
                def smart_abs_filename_spec(filename_spec):
                    resolved = orig_abs(filename_spec)
                    if not os.path.exists(resolved):
                        target_name = os.path.basename(filename_spec)
                        base_dir = os.path.dirname(os.path.abspath(xacro_path))
                        app_dir = os.path.dirname(os.path.abspath(__file__))
                        sim_root = os.path.normpath(os.path.join(app_dir, "..", ".."))

                        if target_name in ["common_materials.xacro", "common_colours.xacro"]:
                            for r_dir in [
                                os.path.join(sim_root, "resources", "fanuc_resources", "urdf"),
                                os.path.join(sim_root, "resources", "abb_resources", "urdf")
                            ]:
                                cand = os.path.join(r_dir, target_name)
                                if os.path.isfile(cand): return cand

                        for search_root in [base_dir, os.path.dirname(base_dir), os.path.dirname(os.path.dirname(base_dir))]:
                            if not os.path.isdir(search_root): continue
                            for root, dirs, files in os.walk(search_root):
                                if target_name in files:
                                    return os.path.join(root, target_name)
                                if root[len(search_root):].count(os.sep) >= 3:
                                    dirs.clear()
                    return resolved
                xacro.abs_filename_spec = smart_abs_filename_spec
            except Exception:
                pass

            doc = xacro.process_file(xacro_path)
            urdf_content = doc.toxml()

            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, "..", ".."))
            cache_dir = os.path.join(sim_root, ".cache")
            os.makedirs(cache_dir, exist_ok=True)

            file_name = os.path.splitext(os.path.basename(xacro_path))[0]
            out_path = os.path.join(cache_dir, f"compiled_{file_name}.urdf")
            with open(out_path, "w", encoding="utf-8") as f:
                f.write(urdf_content)

            return out_path, None
        except Exception as e:
            return None, str(e)

    def load_source_model(self, file_path):
        if not file_path or not os.path.isfile(file_path):
            return False, "Dosya bulunamadı", None

        context_dir = os.path.dirname(os.path.abspath(file_path))
        if file_path.lower().endswith(".xacro"):
            urdf_path, err = self.convert_xacro_to_urdf(file_path)
            if not urdf_path:
                return False, f"Xacro Derleme hatası: {err}", None
        else:
            urdf_path = file_path

        return self.load_robot_model(urdf_path, context_dir=context_dir)

    def load_robot_model(self, urdf_path, context_dir=None):
        if not os.path.isfile(urdf_path):
            return False, "Dosya bulunamadı", None
            
        app_dir = os.path.dirname(os.path.abspath(__file__))
        sim_root = os.path.normpath(os.path.join(app_dir, "..", ".."))
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        if not context_dir: context_dir = urdf_dir

        sanitized_urdf = self.sanitize_and_resolve_urdf(urdf_path, context_dir=context_dir)

        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is not None:
                try: p.removeBody(self.sim_engine.robotId)
                except Exception: pass

            p.setAdditionalSearchPath(urdf_dir)
            if context_dir and os.path.isdir(context_dir): p.setAdditionalSearchPath(context_dir)
            p.setAdditionalSearchPath(os.path.normpath(os.path.join(urdf_dir, "..")))
            p.setAdditionalSearchPath(os.path.join(sim_root, "resources"))
            p.setAdditionalSearchPath(sim_root)

            try:
                self.sim_engine.robotId = p.loadURDF(
                    sanitized_urdf,
                    [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True
                )
            except Exception as e:
                return False, f"URDF Yükleme hatası: {e}", None

            num_joints = p.getNumJoints(self.sim_engine.robotId)
            revolute_joints = [
                i for i in range(num_joints)
                if p.getJointInfo(self.sim_engine.robotId, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, getattr(p, 'JOINT_CONTINUOUS', 0))
            ]
            if not revolute_joints:
                revolute_joints = [i for i in range(num_joints) if p.getJointInfo(self.sim_engine.robotId, i)[2] != p.JOINT_FIXED]
            if not revolute_joints:
                revolute_joints = list(range(min(6, num_joints)))
            self.sim_engine.revolute_joints = revolute_joints

            link_names = {-1: "base_link"}
            for ji in range(num_joints):
                child_link_name = p.getJointInfo(self.sim_engine.robotId, ji)[12].decode('utf-8')
                link_names[ji] = child_link_name
            self.kinematics_engine.link_names = link_names

            # Auto-framing & Auto-sizing
            min_coords = [float('inf')] * 3
            max_coords = [float('-inf')] * 3
            for link_idx in [-1] + list(range(num_joints)):
                try:
                    aabb_min, aabb_max = p.getAABB(self.sim_engine.robotId, link_idx)
                    for k in range(3):
                        min_coords[k] = min(min_coords[k], aabb_min[k])
                        max_coords[k] = max(max_coords[k], aabb_max[k])
                except Exception:
                    pass

            span_x = max_coords[0] - min_coords[0]
            span_y = max_coords[1] - min_coords[1]
            span_z = max_coords[2] - min_coords[2]
            max_span = max(span_x, span_y, span_z)
            if max_span < 0.1 or max_span > 60.0 or math.isinf(max_span): max_span = 0.8
            center_z = (max_coords[2] + min_coords[2]) / 2.0
            if math.isinf(center_z) or math.isnan(center_z): center_z = 0.45

            auto_cam_dist = max(1.6, float(max_span * 2.2))
            auto_cam_target = np.array([0.0, 0.0, float(center_z)])
            auto_grid_size = max(1.5, float(max_span * 1.5))
            self.kinematics_engine.robot_reach_cm = max(35.0, float(max(span_x, span_y, span_z * 0.8)) * 100.0)

            # Kaide Tespiti
            stand_link_idx = None
            for i in range(num_joints):
                name = p.getJointInfo(self.sim_engine.robotId, i)[12].decode('utf-8').lower()
                if any(s in name for s in ['stand', 'table', 'pedestal', 'sehp', 'kaide']):
                    stand_link_idx = i
                    break

            if stand_link_idx is not None:
                aabb_min, aabb_max = p.getAABB(self.sim_engine.robotId, stand_link_idx)
                self.kinematics_engine.stand_top_cm = max(0.0, aabb_max[2] * 100.0)
                self.kinematics_engine.stand_half_extent_cm = max(abs(aabb_max[0]), abs(aabb_max[1])) * 100.0
                self.kinematics_engine.min_safe_z_cm = self.kinematics_engine.stand_top_cm + 5.0
            else:
                self.kinematics_engine.stand_top_cm = 0.0
                self.kinematics_engine.stand_half_extent_cm = 0.0
                self.kinematics_engine.min_safe_z_cm = 5.0

            first_rev = revolute_joints[0] if revolute_joints else 0
            shoulder_st = p.getLinkState(self.sim_engine.robotId, first_rev)
            self.kinematics_engine.base_z_cm = shoulder_st[4][2] * 100.0

            ee_candidates = []
            for ji in range(num_joints):
                info = p.getJointInfo(self.sim_engine.robotId, ji)
                j_name = info[1].decode('utf-8').lower()
                c_name = info[12].decode('utf-8').lower()
                if any(k in c_name or k in j_name for k in ['tcp', 'tool0', 'ee_link', 'flange', 'end_effector']):
                    ee_candidates.append(ji)
            if ee_candidates:
                self.sim_engine.end_effector_index = ee_candidates[-1]
            else:
                rev_only = [i for i in revolute_joints if p.getJointInfo(self.sim_engine.robotId, i)[2] in (p.JOINT_REVOLUTE, getattr(p, 'JOINT_CONTINUOUS', 0))]
                self.sim_engine.end_effector_index = rev_only[-1] if rev_only else (revolute_joints[-1] if revolute_joints else 0)

            shoulder_idx = revolute_joints[1] if len(revolute_joints) > 1 else first_rev
            ls_shoulder = p.getLinkState(self.sim_engine.robotId, shoulder_idx)
            s_pos = ls_shoulder[4]
            self.kinematics_engine.shoulder_r_cm = math.sqrt(s_pos[0]**2 + s_pos[1]**2) * 100.0
            self.kinematics_engine.shoulder_z_cm = s_pos[2] * 100.0

            arm_len_m = 0.0
            curr = self.sim_engine.end_effector_index
            visited = set()
            while curr != shoulder_idx and curr >= 0 and curr not in visited:
                visited.add(curr)
                info = p.getJointInfo(self.sim_engine.robotId, curr)
                arm_len_m += math.sqrt(sum(x**2 for x in info[14]))
                curr = info[16]
            if arm_len_m > 0.1:
                self.kinematics_engine.arm_length_cm = arm_len_m * 100.0 * 0.94
                self.kinematics_engine.robot_reach_cm = self.kinematics_engine.shoulder_r_cm + self.kinematics_engine.arm_length_cm
            else:
                self.kinematics_engine.arm_length_cm = self.kinematics_engine.robot_reach_cm * 0.9

            j_lower, j_upper, j_ranges, j_rest = [], [], [], []
            for j_idx in revolute_joints:
                info = p.getJointInfo(self.sim_engine.robotId, j_idx)
                low = float(info[8])
                high = float(info[9])
                if low >= high:
                    low = -math.pi
                    high = math.pi
                j_lower.append(low)
                j_upper.append(high)
                j_ranges.append(high - low)
                j_rest.append(0.0 if (low <= 0.0 <= high) else ((low + high) / 2.0))
                try: p.changeDynamics(self.sim_engine.robotId, j_idx, linearDamping=0.2, angularDamping=0.2, jointDamping=0.5)
                except: pass
            
            self.kinematics_engine.joint_lower_limits = j_lower
            self.kinematics_engine.joint_upper_limits = j_upper
            self.kinematics_engine.joint_ranges = j_ranges
            self.kinematics_engine.joint_rest_poses = j_rest

            allowed_cols = set()
            for ji in range(num_joints):
                parent_idx = p.getJointInfo(self.sim_engine.robotId, ji)[16]
                if parent_idx >= -1:
                    allowed_cols.add((min(parent_idx, ji), max(parent_idx, ji)))
            initial_contacts = p.getClosestPoints(self.sim_engine.robotId, self.sim_engine.robotId, distance=0.01)
            for c in initial_contacts:
                lA, lB = c[3], c[4]
                if lA != lB: allowed_cols.add((min(lA, lB), max(lA, lB)))
            self.kinematics_engine.allowed_collision_pairs = allowed_cols

            try:
                shapes = p.getVisualShapeData(self.sim_engine.robotId)
            except Exception:
                shapes = []
            
            default_palette = [
                (0.25, 0.25, 0.25), (0.96, 0.76, 0.13), (0.96, 0.76, 0.13),
                (0.96, 0.76, 0.13), (0.96, 0.76, 0.13), (0.20, 0.20, 0.20),
                (0.25, 0.25, 0.25), (0.00, 0.65, 0.85), (0.95, 0.35, 0.10)
            ]
            visual_shapes_meta = []
            mesh_data = []

            for idx, s in enumerate(shapes):
                link_idx = s[1]
                geom_type = s[2]
                dims = s[3]
                mesh_fn = s[4].decode('utf-8') if isinstance(s[4], bytes) else s[4]
                resolved_mesh = self.resolve_mesh_path(urdf_dir, mesh_fn) if mesh_fn else None
                local_pos = s[5]
                local_ori = s[6]
                rgba = s[7]
                col = (float(rgba[0]), float(rgba[1]), float(rgba[2])) if (rgba and any(rgba[:3])) else None
                if not col:
                    if 'cr35' in urdf_path.lower() or 'cr_' in urdf_path.lower():
                        col = (0.28, 0.61, 0.43) if link_idx >= 0 else (0.22, 0.22, 0.22)
                    else:
                        col = default_palette[idx % len(default_palette)]
                        
                visual_shapes_meta.append({
                    'link_idx': link_idx, 'geom_type': geom_type, 'dims': dims,
                    'local_pos': local_pos, 'local_ori': local_ori
                })
                mesh_data.append({
                    'name': f'vis_{idx}_link_{link_idx}', 'mesh_path': resolved_mesh,
                    'color': col, 'geom_type': geom_type, 'dims': dims
                })
            
            self.sim_engine.visual_shapes_meta = visual_shapes_meta

            if self.sim_engine.gpu_shared_state is not None and self.sim_engine.gpu_state_lock is not None:
                with self.sim_engine.gpu_state_lock:
                    self.sim_engine.gpu_shared_state['mesh_data'] = mesh_data
                    self.sim_engine.gpu_shared_state['reload_meshes'] = True
                    self.sim_engine.gpu_shared_state['transforms'] = [None] * len(visual_shapes_meta)
                    self.sim_engine.gpu_shared_state['cam_dist'] = auto_cam_dist
                    self.sim_engine.gpu_shared_state['cam_target'] = auto_cam_target
                    self.sim_engine.gpu_shared_state['cam_yaw'] = 45.0
                    self.sim_engine.gpu_shared_state['cam_pitch'] = -25.0
                    self.sim_engine.gpu_shared_state['grid_size'] = auto_grid_size

            return True, f"Model Başarıyla Yüklendi ({len(revolute_joints)} Eksen)", {
                "max_span": max_span,
                "n_joints": len(revolute_joints)
            }
