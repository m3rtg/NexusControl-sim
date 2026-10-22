import pybullet as p
from i18n import t
import math
import time
import threading

# İz segmentleri pybullet gövdesi değil, GPU renderer'a giden düz veridir.
# Sınırsız büyümemesi için en eski segmentler atılır.
MAX_TRAIL_SEGMENTS = 4000

# Hızlı rotada hızlanma ve yavaşlama, segment süresinin bu oranı kadar sürer (trapez hız profili)
TRAPEZOID_BLEND = 0.2

def motion_profile(strategy, u):
    """Segment içinde normalize zaman u (0..1) -> (s, ds/du, d²s/du²), s = alınan yolun oranı.
    Hızlı: trapez hız profili. Eko: S-eğrisi. Oynatıcı ve enerji tahmini aynı profili kullanır."""
    if strategy == "fast":
        a = TRAPEZOID_BLEND
        v = 1.0 / (1.0 - a)
        if u < a:
            return 0.5 * v / a * u * u, v * u / a, v / a
        if u > 1.0 - a:
            w = 1.0 - u
            return 1.0 - 0.5 * v / a * w * w, v * w / a, -v / a
        return v * (u - 0.5 * a), v, 0.0
    return (1.0 - math.cos(math.pi * u)) / 2.0, 0.5 * math.pi * math.sin(math.pi * u), 0.5 * math.pi ** 2 * math.cos(math.pi * u)

class KinematicsEngine:
    def __init__(self, sim_engine, telemetry_engine):
        self.sim_engine = sim_engine
        self.telemetry = telemetry_engine
        
        self.show_workspace = False
        self.trail_segments = []
        self.last_link_positions = {}
        
        # Kinematics specifics that are populated by Loader
        self.robot_reach_cm = 80.0
        self.shoulder_r_cm = 7.5
        self.shoulder_z_cm = 33.0
        self.arm_length_cm = 72.5
        self.base_z_cm = 33.0
        self.stand_top_cm = 0.0
        self.stand_half_extent_cm = 0.0
        self.min_safe_z_cm = 5.0
        
        # Constraints and physics
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        self.joint_ranges = []
        self.joint_rest_poses = []
        self.allowed_collision_pairs = set()
        self.link_names = {}
        
        # Output artifacts
        self.pending_waypoints = []
        self.pending_waypoint_safety = []
        self.pending_waypoint_labels = []   # arayüz için durak adları (i18n anahtarları)
        self.last_segment_energy = []       # son tahminde segment başına enerji (J)
        self.route_segment_energy = []      # son IK rotasının Hızlı profilde segment enerjileri
        self.time_fast = 0.0
        self.time_eco = 0.0
        self.peak_power_fast = 0.0
        self.peak_power_eco = 0.0

    def prismatic_joint_flags(self):
        """revolute_joints sırasıyla: eklem lineer (prismatic) ise True. Değeri metre, açı değil."""
        with self.sim_engine.bullet_lock:
            return [p.getJointInfo(self.sim_engine.robotId, j)[2] == p.JOINT_PRISMATIC
                    for j in self.sim_engine.revolute_joints]

    def ee_chain_joints(self):
        """Uç işlevciden kaideye kadar zincirdeki eklemler. Zincir dışındaki eklemler
        (ör. gripper parmakları) TCP konumunu etkilemez."""
        chain = set()
        with self.sim_engine.bullet_lock:
            curr = getattr(self.sim_engine, 'end_effector_index', 0)
            while curr >= 0 and curr not in chain:
                chain.add(curr)
                curr = p.getJointInfo(self.sim_engine.robotId, curr)[16]
        return chain

    def detect_collision(self, check_floor):
        """Gerçek temas kontrolü (E-stop için). Çarpışma varsa açıklama, yoksa None döner.
        Engel ve gövde/kaide teması her zaman; kaide/zemin seviye kontrolü sadece yörünge oynarken
        yapılır (refactor öncesi davranış)."""
        with self.sim_engine.bullet_lock:
            robot = self.sim_engine.robotId
            if robot is None: return None

            if self.sim_engine.obstacle_id is not None:
                hits = [c for c in p.getContactPoints(robot, self.sim_engine.obstacle_id) if c[8] < 0.0]
                if hits:
                    return t("estop.obstacle", link=self.link_names.get(hits[0][3], f'Link {hits[0][3]}'))

            for c in p.getContactPoints(robot, robot):
                lA, lB = c[3], c[4]
                if lA == lB or c[8] >= 0.0 or (min(lA, lB), max(lA, lB)) in self.allowed_collision_pairs: continue
                return t("estop.self", a=self.link_names.get(lA, f'Link {lA}'), b=self.link_names.get(lB, f'Link {lB}'))

            if check_floor:
                for j_idx in self.sim_engine.revolute_joints:
                    ls = p.getLinkState(robot, j_idx)
                    lx, ly, lz = ls[4][0] * 100.0, ls[4][1] * 100.0, ls[4][2] * 100.0
                    name = self.link_names.get(j_idx, f"Link {j_idx}")
                    if self.stand_half_extent_cm > 0 and abs(lx) <= self.stand_half_extent_cm and abs(ly) <= self.stand_half_extent_cm:
                        if lz < self.stand_top_cm - 0.5:
                            return t("estop.stand", link=name)
                    elif lz < -1.0:
                        return t("estop.floor", link=name)
        return None

    def calculate_fk(self):
        """Calculates Forward Kinematics and returns formatted log text."""
        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is None: return t("msg.no_robot")
            state = p.getLinkState(self.sim_engine.robotId, getattr(self.sim_engine, 'end_effector_index', 0))
            joint_states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)

        pos = state[4]
        quat = state[5]
        current_joints = [js[0] for js in joint_states]
        rot_matrix = p.getMatrixFromQuaternion(quat)

        T = [
            [rot_matrix[0], rot_matrix[1], rot_matrix[2], pos[0] * 100],
            [rot_matrix[3], rot_matrix[4], rot_matrix[5], pos[1] * 100],
            [rot_matrix[6], rot_matrix[7], rot_matrix[8], pos[2] * 100],
            [0.0, 0.0, 0.0, 1.0]
        ]

        n_joints = len(self.sim_engine.revolute_joints)
        prismatic = self.prismatic_joint_flags()
        fk_log = "\n\n" + "═" * 50 + "\n"
        fk_log += t("fk.title") + "\n"
        fk_log += "═" * 50 + "\n\n"
        
        fk_log += t("fk.joints") + "\n"
        fk_log += " ──────────────────────────────────────────\n"
        fk_log += t("fk.joint_header" if not any(prismatic) else "fk.joint_header_linear") + "\n"
        fk_log += " ──────────────────────────────────────────\n"
        for i, val in enumerate(current_joints):
            if i < len(prismatic) and prismatic[i]:
                fk_log += f"   J{i+1:<3} | {val * 1000:>8.2f} mm | {val:>8.4f} m\n"
            else:
                fk_log += f"   J{i+1:<3} | {math.degrees(val):>8.2f}° | {val:>8.4f}\n"
        fk_log += " ──────────────────────────────────────────\n\n"

        fk_log += t("fk.position") + "\n"
        fk_log += " ──────────────────────────────────────────\n"
        fk_log += t("fk.x", v=f"{T[0][3]:8.2f}") + "\n"
        fk_log += t("fk.y", v=f"{T[1][3]:8.2f}") + "\n"
        fk_log += t("fk.z", v=f"{T[2][3]:8.2f}") + "\n"
        fk_log += " ──────────────────────────────────────────\n\n"

        fk_log += t("fk.pose") + "\n"
        fk_log += t("fk.pose_header") + "\n"
        fk_log += f"  [{T[0][0]:>8.3f}      {T[0][1]:>8.3f}      {T[0][2]:>8.3f}   | {T[0][3]:8.2f} ]\n"
        fk_log += f"  [{T[1][0]:>8.3f}      {T[1][1]:>8.3f}      {T[1][2]:>8.3f}   | {T[1][3]:8.2f} ]\n"
        fk_log += f"  [{T[2][0]:>8.3f}      {T[2][1]:>8.3f}      {T[2][2]:>8.3f}   | {T[2][3]:8.2f} ]\n"
        fk_log += f"  [   0.000          0.000          0.000   |    1.00  ]\n"
        fk_log += "\n" + "═" * 50 + "\n"

        return fk_log

    def check_angles_safety(self, angles):
        """Simulates angles virtually to check for collisions without moving the actual robot"""
        with self.sim_engine.bullet_lock:
            original_states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
            try:
                for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                    if i < len(angles):
                        p.resetJointState(self.sim_engine.robotId, j_idx, angles[i])

                # 1. Self Collision
                if self.allowed_collision_pairs:
                    contacts = p.getClosestPoints(self.sim_engine.robotId, self.sim_engine.robotId, distance=0.0)
                    for c in contacts:
                        lA, lB, dist = c[3], c[4], c[8]
                        if lA == lB: continue
                        pair = (min(lA, lB), max(lA, lB))
                        if pair not in self.allowed_collision_pairs and dist < -0.012:
                            nameA = self.link_names.get(lA, f"Link_{lA}")
                            nameB = self.link_names.get(lB, f"Link_{lB}")
                            return False, t("safety.self", a=nameA, b=nameB)

                # 2-3. Kaide/zemin ve engel
                hit = self._pose_hits_environment()
                if hit: return False, hit

                return True, t("safety.ok")
            finally:
                for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                    p.resetJointState(self.sim_engine.robotId, j_idx, original_states[i][0], original_states[i][1])

    def _pose_hits_environment(self):
        """Robotun o anki (resetJointState ile kurulmuş) duruşu kaideye, zemine veya engele giriyor mu?
        Ucuz kontroller (~0.1 ms); yol örneklemede her adımda çağrılır. bullet_lock tutulurken çağrılmalı."""
        robot = self.sim_engine.robotId
        for j_idx in self.sim_engine.revolute_joints:
            ls = p.getLinkState(robot, j_idx)
            lx, ly, lz = ls[4][0] * 100.0, ls[4][1] * 100.0, ls[4][2] * 100.0
            if self.stand_half_extent_cm > 0 and abs(lx) <= self.stand_half_extent_cm and abs(ly) <= self.stand_half_extent_cm:
                if lz < self.stand_top_cm:
                    return t("safety.stand", link=self.link_names.get(j_idx, f'Link_{j_idx}'))
            elif lz < 2.0:
                return t("safety.floor", link=self.link_names.get(j_idx, f'Link_{j_idx}'))

        if self.sim_engine.obstacle_id is not None:
            # getContactPoints son fizik adımının temaslarını verir, resetJointState ile kurulan deneme
            # duruşunu görmez (her duruş "güvenli" çıkıyordu). getClosestPoints geometriyi o an hesaplar.
            # Küçük pay: rota engeli sıyırırsa çalışırken E-stop tetiklenir.
            margin = 0.01 * self.sim_engine.obstacle_scale
            pts = p.getClosestPoints(robot, self.sim_engine.obstacle_id, margin)
            if pts:
                return t("safety.obstacle", link=self.link_names.get(pts[0][3], f'Link_{pts[0][3]}'))
        return None

    def check_path_safety(self, start, end):
        """Yörünge oynatıcısı eklem uzayında doğrusal gider (Hızlı ve Eko aynı yolu farklı hızda izler).
        Bu yol ~2°'lik adımlarla kaide/zemin/engel için örneklenir. Gövde içi çarpışma pahalı
        (Fanuc'ta ~5 ms) olduğu için sadece waypoint'lerde bakılır; çalışırken E-stop yine yakalar."""
        n = min(len(start), len(end), len(self.sim_engine.revolute_joints))
        if n == 0: return True, t("safety.ok")
        steps = max(1, math.ceil(max(abs(end[i] - start[i]) for i in range(n)) / math.radians(2.0)))
        with self.sim_engine.bullet_lock:
            original_states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
            try:
                for k in range(1, steps + 1):
                    frac = k / steps
                    for i in range(n):
                        p.resetJointState(self.sim_engine.robotId, self.sim_engine.revolute_joints[i], start[i] + (end[i] - start[i]) * frac)
                    hit = self._pose_hits_environment()
                    if hit: return False, t("safety.on_path", hit=hit)
                return True, t("safety.ok")
            finally:
                for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                    p.resetJointState(self.sim_engine.robotId, j_idx, original_states[i][0], original_states[i][1])

    def profile_state(self, start, end, seg_duration, strategy, u):
        """Oynatıcının segmentteki u anındaki eklem konumu, hızı ve ivmesi."""
        s, ds, dds = motion_profile(strategy, u)
        d = [b - a for a, b in zip(start, end)]
        q = [a + di * s for a, di in zip(start, d)]
        qd = [di * ds / seg_duration for di in d]
        qdd = [di * dds / seg_duration ** 2 for di in d]
        return q, qd, qdd

    def estimate_route_energy(self, start, waypoints, total_duration, strategy, samples=40):
        """Rotayı oynatıcının izleyeceği şekilde (eşit süreli segmentler, aynı hız profili) örnekler; her an
        için gereken eklem torkunu ters dinamikle (robot ve yük kütleleriyle) hesaplar. -> (enerji J, tepe güç W)"""
        robot, n = self.sim_engine.robotId, len(self.sim_engine.revolute_joints)
        self.last_peak_torques = [0.0] * n
        self.last_segment_energy = []
        if robot is None or not waypoints or total_duration <= 0 or not self.telemetry.dynamics_ok: return 0.0, 0.0
        seg = total_duration / len(waypoints)
        energy, peak, prev = 0.0, 0.0, list(start)[:n]
        with self.sim_engine.bullet_lock:
            for wp in waypoints:
                seg_energy = 0.0
                for k in range(samples):
                    q, qd, qdd = self.profile_state(prev, wp, seg, strategy, (k + 0.5) / samples)
                    tau = self.telemetry.joint_torques(q, qd, qdd)
                    power = self.telemetry.electrical_power(qd, tau)
                    energy += power * seg / samples
                    seg_energy += power * seg / samples
                    peak = max(peak, power)
                    self.last_peak_torques = [max(a, abs(b)) for a, b in zip(self.last_peak_torques, tau)]
                prev = list(wp)
                self.last_segment_energy.append(seg_energy)
        return energy, peak

    def calculate_ik(self, x_val, y_val, z_val, current_joint_angles):
        x_cm = float(x_val) if x_val else (self.robot_reach_cm * 0.45)
        y_cm = float(y_val) if y_val else 0.0
        z_cm = float(z_val) if z_val else (self.base_z_cm + 15.0)

        math_log = "\n\n" + "=" * 52 + "\n"
        math_log += t("ik.title") + "\n"

        min_z_req = (self.stand_top_cm + 5.0) if (self.stand_half_extent_cm > 0 and abs(x_cm) <= self.stand_half_extent_cm + 5.0 and abs(y_cm) <= self.stand_half_extent_cm + 5.0) else 0.0
        if z_cm < min_z_req:
            math_log += t("ik.below_floor", z=f"{z_cm:.1f}", min=f"{min_z_req:.1f}") + "\n\n"
            z_cm = min_z_req
            
        x_m, y_m, z_m = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
        self.pending_waypoints = []
        # IK araması her waypoint için güvenlik kontrolünü zaten yapıyor; sonuç burada tutulur,
        # rapor ve Manevra butonları tekrar hesaplamaz.
        self.pending_waypoint_safety = []
        n_joints = len(self.sim_engine.revolute_joints)
        prismatic = self.prismatic_joint_flags()
        ee_chain = self.ee_chain_joints()
        # TCP'yi etkilemeyen eklemler (ör. gripper parmakları) IK'da mevcut değerinde kalır;
        # aksi halde IK onları rest pozuna çeker ve tutulan parça düşebilir.
        locked = [k for k, j_idx in enumerate(self.sim_engine.revolute_joints)
                  if j_idx not in ee_chain and k < len(current_joint_angles)]

        def solve_ik_pose(target_pos, seed=None):
            ik_kwargs = {
                'maxNumIterations': 400,
                'residualThreshold': 1e-4
            }
            if len(self.joint_lower_limits) == n_joints:
                ik_kwargs['lowerLimits'] = self.joint_lower_limits
                ik_kwargs['upperLimits'] = self.joint_upper_limits
                ik_kwargs['jointRanges'] = self.joint_ranges
                ik_kwargs['restPoses'] = self.joint_rest_poses

            yaw_base = math.atan2(target_pos[1], target_pos[0]) if (target_pos[0] != 0 or target_pos[1] != 0) else 0.0
            candidate_seeds = [
                seed if seed is not None else current_joint_angles,
                [yaw_base, -0.4, 0.4] + [0.0] * max(0, n_joints - 3),
                [yaw_base, -0.7, 0.8] + [0.0] * max(0, n_joints - 3),
                [yaw_base, 0.0, 0.0] + [0.0] * max(0, n_joints - 3),
                [yaw_base, 0.3, -0.3] + [0.0] * max(0, n_joints - 3),
                [0.0] * n_joints
            ]

            best_angles, best_err, best_safe, best_desc = None, float('inf'), False, ""

            with self.sim_engine.bullet_lock:
                orig = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
                for seed in candidate_seeds:
                    for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                        if i < len(seed):
                            p.resetJointState(self.sim_engine.robotId, j_idx, seed[i])

                    sol = p.calculateInverseKinematics(
                        self.sim_engine.robotId, getattr(self.sim_engine, 'end_effector_index', 0), target_pos, **ik_kwargs
                    )
                    angles = list(sol[:n_joints])
                    if len(self.joint_lower_limits) == n_joints:
                        for k in range(n_joints):
                            low = self.joint_lower_limits[k]
                            high = self.joint_upper_limits[k]
                            if low < high:
                                angles[k] = max(low, min(high, angles[k]))
                    for k in locked:
                        angles[k] = current_joint_angles[k]

                    for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                        p.resetJointState(self.sim_engine.robotId, j_idx, angles[i])
                        
                    ee_pos = p.getLinkState(self.sim_engine.robotId, getattr(self.sim_engine, 'end_effector_index', 0))[4]
                    err = math.sqrt(sum((a - b) ** 2 for a, b in zip(ee_pos, target_pos)))
                    is_safe, desc = self.check_angles_safety(angles)
                    
                    if is_safe and err < 0.03:
                        best_angles = angles
                        best_err = err
                        best_safe = True
                        best_desc = desc
                        break
                        
                    if not best_angles or (is_safe and not best_safe) or (is_safe == best_safe and err < best_err):
                        best_angles = angles
                        best_err = err
                        best_safe = is_safe
                        best_desc = desc
                    
                for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                    p.resetJointState(self.sim_engine.robotId, j_idx, orig[i][0], orig[i][1])

            return best_angles, best_err, best_safe, best_desc

        def route_safety(route):
            """Her waypoint için (güvenli, açıklama): hem duruşun kendisi hem önceki noktadan oraya giden yol."""
            result, prev = [], current_joint_angles
            for angles, _, safe, desc in route:
                if safe:
                    safe, desc = self.check_path_safety(prev, angles)
                result.append((safe, desc))
                prev = angles
            return result

        with self.sim_engine.bullet_lock:
            final = solve_ik_pose([x_m, y_m, z_m])
            final_angles, final_err, final_safe, final_desc = final
            route, route_checks = [final], route_safety([final])
            labels = ["route.target"]

            if self.sim_engine.obstacle_id is not None:
                if all(ok for ok, _ in route_checks):
                    math_log += t("ik.direct_clear") + "\n\n"
                elif not final_safe:
                    # Hedef duruşun kendisi çarpışıyorsa hiçbir kaçış rotası işe yaramaz
                    math_log += t("ik.target_collides", desc=final_desc) + "\n\n"
                else:
                    # Doğrudan rota engele değiyor: önce yukarı, engelin üstünden hedefin üstüne, sonra aşağı.
                    # Tüm rota temiz çıkana kadar irtifa artırılır (tepe + 15/30/45/60 cm, model ölçeğinde).
                    state = p.getLinkState(self.sim_engine.robotId, getattr(self.sim_engine, 'end_effector_index', 0))
                    cx, cy, cz = state[4]
                    obstacle_top = self.sim_engine.obstacle_pos[2] + self.sim_engine.obstacle_half[2]
                    found_z = None
                    yaw0, yaw1 = math.atan2(cy, cx), math.atan2(y_m, x_m)
                    d_yaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
                    r0, r1 = math.hypot(cx, cy), math.hypot(x_m, y_m)
                    n_arc = max(1, math.ceil(abs(d_yaw) / math.radians(30)))
                    for k in range(1, 5):
                        safe_z = max(cz, z_m, obstacle_top + 0.15 * k * self.sim_engine.obstacle_scale)
                        # Yukarı çık, bu irtifada taban etrafında yay çizerek (~30°'de bir ara nokta) hedefin
                        # üstüne git, sonra in. İki uç nokta arasında eklem uzayında doğrudan gitmek ön kolu
                        # engele indirebiliyordu. Her nokta öncekinden başlatılır ki dirsek yön değiştirmesin.
                        points = [[cx, cy, safe_z]]
                        for i in range(1, n_arc):
                            yaw, r = yaw0 + d_yaw * i / n_arc, r0 + (r1 - r0) * i / n_arc
                            points.append([r * math.cos(yaw), r * math.sin(yaw), safe_z])
                        points.append([x_m, y_m, safe_z])

                        candidate, checks, prev, ok = [], [], current_joint_angles, True
                        for pt in points:
                            wp = solve_ik_pose(pt, seed=prev)
                            safe, desc = wp[2], wp[3]
                            if safe: safe, desc = self.check_path_safety(prev, wp[0])
                            candidate.append(wp); checks.append((safe, desc)); prev = wp[0]
                            if not safe: ok = False; break
                        if ok:
                            safe, desc = final_safe, final_desc
                            if safe: safe, desc = self.check_path_safety(prev, final_angles)
                            candidate.append(final); checks.append((safe, desc)); ok = safe
                        if ok:
                            route, route_checks, found_z = candidate, checks, safe_z
                            labels = (["route.up"] + [f"route.arc:{i}" for i in range(1, len(points) - 1)]
                                      + ["route.above", "route.target"])
                            break
                    if found_z is not None:
                        math_log += t("ik.detour", z=f"{found_z * 100:.0f}") + "\n\n"
                    else:
                        math_log += t("ik.no_route") + "\n\n"

            self.pending_waypoints = [wp[0] for wp in route]
            self.pending_waypoint_labels = labels if len(labels) == len(route) else [f"route.step:{i + 1}" for i in range(len(route))]
            self.pending_waypoint_safety = route_checks

        deg_diffs = [abs(math.degrees(self.pending_waypoints[-1][i] - current_joint_angles[i])) for i in range(min(n_joints, len(current_joint_angles)))
                     if not (i < len(prismatic) and prismatic[i])]
        max_deg = max(deg_diffs) if deg_diffs else 0

        rated_capacity = max(3.0, (self.robot_reach_cm / 70.0) * 5.0)
        payload = self.telemetry.payload_kg
        load_ratio = (payload / rated_capacity) if rated_capacity > 0 else 0.0
        duration_factor = math.sqrt(1.0 + 1.2 * load_ratio + 0.5 * (load_ratio ** 2))

        time_base = max(0.5, max_deg / 100.0)
        self.time_fast = time_base * duration_factor
        if len(self.pending_waypoints) > 1: self.time_fast *= 1.5
        self.time_eco = self.time_fast * 1.8

        eff = self.telemetry.motor_efficiency
        tariff = self.telemetry.electricity_rate
        standby_total_w = self.telemetry.motor_standby_w * n_joints

        # Enerji: rota iki hız profiliyle ters dinamikten (robot + yük kütlesi) hesaplanır
        energy_fast_joules, self.peak_power_fast = self.estimate_route_energy(current_joint_angles, self.pending_waypoints, self.time_fast, "fast")
        peak_tau_fast = list(self.last_peak_torques)
        self.route_segment_energy = list(self.last_segment_energy)
        energy_eco_joules, self.peak_power_eco = self.estimate_route_energy(current_joint_angles, self.pending_waypoints, self.time_eco, "eco")
        peak_tau_eco = list(self.last_peak_torques)
        cost_fast_tl = energy_fast_joules / 3.6e6 * tariff
        cost_eco_tl = energy_eco_joules / 3.6e6 * tariff

        math_log += t("ik.step1", x=f"{x_cm:.1f}", y=f"{y_cm:.1f}", z=f"{z_cm:.1f}") + "\n\n"
        math_log += t("ik.step2") + "\n"
        if final_err <= 0.03:
            math_log += t("ik.err_ok", cm=f"{final_err * 100.0:.2f}") + "\n\n"
        else:
            math_log += t("ik.err_bad", cm=f"{final_err * 100.0:.2f}") + "\n\n"
            
        math_log += t("ik.step3") + "\n"
        if payload <= 0.0:
            math_log += t("ik.payload_none") + "\n"
        else:
            math_log += t("ik.payload", kg=f"{payload:.2f}", cap=f"{rated_capacity:.1f}", ratio=f"{load_ratio*100:.0f}") + "\n"
        math_log += t("ik.robot_mass", kg=f"{self.telemetry.robot_mass_kg:.1f}", source=t(self.telemetry.mass_source)) + "\n"
        math_log += t("ik.motor", eff=f"{eff*100:.0f}", tariff=f"{tariff:.2f}", standby=f"{standby_total_w:.0f}") + "\n"
        if self.telemetry.dynamics_ok:
            math_log += t("ik.energy_model") + "\n\n"
        else:
            math_log += t("ik.no_dynamics") + "\n\n"
        math_log += t("ik.fast_header") + "\n"
        math_log += t("ik.time", s=f"{self.time_fast:.2f}") + "\n"
        math_log += t("ik.energy", wh=f"{energy_fast_joules / 3600:.3f}", peak=f"{self.peak_power_fast:.0f}") + "\n"
        math_log += t("ik.cost", tl=f"{cost_fast_tl:.5f}") + "\n\n"
        math_log += t("ik.eco_header") + "\n"
        math_log += t("ik.time", s=f"{self.time_eco:.2f}") + "\n"
        math_log += t("ik.energy", wh=f"{energy_eco_joules / 3600:.3f}", peak=f"{self.peak_power_eco:.0f}") + "\n"
        math_log += t("ik.cost", tl=f"{cost_eco_tl:.5f}") + "\n"
        if cost_eco_tl < cost_fast_tl:
            math_log += t("ik.saving_eco", pct=f"{(1 - cost_eco_tl / max(1e-12, cost_fast_tl)) * 100:.1f}") + "\n\n"
        else:
            math_log += t("ik.saving_fast", pct=f"{(1 - cost_fast_tl / max(1e-12, cost_eco_tl)) * 100:.1f}") + "\n\n"

        # Girilen motor verisiyle (toplam güç / eksen etiketi) tork sınırı aşılıyor mu
        torque_warnings = []
        for k, spec in enumerate(self.telemetry.motor_specs):
            if not spec["real_limit"] or k >= len(peak_tau_fast): continue
            unit = "N" if spec["linear"] else "N·m"
            if peak_tau_eco[k] > spec["limit"]:
                torque_warnings.append(t("ik.torque_both", j=k + 1, need=f"{peak_tau_eco[k]:.2f}", limit=f"{spec['limit']:.2f}", unit=unit))
            elif peak_tau_fast[k] > spec["limit"]:
                torque_warnings.append(t("ik.torque_fast", j=k + 1, need=f"{peak_tau_fast[k]:.2f}", limit=f"{spec['limit']:.2f}", unit=unit))
        if torque_warnings:
            math_log += t("ik.torque_section") + "\n" + "\n".join(torque_warnings) + "\n\n"

        math_log += t("ik.step4") + "\n"
        has_collision_risk = False
        collision_warning_text = ""
        for idx, (is_safe, desc) in enumerate(self.pending_waypoint_safety, 1):
            if is_safe:
                math_log += t("ik.wp_safe", n=idx) + "\n"
            else:
                has_collision_risk = True
                collision_warning_text = desc
                math_log += t("ik.wp_unsafe", n=idx, desc=desc) + "\n"

        math_log += "=" * 52 + "\n"
        return math_log, has_collision_risk, final_err, collision_warning_text

    def toggle_workspace(self):
        with self.sim_engine.bullet_lock:
            self.show_workspace = not self.show_workspace
            self.trail_segments = []
            self.publish_trail()
            if self.show_workspace:
                self.update_link_positions()
        return self.show_workspace

    def clear_trail(self):
        """İzi siler, iz açıksa çizmeye devam eder."""
        with self.sim_engine.bullet_lock:
            self.trail_segments = []
            self.publish_trail()
            if self.show_workspace:
                self.update_link_positions()

    def publish_trail(self):
        gpu_state, gpu_lock = self.sim_engine.gpu_shared_state, self.sim_engine.gpu_state_lock
        if gpu_state is None or gpu_lock is None: return
        # Her değişimde yeni tuple: renderer değişikliği kimlikten anlar
        with gpu_lock:
            gpu_state['trail'] = tuple(self.trail_segments)

    def update_link_positions(self):
        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is not None:
                for joint_idx in self.sim_engine.revolute_joints:
                    state = p.getLinkState(self.sim_engine.robotId, joint_idx)
                    self.last_link_positions[joint_idx] = state[4]

    def draw_swept_volume(self):
        if not self.show_workspace: return
        added = False

        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is None or len(self.sim_engine.revolute_joints) < 2: return

            for joint_idx in [self.sim_engine.revolute_joints[-2], self.sim_engine.revolute_joints[-1]]:
                state = p.getLinkState(self.sim_engine.robotId, joint_idx)
                current_pos = state[4]
                last_pos = self.last_link_positions.get(joint_idx, current_pos)

                dist = math.sqrt((current_pos[0] - last_pos[0]) ** 2 +
                                 (current_pos[1] - last_pos[1]) ** 2 +
                                 (current_pos[2] - last_pos[2]) ** 2)

                if dist > 0.015:
                    mid_x = (current_pos[0] + last_pos[0]) / 2.0
                    mid_y = (current_pos[1] + last_pos[1]) / 2.0
                    mid_z = (current_pos[2] + last_pos[2]) / 2.0

                    diff = [current_pos[0] - last_pos[0], current_pos[1] - last_pos[1], current_pos[2] - last_pos[2]]
                    pitch = -math.asin(diff[2] / dist) if dist != 0 else 0
                    yaw = math.atan2(diff[1], diff[0])
                    rot = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, pitch, yaw]))

                    # [merkez(3), rotasyon matrisi(9, satır öncelikli), yarı boyutlar(3)]
                    self.trail_segments.append((mid_x, mid_y, mid_z, *rot, dist / 2, 0.02, 0.01))
                    self.last_link_positions[joint_idx] = current_pos
                    added = True

            if added:
                del self.trail_segments[:-MAX_TRAIL_SEGMENTS]
                self.publish_trail()
