import pybullet as p
import math
import time
import threading

# İz segmentleri pybullet gövdesi değil, GPU renderer'a giden düz veridir.
# Sınırsız büyümemesi için en eski segmentler atılır.
MAX_TRAIL_SEGMENTS = 4000

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
        self.time_fast = 0.0
        self.time_eco = 0.0
        self.peak_power_fast = 0.0
        self.peak_power_eco = 0.0

    def calculate_fk(self):
        """Calculates Forward Kinematics and returns formatted log text."""
        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is None: return "Hata: Robot yüklü değil."
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
        fk_log = "\n\n" + "═" * 50 + "\n"
        fk_log += "   NİHAİ UÇ İŞLEVCİ (TCP) İLERİ KİNEMATİK RAPORU\n"
        fk_log += "═" * 50 + "\n\n"
        
        fk_log += "■ 1. MEVCUT EKLEM AÇILARI\n"
        fk_log += " ──────────────────────────────────────────\n"
        fk_log += "  Eksen | Derece (°) | Radyan (rad)\n"
        fk_log += " ──────────────────────────────────────────\n"
        for i, val in enumerate(current_joints):
            fk_log += f"   J{i+1:<3} | {math.degrees(val):>8.2f}° | {val:>8.4f}\n"
        fk_log += " ──────────────────────────────────────────\n\n"

        fk_log += "■ 2. UZAYDAKİ SON KONUM (X, Y, Z)\n"
        fk_log += " ──────────────────────────────────────────\n"
        fk_log += f"  X Ekseni (İleri/Geri)  : {T[0][3]:8.2f} cm\n"
        fk_log += f"  Y Ekseni (Sağ/Sol)     : {T[1][3]:8.2f} cm\n"
        fk_log += f"  Z Ekseni (Yukarı/Aşağı): {T[2][3]:8.2f} cm\n"
        fk_log += " ──────────────────────────────────────────\n\n"

        fk_log += "■ 3. YÖNELİM VE DÖNÜŞÜM MATRİSİ (TCP Pose)\n"
        fk_log += "  Rotasyon (X)   Rotasyon (Y)   Rotasyon (Z) | Pozisyon\n"
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
                            return False, f"Gövde Çarpışması ({nameA} <-> {nameB})"

                # 2. Stand & Ground
                for j_idx in self.sim_engine.revolute_joints:
                    ls = p.getLinkState(self.sim_engine.robotId, j_idx)
                    lx = ls[4][0] * 100.0
                    ly = ls[4][1] * 100.0
                    lz = ls[4][2] * 100.0
                    if self.stand_half_extent_cm > 0 and abs(lx) <= self.stand_half_extent_cm and abs(ly) <= self.stand_half_extent_cm:
                        if lz < self.stand_top_cm:
                            name = self.link_names.get(j_idx, f"Link_{j_idx}")
                            return False, f"Kaideye Çarpma ({name})"
                    elif lz < 2.0:
                        name = self.link_names.get(j_idx, f"Link_{j_idx}")
                        return False, f"Zemine Çarpma ({name})"

                # 3. Dynamic Obstacle
                if self.sim_engine.obstacle_id is not None:
                    obs_contacts = p.getContactPoints(self.sim_engine.robotId, self.sim_engine.obstacle_id)
                    real_obs = [c for c in obs_contacts if c[8] < 0.0]
                    if real_obs:
                        c = real_obs[0]
                        link_hit = self.link_names.get(c[3], f"Link_{c[3]}")
                        return False, f"Engelle Çarpışma ({link_hit})"

                return True, "Güvenli (Çarpışmasız)"
            finally:
                for i, j_idx in enumerate(self.sim_engine.revolute_joints):
                    p.resetJointState(self.sim_engine.robotId, j_idx, original_states[i][0], original_states[i][1])

    def calculate_ik(self, x_val, y_val, z_val, current_joint_angles):
        x_cm = float(x_val) if x_val else (self.robot_reach_cm * 0.45)
        y_cm = float(y_val) if y_val else 0.0
        z_cm = float(z_val) if z_val else (self.base_z_cm + 15.0)

        math_log = "\n\n" + "=" * 52 + "\n"
        math_log += "=== TERS KİNEMATİK (IK) MATEMATİKSEL ÇÖZÜMÜ ===\n"

        min_z_req = (self.stand_top_cm + 5.0) if (self.stand_half_extent_cm > 0 and abs(x_cm) <= self.stand_half_extent_cm + 5.0 and abs(y_cm) <= self.stand_half_extent_cm + 5.0) else 0.0
        if z_cm < min_z_req:
            math_log += f">> [!] GÜVENLİK KORUMASI: Hedef Z ({z_cm:.1f} cm) zemin/kaide seviyesinin altında!\n"
            math_log += f"    Z={min_z_req:.1f} cm güvenli yüksekliğe otomatik çekildi.\n\n"
            z_cm = min_z_req
            
        x_m, y_m, z_m = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
        self.pending_waypoints = []
        n_joints = len(self.sim_engine.revolute_joints)

        def solve_ik_pose(target_pos):
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
                current_joint_angles,
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

        with self.sim_engine.bullet_lock:
            if self.sim_engine.obstacle_id is not None:
                state = p.getLinkState(self.sim_engine.robotId, getattr(self.sim_engine, 'end_effector_index', 0))
                cx, cy, cz = state[4]
                safe_z = max(cz, z_m, self.sim_engine.obstacle_pos[2] + 0.35)

                wp1, err1, safe1, desc1 = solve_ik_pose([cx, cy, safe_z])
                self.pending_waypoints.append(wp1)

                wp2, err2, safe2, desc2 = solve_ik_pose([x_m, y_m, safe_z])
                self.pending_waypoints.append(wp2)

                math_log += f">> OTONOM KAÇIŞ AKTİF: Engel Algılandı.\n"
                math_log += f">> {safe_z * 100:.0f}cm irtifadan U-Dönüş rotası çizildi.\n\n"

            final_angles, final_err, final_safe, final_desc = solve_ik_pose([x_m, y_m, z_m])
            self.pending_waypoints.append(final_angles)

        deg_diffs = [abs(math.degrees(self.pending_waypoints[-1][i] - current_joint_angles[i])) for i in range(min(n_joints, len(current_joint_angles)))]
        max_deg = max(deg_diffs) if deg_diffs else 0

        rated_capacity = max(3.0, (self.robot_reach_cm / 70.0) * 5.0)
        payload = self.telemetry.payload_kg
        load_ratio = (payload / rated_capacity) if rated_capacity > 0 else 0.0
        duration_factor = math.sqrt(1.0 + 1.2 * load_ratio + 0.5 * (load_ratio ** 2))

        time_base = max(0.5, max_deg / 100.0)
        self.time_fast = time_base * duration_factor
        if self.sim_engine.obstacle_id is not None: self.time_fast *= 1.5
        self.time_eco = self.time_fast * 1.8

        eff = self.telemetry.motor_efficiency
        tariff = self.telemetry.electricity_rate
        standby_total_w = self.telemetry.motor_standby_w * n_joints

        load_power_mult = 1.0 + (0.75 * load_ratio)
        self.peak_power_fast = (max_deg * 12.0 + 50.0) * load_power_mult
        self.peak_power_eco = self.peak_power_fast * 0.38

        p_avg_fast_elec = (self.peak_power_fast * 0.55 / eff) + standby_total_w
        energy_fast_joules = p_avg_fast_elec * self.time_fast
        cost_fast_kwh = energy_fast_joules / 3.6e6
        cost_fast_tl = cost_fast_kwh * tariff

        p_avg_eco_elec = (self.peak_power_eco * 0.42 / eff) + standby_total_w
        energy_eco_joules = p_avg_eco_elec * self.time_eco
        cost_eco_kwh = energy_eco_joules / 3.6e6
        cost_eco_tl = cost_eco_kwh * tariff
        saving_pct = max(0.0, (1.0 - (cost_eco_tl / max(1e-6, cost_fast_tl))) * 100.0)

        math_log += f">> Adım 1: Hedef Uzay Vektörü (P_hedef)\n   P = [ {x_cm:.1f} cm, {y_cm:.1f} cm, {z_cm:.1f} cm ]^T\n\n"
        math_log += f">> Adım 2: Jacobian Ters Matris & Açı Sınırları (Constrained IK)\n"
        if final_err <= 0.03:
            math_log += f"   Hedef Doğrulama Sapması: {final_err * 100.0:.2f} cm (Mükemmel Hassasiyet)\n\n"
        else:
            math_log += f"   [!] Hedef Doğrulama Sapması: {final_err * 100.0:.2f} cm (Erişim sınırının dışında)\n\n"
            
        math_log += ">> Adım 3: DİNAMİK YÖRÜNGE, MOTOR YÜKÜ VE ELEKTRİK MALİYETİ\n"
        if payload <= 0.0:
            math_log += f"  - Uç Yükü (Payload) : 0.0 kg (Yüksüz / Nominal motor hızı ve torku)\n"
        else:
            math_log += f"  - Uç Yükü (Payload) : {payload:.2f} kg (Kapasite: {rated_capacity:.1f} kg | Yük Oranı: %{load_ratio*100:.0f})\n"
        
        math_log += f"  - Motor Verimi      : %{eff*100:.0f} | Birim Elektrik: {tariff:.2f} TL/kWh | Bekleme: {standby_total_w:.0f}W\n\n"
        math_log += " [ Strateji 1: Hızlı Rota (Trapezoidal) ]\n"
        math_log += f"  • Tahmini Süre : {self.time_fast:.2f} sn\n"
        math_log += f"  • Tahmini Maliyet : {cost_fast_tl:.4f} TL\n\n"
        math_log += " [ Strateji 2: Ekonomik Rota (S-Eğrisi) ]\n"
        math_log += f"  • Tahmini Süre : {self.time_eco:.2f} sn\n"
        math_log += f"  • Tahmini Maliyet : {cost_eco_tl:.4f} TL\n"
        math_log += f"  >> TASARRUF FARKI : Ekonomik rota ile %{saving_pct:.1f} daha az elektrik faturası!\n\n"

        math_log += ">> Adım 4: SANAL GÜVENLİK & ÇARPIŞMA ANALİZİ\n"
        has_collision_risk = False
        collision_warning_text = ""
        for idx, wp in enumerate(self.pending_waypoints, 1):
            is_safe, desc = self.check_angles_safety(wp)
            if is_safe:
                math_log += f"  [✓] Manevra {idx}: Güvenli (Çarpışmasız rota)\n"
            else:
                has_collision_risk = True
                collision_warning_text = desc
                math_log += f"  [!] UYARI Manevra {idx}: TEHLİKE -> {desc}!\n"

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
