import math
import random
import time
import pybullet as p
from i18n import t

# Motorun nominal torkunda bakır (I²R) kaybı, nominal mekanik gücünün bu oranı kabul edilir.
# Duran bir kolun yük tutarken harcadığı güç bu terimden gelir (hız sıfırken |tork x hız| = 0).
COPPER_LOSS_AT_RATED = 0.15
# Servo motorlar kısa süre nominal torkunun bu katını verebilir (tepe tork)
SERVO_PEAK_FACTOR = 3.0
KGCM_TO_NM = 0.0980665
KG_TO_N = 9.80665

def _parse_float(text, default=None):
    try:
        v = float(str(text).strip().replace(',', '.'))
        return v if v > 0 else default
    except Exception:
        return default

class TelemetryEngine:
    def __init__(self, sim_engine):
        self.sim_engine = sim_engine

        self.payload_kg = 0.0
        self.motor_efficiency = 0.85
        self.motor_standby_w = 15.0
        self.electricity_rate = 4.50

        # Kütle modeli: model yüklendiğindeki link kütleleri, kullanıcının girdiği robot ağırlığına ölçeklenir
        self.robot_mass_kg = 0.0
        self.robot_mass_default = 0.0
        self.mass_source = ""
        self._base_dynamics = {}   # link -> (kütle, atalet köşegeni)
        self._root_mass = 0.0
        self.copper_k = []         # eklem başına bakır kaybı katsayısı, W / (Nm)²
        # Motor verisi kaynağı: "auto" (ağırlıktan), "total" (robotun toplam gücü), "labels" (eksen etiketleri)
        self.motor_mode = "auto"
        self.total_rated_w = 0.0
        self.axis_labels = []
        self.motor_specs = []      # eksen başına: tip, tork sınırı, bakır katsayısı, sabit güç, gerçek sınır mı
        self._stall_since = {}
        self.dynamics_ok = False   # ters dinamik hesaplanabiliyor mu (bazı URDF ataletlerini Bullet reddeder)
        self._ee_local = [0.0, 0.0, 0.0]
        self._playback = None      # (zaman, hızlar, torklar): IK oynatılırken fizik yerine ters dinamikten

        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0
        self.current_power = 0.0
        self.power_history = [0.0] * 100
        # Grafik döngü hızından bağımsız olsun: 100 örnek x 1/30 sn = son ~3.3 sn
        self.history_interval = 1.0 / 30.0
        self._history_elapsed = 0.0

    def on_robot_loaded(self, stand_link_idx, mass_info):
        """RobotLoader yeni model yükleyince çağırır. Kaide (stand) robot ağırlığına dahil edilmez."""
        robot = self.sim_engine.robotId
        self._base_dynamics = {}
        with self.sim_engine.bullet_lock:
            for li in range(p.getNumJoints(robot)):
                if li == stand_link_idx: continue
                info = p.getDynamicsInfo(robot, li)
                self._base_dynamics[li] = (info[0], list(info[2]))
        self._root_mass = mass_info.get("root_mass", 0.0)
        self.robot_mass_default = self._root_mass + sum(m for m, _ in self._base_dynamics.values())
        self.robot_mass_kg = self.robot_mass_default
        # Jacobian, link kütle merkezine göre verilen noktada hesaplanır; yük uç çerçevesinin orijininde
        ee = getattr(self.sim_engine, 'end_effector_index', None)
        if ee is not None and ee >= 0:
            with self.sim_engine.bullet_lock:
                self._ee_local = [-v for v in p.getLinkState(robot, ee)[2]]

        est = mass_info.get("estimated_mass", 0.0)
        if est <= 0.0:
            self.mass_source = "mass.urdf"
        elif est >= 0.5 * self.robot_mass_default:
            self.mass_source = "mass.estimated"
        else:
            self.mass_source = "mass.partial"
        self.apply_mass_model()

    def apply_mass_model(self):
        """Link kütlelerini robot ağırlığına ölçekler, motorları buna göre kalibre eder, yükü uca ekler."""
        robot = self.sim_engine.robotId
        if robot is None or not self._base_dynamics: return
        f = self.robot_mass_kg / self.robot_mass_default if self.robot_mass_default > 0 else 1.0
        ee = getattr(self.sim_engine, 'end_effector_index', None)
        with self.sim_engine.bullet_lock:
            for li, (m, inertia) in self._base_dynamics.items():
                p.changeDynamics(robot, li, mass=m * f, localInertiaDiagonal=[v * f for v in inertia])
            # Motorlar robotun kendi ağırlığına göre boyutlanır, yüke göre değil
            self.calibrate_motors()
            if ee in self._base_dynamics and self.payload_kg > 0:
                m, inertia = self._base_dynamics[ee]
                # Yük uç çerçevesinde nokta kütle kabul edilir
                p.changeDynamics(robot, ee, mass=m * f + self.payload_kg,
                                 localInertiaDiagonal=[v * f + 1e-4 * self.payload_kg for v in inertia])

    @property
    def mass_factor(self):
        return self.robot_mass_kg / self.robot_mass_default if self.robot_mass_default > 0 else 1.0

    def joint_torques(self, q, qd, qdd, with_payload=True):
        """Verilen hareket için gereken eklem torkları (yerçekimi + ivme + Coriolis).
        pybullet ters dinamik modelini ilk çağrıda kurup önbelleğe alır, sonraki changeDynamics değişikliklerini
        görmez. Bu yüzden model yüklendiğindeki kütlelerle hesaplanır ve robot ağırlığı oranıyla ölçeklenir
        (dinamik, kütlelere doğrusal bağlı). Yük uç çerçevesinde nokta kütle olarak Jacobian'la eklenir."""
        robot = self.sim_engine.robotId
        n = len(q)
        if robot is None or not self.dynamics_ok: return [0.0] * n
        f = self.mass_factor
        tau = [t * f for t in p.calculateInverseDynamics(robot, list(q), list(qd), list(qdd))]
        ee = getattr(self.sim_engine, 'end_effector_index', None)
        if with_payload and self.payload_kg > 0 and ee is not None and ee >= 0:
            zero = [0.0] * n
            lin, _ = p.calculateJacobian(robot, ee, self._ee_local, list(q), zero, zero)
            acc = [sum(lin[r][k] * qdd[k] for k in range(n)) for r in range(3)]   # J̇·q̇ terimi ihmal
            force = [self.payload_kg * (acc[0]), self.payload_kg * (acc[1]), self.payload_kg * (acc[2] + 9.81)]
            tau = [t + sum(lin[r][k] * force[r] for r in range(3)) for k, t in enumerate(tau)]
        return tau

    def apply_motor_settings(self, settings):
        """Güç ve Enerji sayfasındaki motor verisi; bir sonraki apply_mass_model/calibrate_motors'ta kullanılır."""
        self.motor_mode = settings.get("mode", "auto")
        self.total_rated_w = _parse_float(settings.get("total_power"), 0.0)
        try:
            standby = str(settings.get("standby", "")).strip().replace(',', '.')
            self.motor_standby_w = max(0.0, float(standby)) if standby else 15.0
        except Exception:
            self.motor_standby_w = 15.0
        self.axis_labels = []
        for ax in settings.get("axes", []):
            torque = _parse_float(ax.get("torque"))
            if torque is not None:
                torque *= KG_TO_N if ax.get("linear") else KGCM_TO_NM
            self.axis_labels.append({
                "kind": "stepper" if ax.get("kind") == "Step" else "servo",
                "power": _parse_float(ax.get("power")),
                "torque": torque,                          # motor milinde, N·m (lineer eksende N)
                "gear": _parse_float(ax.get("gear"), 1.0),
            })

    def calibrate_motors(self):
        """Her eklem için referans tork: rastgele duruşlarda yerçekimi torkunun en büyüğünün 2 katı.
        Otomatik modda motor bundan boyutlanır; bakır kaybı katsayısı k = COPPER_LOSS_AT_RATED x nominal güç /
        nominal tork²: nominal torkta tutmak nominal gücün %15'i kadar elektrik çeker. Toplam güç ve eksen
        etiketi modlarında nominal güç/tork kullanıcının girdiği değerlerden gelir."""
        robot, joints = self.sim_engine.robotId, self.sim_engine.revolute_joints
        n = len(joints)
        if robot is None or n == 0: return
        limits, w_max = [], []
        for j in joints:
            info = p.getJointInfo(robot, j)
            low, high = float(info[8]), float(info[9])
            limits.append((low, high) if low < high else (-math.pi, math.pi))
            w_max.append(float(info[11]) if info[11] > 0 else 3.0)
        rng = random.Random(0)
        gravity = [0.0] * n
        # İlk çağrı ters dinamik modelini kurar: kütleler burada henüz model yüklendiği hâlde olmalı
        self.dynamics_ok = True
        try:
            for _ in range(48):
                q = [rng.uniform(lo, hi) for lo, hi in limits]
                tau = self.joint_torques(q, [0.0] * n, [0.0] * n, with_payload=False)
                for k in range(n): gravity[k] = max(gravity[k], abs(tau[k]))
        except Exception as e:
            # Ters dinamik yoksa enerji sadece fizikten (mekanik güç + bekleme) hesaplanır, tahmin yapılamaz
            print(f"[Dinamik] Ters dinamik hesaplanamadı, URDF atalet değerlerini kontrol edin: {e}")
            self.dynamics_ok = False
            self.copper_k = []
            self.motor_specs = []
            self.sim_engine.motor_forces = []
            return
        tau_ref = [2.0 * g for g in gravity]
        # Dikey eksenli J1 gibi yerçekimi torku ~0 olan eklemler de gerçekte boyutlu bir motor taşır
        floor = max(0.5, 0.1 * max(tau_ref))
        tau_ref = [max(t, floor) for t in tau_ref]
        linear = [p.getJointInfo(robot, j)[2] == p.JOINT_PRISMATIC for j in joints]

        def servo(k, power, tau_rated, real):
            return {"kind": "servo", "limit": SERVO_PEAK_FACTOR * tau_rated, "copper_k": COPPER_LOSS_AT_RATED * power / tau_rated ** 2,
                    "constant_w": 0.0, "real_limit": real, "linear": linear[k]}

        def auto(k):
            # Ağır robotlar sabit 800 Nm ile kendi ağırlığını taşıyamaz; hiçbir eklem eskisinden zayıf olmaz
            spec = servo(k, tau_ref[k] * w_max[k], tau_ref[k], False)
            spec["limit"] = max(800.0, spec["limit"])
            return spec

        specs = []
        if self.motor_mode == "total" and self.total_rated_w > 0:
            # Toplam güç, eksenlere gereken güç (referans tork x maks. hız) oranında dağıtılır
            weights = [t * w for t, w in zip(tau_ref, w_max)]
            for k in range(n):
                power = self.total_rated_w * weights[k] / sum(weights)
                specs.append(servo(k, power, power / w_max[k], True))
        elif self.motor_mode == "labels":
            for k in range(n):
                ax = self.axis_labels[k] if k < len(self.axis_labels) else None
                if ax is None or (ax["power"] is None and ax["torque"] is None):
                    specs.append(auto(k)); continue
                tau_joint = ax["torque"] * ax["gear"] if ax["torque"] is not None else None
                if ax["kind"] == "stepper":
                    # Step sürücüsü yükten bağımsız sabit akım verir: etiketteki güç sürekli harcanır,
                    # tutma torku aşılırsa motor adım kaçırır (fizikte tork sınırı)
                    specs.append({"kind": "stepper", "limit": tau_joint if tau_joint else tau_ref[k], "copper_k": 0.0,
                                  "constant_w": ax["power"] or 0.0, "real_limit": tau_joint is not None, "linear": linear[k]})
                else:
                    power = ax["power"] if ax["power"] is not None else tau_joint * w_max[k]
                    tau_rated = tau_joint if tau_joint is not None else power / w_max[k]
                    specs.append(servo(k, power, tau_rated, True))
        else:
            specs = [auto(k) for k in range(n)]

        self.motor_specs = specs
        self.copper_k = [sp["copper_k"] for sp in specs]
        self.sim_engine.motor_forces = [sp["limit"] for sp in specs]
        self._stall_since = {}

    def electrical_power(self, velocities, torques):
        """Mekanik güç / verim + servo bakır kaybı (k x tork²) veya step motor sabit gücü + bekleme gücü."""
        eff = max(0.2, self.motor_efficiency)
        power = 0.0
        for k, (w, t) in enumerate(zip(velocities, torques)):
            spec = self.motor_specs[k] if k < len(self.motor_specs) else None
            power += abs(w * t) / eff
            if spec is None: continue
            power += spec["constant_w"] if spec["kind"] == "stepper" else spec["copper_k"] * t * t
        return power + self.motor_standby_w * len(self.sim_engine.revolute_joints)

    def detect_overload(self, targets):
        """Gerçek tork sınırı girilmiş eksenlerde (toplam güç / etiket) motor sınırda, eksen hareket etmiyor ve
        hedefinden uzaksa 0.5 sn sonra açıklama döner (step motor adım kaçırır, servo aşırı yük alarmı verir).
        Slider sıçramalarında motor kısa süre sınıra dayanır ama hareket ettiği için tetiklenmez."""
        specs = self.motor_specs
        if not any(sp["real_limit"] for sp in specs): return None
        now = time.perf_counter()
        with self.sim_engine.bullet_lock:
            states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
        for k, (sp, st) in enumerate(zip(specs, states)):
            if not sp["real_limit"] or k >= len(targets): continue
            far = abs(targets[k] - st[0]) > (0.005 if sp["linear"] else math.radians(2))
            stalled = abs(st[3]) >= 0.97 * sp["limit"] and abs(st[1]) < 0.05 and far
            if not stalled:
                self._stall_since.pop(k, None); continue
            since = self._stall_since.setdefault(k, now)
            if now - since >= 0.5:
                unit = "N" if sp["linear"] else "N·m"
                return t("estop.torque", j=k + 1, limit=f"{sp['limit']:.2f}", unit=unit)
        return None

    def set_playback_state(self, velocities, torques):
        # IK oynatıcısı eklemleri doğrudan konumlar; fizik hız/tork görmez. Ters dinamikten gelen değerler
        # kısa süre için fiziğin yerine kullanılır.
        self._playback = (time.perf_counter(), list(velocities), list(torques))

    def clear_playback_state(self):
        self._playback = None

    def apply_payload_settings(self, payload_val, tariff_val, efficiency_val, robot_mass_val=None):
        try:
            val_p = payload_val.strip().replace(',', '.')
            self.payload_kg = max(0.0, float(val_p)) if val_p else 0.0
        except Exception:
            self.payload_kg = 0.0

        try:
            val_t = tariff_val.strip().replace(',', '.')
            self.electricity_rate = max(0.01, float(val_t)) if val_t else 4.50
        except Exception:
            self.electricity_rate = 4.50

        try:
            val_e = efficiency_val.strip().replace(',', '.')
            eff_pct = max(10.0, min(100.0, float(val_e))) if val_e else 85.0
            self.motor_efficiency = eff_pct / 100.0
        except Exception:
            self.motor_efficiency = 0.85

        try:
            val_m = (robot_mass_val or "").strip().replace(',', '.')
            self.robot_mass_kg = float(val_m) if val_m and float(val_m) > 0 else self.robot_mass_default
        except Exception:
            self.robot_mass_kg = self.robot_mass_default

        self.apply_mass_model()

        return t("msg.energy_applied", payload=f"{self.payload_kg:.2f}", robot=f"{self.robot_mass_kg:.1f}",
                 eff=f"{self.motor_efficiency*100:.0f}", tariff=f"{self.electricity_rate:.2f}")

    def reset_energy_counter(self):
        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0
        return t("msg.counter_reset")

    def calculate_power_tick(self, dt):
        """Called periodically by the background engine thread to calculate physics energy consumption"""
        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is None: return

            playback = self._playback
            if self.sim_engine.e_stop_active:
                # Motorlar kilitli, fizik adımlanmıyor: sadece bekleme gücü (eskiden son değer donup kalıyordu)
                velocities = torques = [0.0] * len(self.sim_engine.revolute_joints)
            elif playback is not None and time.perf_counter() - playback[0] < 0.2:
                velocities, torques = playback[1], playback[2]
            else:
                joint_states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
                velocities = [js[1] for js in joint_states]
                torques = [js[3] for js in joint_states]   # uygulanan motor torku

            p_elec = self.electrical_power(velocities, torques)
            self.current_power = (self.current_power * 0.8) + (p_elec * 0.2)

            self.cumulative_energy_joules += p_elec * dt
            self.cumulative_energy_kwh = self.cumulative_energy_joules / 3.6e6
            self.cumulative_cost_tl = self.cumulative_energy_kwh * self.electricity_rate

            # Update history graph data
            self._history_elapsed += dt
            if self._history_elapsed >= self.history_interval:
                self._history_elapsed = 0.0
                self.power_history.append(self.current_power)
                if len(self.power_history) > 100:
                    self.power_history.pop(0)
