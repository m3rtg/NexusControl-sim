import pybullet as p
from i18n import t
import pybullet_data
import threading
import random
import math
import os

class SimulationEngine:
    def __init__(self):
        self.bullet_lock = threading.RLock()
        self.physicsClient = None
        
        # State variables
        self.robotId = None
        self.revolute_joints = []
        self.obstacle_id = None
        self.obstacle_pos = [0.0, 0.0, 0.0]
        self.obstacle_half = [0.0, 0.0, 0.0]
        # Engel boyutu ve mesafesi yüklenen modele göre ölçeklenir (RobotLoader ayarlar).
        # 1.0 = Fanuc LR Mate (80 cm erişim); keep_out = kaidenin yarı genişliği (m)
        self.obstacle_scale = 1.0
        self.obstacle_keep_out = 0.0
        self.e_stop_active = False
        # Eklem başına motor tork sınırı; TelemetryEngine.calibrate_motors robot ağırlığına göre ayarlar
        self.motor_forces = []
        
        # For GPU renderer sharing
        self.gpu_state_lock = None
        self.gpu_shared_state = None
        
    def set_gpu_shared_state(self, gpu_state, gpu_lock):
        self.gpu_shared_state = gpu_state
        self.gpu_state_lock = gpu_lock

    def init_pybullet(self):
        with self.bullet_lock:
            self.physicsClient = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            # Plane is usually added if needed, but original code relies on URDF for stand/floor.

    def toggle_obstacle(self):
        with self.bullet_lock:
            if self.obstacle_id is None:
                s = self.obstacle_scale
                hx, hy, hz = 0.08 * s, 0.08 * s, 0.2 * s
                # Kaidenin (varsa) dışında kalsın
                x = max(0.4 * s, self.obstacle_keep_out + hx + 0.05 * s)
                self._place_obstacle([x, 0.0, hz], [hx, hy, hz])
                return True, t("msg.obstacle_on")
            else:
                self.remove_obstacle()
                return False, t("msg.obstacle_off")

    def randomize_obstacle(self):
        with self.bullet_lock:
            s = self.obstacle_scale
            # Kolun o anki duruşuyla çakışan konum çıkarsa yeniden dene; yoksa engel anında E-stop tetikler
            for _ in range(30):
                hx = random.uniform(0.04, 0.08) * s
                hy = random.uniform(0.04, 0.08) * s
                hz = random.uniform(0.15, 0.40) * s

                # Kare kaidenin köşesine de değmemesi için merkezden en az bu kadar uzakta olmalı
                r_min = (self.obstacle_keep_out + max(hx, hy)) * math.sqrt(2) + 0.05 * s
                angle = random.uniform(-math.pi / 1.5, math.pi / 1.5)
                radius = random.uniform(max(0.35 * s, r_min), max(0.60 * s, r_min + 0.25 * s))
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)

                self._place_obstacle([x, y, hz], [hx, hy, hz])
                if self.robotId is None or not p.getClosestPoints(self.robotId, self.obstacle_id, 0.02 * s):
                    break
            return t("msg.obstacle_random", h=f"{hz * 200:.0f}", x=f"{x * 100:.0f}", y=f"{y * 100:.0f}")

    def _place_obstacle(self, pos, half):
        if self.obstacle_id is not None:
            p.removeBody(self.obstacle_id)
        self.obstacle_pos = list(pos)
        self.obstacle_half = list(half)
        obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        self.obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obs_col, basePosition=pos)
        if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
            with self.gpu_state_lock:
                self.gpu_shared_state['obstacle'] = (list(pos), list(half))

    def remove_obstacle(self):
        with self.bullet_lock:
            if self.obstacle_id is not None:
                p.removeBody(self.obstacle_id)
                self.obstacle_id = None
        if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
            with self.gpu_state_lock:
                self.gpu_shared_state['obstacle'] = None

    def reset_estop(self, shared_targets):
        self.e_stop_active = False
        with self.bullet_lock:
            self.remove_obstacle()
            if self.robotId is not None:
                for i, joint_idx in enumerate(self.revolute_joints):
                    p.resetJointState(self.robotId, joint_idx, 0.0)
                    if i < len(shared_targets):
                        shared_targets[i] = 0.0
                        
        return t("msg.estop_reset")

    def step_simulation(self, targets):
        """Called periodically by the background engine thread."""
        with self.bullet_lock:
            if self.robotId is not None and not self.e_stop_active:
                for i, joint_idx in enumerate(self.revolute_joints):
                    if i < len(targets):
                        p.setJointMotorControl2(
                            self.robotId, joint_idx, p.POSITION_CONTROL,
                            targetPosition=targets[i], force=self.motor_forces[i] if i < len(self.motor_forces) else 800,
                            maxVelocity=4.0,
                            positionGain=0.15, velocityGain=1.0
                        )
                p.stepSimulation()
