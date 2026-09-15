import pybullet as p
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
        self.e_stop_active = False
        
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
                hx, hy, hz = 0.08, 0.08, 0.2
                self.obstacle_pos = [0.4, 0.0, hz]
                obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
                self.obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obs_col,
                                                     basePosition=self.obstacle_pos)
                
                if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
                    with self.gpu_state_lock:
                        self.gpu_shared_state['obstacle'] = (list(self.obstacle_pos), [hx, hy, hz])
                return True, "Engel aktif edildi."
            else:
                p.removeBody(self.obstacle_id)
                self.obstacle_id = None
                
                if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
                    with self.gpu_state_lock:
                        self.gpu_shared_state['obstacle'] = None
                return False, "Engel kaldirildi."

    def randomize_obstacle(self):
        with self.bullet_lock:
            if self.obstacle_id is not None:
                p.removeBody(self.obstacle_id)

            hx = random.uniform(0.04, 0.08)
            hy = random.uniform(0.04, 0.08)
            hz = random.uniform(0.15, 0.40)

            angle = random.uniform(-math.pi / 1.5, math.pi / 1.5)
            radius = random.uniform(0.35, 0.60)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = hz

            self.obstacle_pos = [x, y, z]
            obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
            self.obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obs_col,
                                                 basePosition=self.obstacle_pos)
                                                 
            if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
                with self.gpu_state_lock:
                    self.gpu_shared_state['obstacle'] = ([x, y, z], [hx, hy, hz])

            return f"Rastgele Engel: Uzunluk {hz * 200:.0f}cm, X:{x * 100:.0f} Y:{y * 100:.0f}"

    def reset_estop(self, shared_targets):
        self.e_stop_active = False
        with self.bullet_lock:
            if self.obstacle_id is not None:
                p.removeBody(self.obstacle_id)
                self.obstacle_id = None
                
            if self.gpu_shared_state is not None and self.gpu_state_lock is not None:
                with self.gpu_state_lock:
                    self.gpu_shared_state['obstacle'] = None
                    
            if self.robotId is not None:
                for i, joint_idx in enumerate(self.revolute_joints):
                    p.resetJointState(self.robotId, joint_idx, 0.0)
                    if i < len(shared_targets):
                        shared_targets[i] = 0.0
                        
        return "SİSTEM SIFIRLANDI. Güvenlik için engel silindi, Base'e dönüldü."

    def step_simulation(self, targets):
        """Called periodically by the background engine thread."""
        with self.bullet_lock:
            if self.robotId is not None and not self.e_stop_active:
                for i, joint_idx in enumerate(self.revolute_joints):
                    if i < len(targets):
                        p.setJointMotorControl2(
                            self.robotId, joint_idx, p.POSITION_CONTROL,
                            targetPosition=targets[i], force=800, maxVelocity=4.0,
                            positionGain=0.15, velocityGain=1.0
                        )
                p.stepSimulation()
