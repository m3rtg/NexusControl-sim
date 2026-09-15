import pybullet as p

class TelemetryEngine:
    def __init__(self, sim_engine):
        self.sim_engine = sim_engine
        
        self.payload_kg = 0.0
        self.base_ee_mass = 1.0
        self.motor_efficiency = 0.85
        self.motor_standby_w = 15.0
        self.electricity_rate = 4.50
        
        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0
        self.current_power = 0.0
        self.power_history = [0.0] * 100

    def apply_payload_settings(self, payload_val, tariff_val, efficiency_val):
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

        with self.sim_engine.bullet_lock:
            if getattr(self.sim_engine, 'robotId', None) is not None and getattr(self.sim_engine, 'end_effector_index', None) is not None:
                p.changeDynamics(
                    self.sim_engine.robotId, 
                    self.sim_engine.end_effector_index, 
                    mass=self.base_ee_mass + self.payload_kg
                )

        return f"Yük Güncellendi: {self.payload_kg:.2f} kg | Verim: %{self.motor_efficiency*100:.0f} | Tarife: {self.electricity_rate:.2f} TL/kWh"

    def reset_energy_counter(self):
        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0
        return "Enerji ve maliyet sayacı sıfırlandı."

    def calculate_power_tick(self, dt):
        """Called periodically by the background engine thread to calculate physics energy consumption"""
        with self.sim_engine.bullet_lock:
            if self.sim_engine.robotId is None: return

            total_mech = 0.0
            joint_states = p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)
            for js in joint_states:
                total_mech += abs(js[1] * js[3])   # velocity × torque

            n_axes = len(self.sim_engine.revolute_joints)
            eff = max(0.2, self.motor_efficiency)
            standby_p = self.motor_standby_w * n_axes

            p_elec = (total_mech / eff) + standby_p
            self.current_power = (self.current_power * 0.8) + (p_elec * 0.2)

            self.cumulative_energy_joules += p_elec * dt
            self.cumulative_energy_kwh = self.cumulative_energy_joules / 3.6e6
            self.cumulative_cost_tl = self.cumulative_energy_kwh * self.electricity_rate
            
            # Update history graph data
            self.power_history.append(self.current_power)
            if len(self.power_history) > 100:
                self.power_history.pop(0)
