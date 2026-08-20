import math
import matplotlib.pyplot as plt
from utils import config


class EnergyModel:
    """
    Implementation of energy model (Y. Zeng2019)

    It should be noted that this class mainly calculates the power consumption required for UAV flight, while
    communication-related energy consumption does not require a special model.

    Attributes:
        delta: profile drag coefficient
        rho: air density
        s: rotor solidity, defined as the ratio of the total blade area to the disc area
        a: rotor disc area
        omega: blade angular velocity in radians/second
        r: rotor radius in meter
        k: incremental correction factor to induced power
        w: aircraft weight in Newton
        u_tip: tip speed of the rotor blade
        v0: mean rotor induced velocity in hover
        d0: fuselage drag ratio

    References:
        [1] Y. Zeng, J. Xu and R. Zhang, "Energy Minimization for Wireless Communication with Rotary-wing UAV," IEEE
            transactions on wireless communications, vol. 18, no. 4, pp. 2329-2345, 2019.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/21
    Updated at: 2025/4/23
    """

    def __init__(self, drone):
        self.my_drone = drone

        self.delta = config.PROFILE_DRAG_COEFFICIENT
        self.rho = config.AIR_DENSITY
        self.s = config.ROTOR_SOLIDITY
        self.a = config.ROTOR_DISC_AREA
        self.omega = config.BLADE_ANGULAR_VELOCITY
        self.r = config.ROTOR_RADIUS
        self.k = config.INCREMENTAL_CORRECTION_FACTOR
        self.w = config.AIRCRAFT_WEIGHT
        self.u_tip = config.ROTOR_BLADE_TIP_SPEED
        self.v0 = config.MEAN_ROTOR_VELOCITY
        self.d0 = config.FUSELAGE_DRAG_RATIO

        self.my_drone.simulator.env.process(self.energy_monitor())

    def power_consumption(self, speed):
        p0 = (self.delta / 8) * self.rho * self.s * self.a * (self.omega ** 3) * (self.r ** 3)
        pi = (1 + self.k) * (self.w ** 1.5) / (math.sqrt(2 * self.rho * self.a))
        blade_profile = p0 * (1 + (3 * speed ** 2) / (self.u_tip ** 2))
        induced = pi * (math.sqrt(1 + speed ** 4 / (4 * self.v0 ** 4)) - speed ** 2 / (2 * self.v0 ** 2)) ** 0.5
        parasite = 0.5 * self.d0 * self.rho * self.s * self.a * speed ** 3

        p = blade_profile + induced + parasite
        return p

    def energy_monitor(self):
        """Monitoring energy consumption of drone under a certain energy model"""

        while True:
            yield self.my_drone.simulator.env.timeout(1 * 1e5)  # report residual energy every 0.1s
            if self.my_drone.residual_energy <= config.ENERGY_THRESHOLD:
                self.my_drone.sleep = True
                self.my_drone.simulator.event_bus.publish(
                    "node_sleep",
                    self.my_drone.simulator.env.now,
                    node=self.my_drone.identifier,
                    energy_j=self.my_drone.residual_energy,
                )
                break

    def test(self):
        total_power = []

        test_speed = [i for i in range(0, 71, 2)]  # speed ranges from 0m/s to 70m/s
        for speed in test_speed:
            temp_p, temp_blade, temp_induced, temp_para = self.power_consumption(speed)
            total_power.append(temp_p)

        plt.figure()
        plt.plot(test_speed, total_power, color='black', linestyle='-', linewidth=2, label='total')
        plt.legend()
        plt.xlabel('UAV speed (m/s)')
        plt.ylabel('Required power (W)')
        plt.grid()
        plt.show()


# if __name__ == "__main__":
#     em = EnergyModel()
#     em.test()
