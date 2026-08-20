from utils import config
from sko.GA import GA


class CentralController:
    def __init__(self, simulator):
        self.simulator = simulator
        self.channel_assignment_dict = {i: None for i in range(config.NUMBER_OF_DRONES)}
        self.opt_interval = 1 * 1e6
        self.simulator.env.process(self.optimize_periodically())

    def optimize_periodically(self):
        """periodically run the dynamic channel assignment optimization"""
        while True:
            self._optimize()
            yield self.simulator.env.timeout(self.opt_interval)

    def _optimize(self):
        """Channel assignment using genetic algorithm"""
        best_x = self._dca_ga()

        self.channel_assignment_dict.update(zip(self.channel_assignment_dict.keys(), best_x))

    def _fitness_fun_ga(self, x):
        """Fitness function of the genetic algorithm, the goal is to minimize the total interference at all drones"""
        fitness = 0
        c = 0.5  # overlapping channel factor
        identifiers = [drone.identifier for drone in self.simulator.drones]
        gains = self.simulator.channel.channel_model.gains(
            self.simulator.env.now,
            self.simulator.drones,
            identifiers,
            identifiers,
        )

        for drone1 in self.simulator.drones:  # viewed as the potential receiver
            for drone2 in self.simulator.drones:  # viewed as the potential interference source
                if drone1.identifier != drone2.identifier:
                    w = max(1 - abs(x[drone1.identifier] - x[drone2.identifier]) * c, 0)

                    channel_gain = gains[(drone2.identifier, drone1.identifier)]
                    fitness += w * config.TRANSMITTING_POWER * channel_gain * 1e8

        return fitness

    def _dca_ga(self):
        ga = GA(
            func=self._fitness_fun_ga,
            n_dim=config.NUMBER_OF_DRONES,
            size_pop=50,
            max_iter=200,
            prob_mut=0.1,
            lb=[1] * config.NUMBER_OF_DRONES,
            ub=[14] * config.NUMBER_OF_DRONES,
            precision=1)

        best_x, best_y = ga.run()
        best_x = best_x.astype(int).tolist()

        return best_x
