from collections import defaultdict


from routing.parameters import routing_interval_us


class BaseTable:
    """
    Base class of the table (regardless of the neighbor or routing table)

    This base class provides the following common functions:
    - is_empty(): to determine if the table is empty
    - is_item(): to determine if a recording is in the table
    - get_updated_time(): get the updated time for the specific item in this table
                          NOTE: updated time will be recorded in the last position
    - add_item(): this function should be overridden
    - remove_item(): delete specific item in the table
    - purge(): update the table, i.e., remove the expired item
    - print_item(): this function can be overridden to display the table
    - clear(): clear the whole table

    Author: Zihao Zhou & Kai Fang
    Created at: 2026/3/10
    Updated at: 2026/3/10
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.table = defaultdict(list)
        self.entry_life_time = routing_interval_us("table_entry_lifetime_s", 2.0)

    def is_empty(self):
        """Determine if the table is empty"""
        return not bool(self.table)

    def is_item(self, drone_id):
        """Determine if a specific item is in the table"""
        if drone_id in self.table.keys():
            if self.get_updated_time(drone_id) + self.entry_life_time > self.env.now:  # valid neighbor
                return True
        else:
            return False

    def get_updated_time(self, drone_id):
        if drone_id not in self.table.keys():
            raise RuntimeError('This item is not in this table!')
        else:
            return self.table[drone_id][-1]

    def add_item(self, hello_packet, cur_time):
        """It needs to be overridden"""
        pass

    def remove_item(self, drone_id):
        del self.table[drone_id]

    def purge(self):
        """Remove the expired item"""
        if not bool(self.table):
            # it means that the neighbor table is empty
            return

        for key in list(self.table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:
                self.remove_item(key)

    def print_item(self, my_drone):
        pass

    def clear(self):
        self.table.clear()
