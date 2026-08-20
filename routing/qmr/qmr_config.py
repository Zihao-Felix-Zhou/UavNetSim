from utils.radio import routing_neighbor_distance

hello_interval = 0.2 * 1e6
hello_interval_count = 10
discount_factor_update_interval = 10 * hello_interval
history_packet_life_time = 10 * hello_interval

window_mean_beta = 0.5
max_mac_delay_recorder_len = 100
max_queue_delay_recorder_len = 100
max_delay_list_len = 100

communication_range = routing_neighbor_distance()

max_reward = 100
min_reward = -5
omega = 0.8

eps_start = 0.8
eps_decay = 0.99
eps_update_interval = 0.1 * 1e6
eps_on = 0

use_fix_learning_rate = 0
fix_learning_rate = 0.3

self_information_update_interval = 0.01 * 1e6
self_information_storage_len = 10 * 1e6 / self_information_update_interval



fixed_hello_interval = 0.3 * 1e6

# 'greedy' -> e-greedy
# 'qmr' -> qmr traditional
which_exploration_mechanism = 'qmr'
