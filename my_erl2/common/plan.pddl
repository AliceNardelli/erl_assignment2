Number of literals: 19
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
27% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 5.000)b (5.000 | 5.000)b (3.000 | 10.001)b (2.000 | 15.002)b (1.000 | 20.003);;;; Solution Found
; States evaluated: 8
; Cost: 25.004
; Time 0.01
0.000: (move sherlock_robot wp2 wp4)  [5.000]
5.001: (move_gripper sherlock_robot wp4 hy)  [5.000]
10.002: (perceive_hint sherlock_robot wp4 hy)  [5.000]
15.003: (check_consistency hy)  [5.000]
20.004: (check_correct hy)  [5.000]
