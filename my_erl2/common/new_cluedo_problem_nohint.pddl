(define (problem task)
(:domain cluedo_domain)
(:objects
    sherlock_robot - robot
    wp0 wp1 wp2 wp3 wp4 - location
    hy - hypotesis
)
(:init
    (not_visited wp4)

    (visited wp0)
    (visited wp3)
    (visited wp1)
    (visited wp2)

    (at wp2 sherlock_robot)





    (not_gripper_positioned)

    (not_initial_location wp1)
    (not_initial_location wp2)
    (not_initial_location wp3)
    (not_initial_location wp4)


)
(:goal (and
    (end_game)
))
)
