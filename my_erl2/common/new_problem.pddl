(define (problem task)
(:domain cluedo_domain)
(:objects
    sherlock_robot - robot
    wp1 wp2 wp3 wp4 - location
    hy0 hy1 hy2 hy3 hy4 hy5 - hypotesis
)
(:init
    (visited wp1)

    (at wp1 sherlock_robot)





)
(:goal (and
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
))
)
