(define (domain cluedo_domain)

    (:requirements
        :strips
        :typing
        :durative-actions 
        :fluents 
        :duration-inequalities
      
    )

    (:types
       robot
       location
       hypotesis
    )

    (:predicates
       (visited ?l - location)
       (at ?l - location ?r - robot)
       (perceived ?hy - hypotesis)
       (consistent ?hy -hypotesis)
       (end_game)
       (gripper_positioned)
       (not_gripper_positioned)
       (not_initial_location ?l -location)
    )

    (:durative-action move
       
        :parameters (?r - robot ?l1 ?l2 - location)

        :duration
            (= ?duration 5)

        :condition
	        (and
	            (at start (visited ?l1))
	            (at start  (at ?l1 ?r ))
	            (at start (not_gripper_positioned))
	           
	            )

        :effect
	        (and
	            (at end (not (at ?l1 ?r )))
	            (at end (at ?l2 ?r))
                    (at end (visited ?l2))
                )
	)

   
    (:action new_turn
        :parameters (?l1 ?l2 ?l3 ?l4 ?l0 - location)
        :precondition (and (visited ?l1)(visited ?l2)(visited ?l3)(visited ?l4)(visited ?l0)(not_initial_location ?l1)(not_initial_location ?l2)(not_initial_location ?l3)(not_initial_location ?l4))
        :effect (and
         (not(visited ?l1))(not(visited ?l2))(not(visited ?l3))(not(visited ?l4))
    ))
        (:durative-action move_gripper
       
        :parameters ( ?r - robot  ?l  - location ?hy - hypotesis)

        :duration
            (= ?duration 5)

        :condition
	        (and
	            (at start (not_gripper_positioned))
	            (at start  (at ?l ?r ))
	            (at start (visited ?l))
	            (at start (not_initial_location ?l))
	           
	            )

        :effect
	        (and
	            (at end (gripper_positioned))
	            (at end (not(not_gripper_positioned)))
                 
                )
	) 
	 (:durative-action perceive_hint
       
        :parameters ( ?r - robot  ?l  - location ?hy - hypotesis)

        :duration
            (= ?duration 5)

        :condition
	        (and
	            (at start (gripper_positioned))
	            (at start  (at ?l ?r ))
	            (at start (visited ?l))
	            (at start (not_initial_location ?l))
	           
	            )

        :effect
	        (and
	            (at end (not_gripper_positioned))
	            (at end (not(gripper_positioned)))
                    (at end (perceived ?hy))
                )
	)  
	(:durative-action check_consistency
       
        :parameters (?hy - hypotesis)

        :duration
            (= ?duration 5)

        :condition
	        (and
	            (at start (perceived ?hy))
	   	           
	            )

        :effect
	        (and
	            (at end (consistent ?hy))
                    (at end (not (perceived ?hy)))
                )
	)  
	
    	(:durative-action check_correct
       
        :parameters (?hy - hypotesis)

        :duration
            (= ?duration 5)

        :condition
	        (and
	            (at start (consistent ?hy))
	   	           
	            )

        :effect
	        (and
	            (at end (end_game))
                    (at end (not (consistent ?hy)))
                )
	)     

    
 

    
)
