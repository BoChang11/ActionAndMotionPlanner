;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Robot transfers objects
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain SOKO)
  (:requirements :typing)
  (:types room door movable)
  (:predicates (robotInRoom ?Ri)
	       (objInRoom ?o ?Ri)
	       (connects ?Ri ?Rj ?d)
	       (empty ?Ri)
               (carry ?o)
	       (robotEmpty)	
               )

  (:functions (total-cost) - number
  	      (moveCost ?Ri ?Rj - room ?d - door)
	      (pickupObjectCost ?Ri - room ?o - movable)
	      (releaseObjectCost ?Ri - room ?o - movable)
	      (moveWithObjectCost ?Ri ?Rj - room ?d - door ?o - movable)
  )	

  (:action move
  	   :parameters (?Ri ?Rj - room ?d - door)
 	   :precondition
	    (and (robotInRoom ?Ri)
	    	 (connects ?Ri ?Rj ?d)
		 (robotEmpty)
            )
	   :effect
	   (and (not (robotInRoom ?Ri))
	   	(robotInRoom ?Rj)
		(increase (total-cost) (moveCost ?Ri ?Rj ?d))
           )
  )

  (:action pickupObject
  	   :parameters (?Ri - room ?o - movable)
 	   :precondition
	    (and (robotInRoom ?Ri)
	    	 (objInRoom ?o ?Ri)
		 (robotEmpty)
            )     
	   :effect
	   (and (carry ?o)
                (not (robotEmpty))
                (increase (total-cost) (pickupObjectCost ?Ri ?o))
           )
  )

  (:action moveWithObject
  	   :parameters (?Ri ?Rj - room ?d - door ?o - movable)
 	   :precondition
	    (and (robotInRoom ?Ri)
	    	 (objInRoom ?o ?Ri)
                 (carry ?o)
		 (empty ?Rj)	
                 (connects ?Ri ?Rj ?d)
            )     
	   :effect
	   (and (not (objInRoom ?o ?Ri))
	   	(objInRoom ?o ?Rj)
		(not (robotInRoom ?Ri))
		(robotInRoom ?Rj)
		(empty ?Ri)
		(not (empty ?Rj))
		(carry ?o)
                (increase (total-cost) (moveWithObjectCost ?Ri ?Rj ?d ?o))
		        )
  )

  (:action releaseObject
  	   :parameters (?Ri - room ?o - movable)
 	   :precondition
	    (and (robotInRoom ?Ri)
	    	 (objInRoom ?o ?Ri)
                 (carry ?o)
	         (not (robotEmpty))
            )     
	   :effect
	   (and (not (carry ?o))
                (robotEmpty)
                (increase (total-cost) (releaseObjectCost ?Ri ?o))
           )
  )

)
