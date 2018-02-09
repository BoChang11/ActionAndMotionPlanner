(define (problem SOKO-QUERY1)
(:domain SOKO)
(:objects 
	  R1 - room 
	  R2 - room
	  R3 - room
	  R4 - room
	  d1 - door
	  d2 - door
	  d3 - door
	  d4 - door
	  o1 - movable
	  o2 - movable)
(:init
  (connects R1 R4 d4)
  (connects R4 R1 d4)
  (connects R1 R2 d1)
  (connects R2 R1 d1)
  (connects R2 R3 d2)
  (connects R3 R2 d2)
  (connects R3 R4 d3)
  (connects R4 R3 d3)
  (robotInRoom R1)
  (objInRoom o1 R2)
  (objInRoom o2 R4)
  (empty R1)
  (empty R3)
  (not (carry o1))
  (not (carry o2))
  (robotEmpty)
  (= (moveCost R1 R2) 1)
  (= (moveCost R2 R1) 1)
  (= (moveCost R2 R3) 1)
  (= (moveCost R3 R2) 1)
  (= (moveCost R3 R4) 1)
  (= (moveCost R4 R3) 1)
  (= (moveCost R4 R1) 3)
  (= (moveCost R1 R4) 3)

  (= (moveWithObjectCost R1 R2 d1 o1) 5)
  (= (moveWithObjectCost R1 R2 d1 o2) 10)
  (= (moveWithObjectCost R2 R1 d1 o1) 5)
  (= (moveWithObjectCost R2 R1 d1 o2) 10)

  (= (moveWithObjectCost R2 R3 d2 o1) 5)
  (= (moveWithObjectCost R2 R3 d2 o2) 10)
  (= (moveWithObjectCost R3 R2 d2 o1) 5)
  (= (moveWithObjectCost R3 R2 d2 o2) 10)

  (= (moveWithObjectCost R3 R4 d3 o1) 5)
  (= (moveWithObjectCost R3 R4 d3 o2) 10)
  (= (moveWithObjectCost R4 R3 d3 o1) 5)
  (= (moveWithObjectCost R4 R3 d3 o2) 10)

  (= (moveWithObjectCost R4 R1 d4 o1) 15)
  (= (moveWithObjectCost R4 R1 d4 o2) 100000)
  (= (moveWithObjectCost R1 R4 d4 o1) 15)
  (= (moveWithObjectCost R1 R4 d4 o2) 100000)

  (= (pickUpCost o1) 1)
  (= (pickUpCost o2) 2)

  (= (releaseCost o1) 1)
  (= (releaseCost o2) 2)

  (= (total-cost) 0)
 )
 (:goal 
       (and (objInRoom o2 R1)
       	    (objInRoom o1 R4)
            (not (carry o1))
            (not (carry o2))))
 (:metric minimize (total-cost))
)
