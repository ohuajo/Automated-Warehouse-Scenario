
%%%%%%%%%%%%%%%%% Generating search space %%%%%%%%%%%%%%%%%%%%%%
% Row size
rowSize(RS):- RS=#count{X:init(object(node,N),value(at,pair(X,Y)))}.
% Column size
colSize(CS):- CS=#count{Y:init(object(node,N),value(at,pair(X,Y)))}.
% Total robots
tRobots(TR):- TR=#count{R:init(object(robot,R),value(at,pair(X,Y)))}.
% Total nodes
tNodes(TN):- TN=#count{N:init(object(node,N),value(at,pair(X,Y)))}.
% Total shelves
tShelves(TS):- TS=#count{S:init(object(shelf,S),value(at,pair(X,Y)))}.
% Total products
tProducts(TI):- TI=#count{I:init(object(product,I),value(on,pair(X,Y)))}.
% Total picking stations
tPStation(TP):- TP=#count{P:init(object(pickingStation,P),value(at,pair(X,Y)))}.
% Total orders
tOrders(TO):- TO=#count{O:init(object(order,O),value(pickingStation,J))}.
% Nodes and pairs
node(N):- init(object(node,N),value(at,pair(X,Y))).
nodeAt(N,pair(X,Y)):- init(object(node,N),value(at,pair(X,Y))).
pair(X,Y):- init(object(node,N),value(at,pair(X,Y))).
% Highways
highway(H):- init(object(highway,H),value(at,pair(X,Y))).
% Pickingstaions
pickingStationAt(P,N):- init(object(pickingStation,P),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
pickingStation(P):- init(object(pickingStation,P),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
% Robots
robotAt(R,object(node,N),0):- init(object(robot,R),value(at,pair(X,Y))), nodeAt(N,pair(X,Y)).
robot(R):- init(object(robot,R),value(at,pair(X,Y))).
% Shelves
shelfOn(S,object(node,N),0):- init(object(shelf,S),value(at,pair(X,Y))), nodeAt(N,pair(X,Y)).
shelf(S):- init(object(shelf,S),value(at,pair(X,Y))).
% Products
productOn(I,object(shelf,S),with(quantity,U),0):- init(object(product,I),value(on,pair(S,U))).
product(I):- init(object(product,I),value(on,pair(S,U))).
%Orders
orderAt(O,object(node,N),contains(I,U),0):- init(object(order,O),value(pickingStation,P)), pickingStationAt(P,N), init(object(order,O),value(line,pair(I,U))).
order(O):- init(object(order,O),value(pickingStation,P)).
% Robot movement
move(0,1;0,-1;-1,0;1,0).
{robotMove(R,move(DX,DY),T):move(DX,DY)}1 :- R=1..TR, tRobots(TR), T=1..t-1.
occurs(object(robot,R),move(DX,DY),T):-robotMove(R,move(DX,DY),T).
% Robot pick up shelves
{pickUpShelf(R,S,T):shelf(S)}1:- R=1..TR, tRobots(TR), T=1..t-1.
occurs(object(robot,R),pickup,T):-pickUpShelf(R,_,T).
% Robot put down shelves
{putDownShelf(R,S,T):shelf(S)}1:- R=1..TR, tRobots(TR), T=1..t-1.
occurs(object(robot,R),putdown,T):-putDownShelf(R,_,T).
% Robot delivers items
{deliver(R,O,with(S,I,U1),T):orderAt(O,object(node,N),contains(I,U),T), productOn(I,object(shelf,S),with(quantity,U2),T), U1=1..U2}1 :- R=1..TR, tRobots(TR), T=1..t-1.
occurs(object(robot,R),deliver(O,I,U),T):- deliver(R,O,with(S,I,U),T).


%%%%%%%%%%%%%%%%%%%%%%%%%%% Defining and testing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Robots
% A robot can't execute 2 actions at same time
:- occurs(object(robot,R),A1,T), occurs(object(robot,R),A2,T), A1!=A2.
% No robot collission allowed 
:- 2{robotAt(R,object(node,N),T):robot(R)}, node(N), T=1..t.
:- robotAt(R1,object(node,N1),T), robotAt(R1,object(node,N2),T+1), robotAt(R2,object(node,N2),T), robotAt(R2,object(node,N1),T+1), R1!=R2.
% A robot can only be on one node at a time
:- not {robotAt(R,object(node,N),T):node(N)} = 1, robot(R), T=1..t.
% Robot cannot move outside of the grid
:- robotAt(RI,object(node,N),T), robotMove(R,move(DX,DY),T), nodeAt(N,pair(X,Y)), X+DX<1.
:- robotAt(RI,object(node,N),T), robotMove(R,move(DX,DY),T), nodeAt(N,pair(X,Y)), Y+DY<1.
:- robotAt(RI,object(node,N),T), robotMove(R,move(DX,DY),T), nodeAt(N,pair(X,Y)), X+DX>RS, rowSize(RS).
:- robotAt(RI,object(node,N),T), robotMove(R,move(DX,DY),T), nodeAt(N,pair(X,Y)), Y+DY>CS, colSize(CS).
% Effectand law of inertia of moving a robot
robotAt(R,object(node,N2),T+1):- robotAt(R,object(node,N),T), nodeAt(N,pair(X,Y)), nodeAt(N2, pair(X+DX,Y+DY)), robotMove(R,move(DX,DY),T).
robotAt(R,object(node,N),T+1):- robotAt(R,object(node,N),T), not robotMove(R,move(_,_),T), T<t.

%% Shelves
% No shelf on 2 nodes
:- 2{shelfOn(S,object(node,N),T): node(N)}, shelf(S), T=1..t.
% No 2 shelves on the same node
:- 2{shelfOn(S,object(node,N),T): shelf(S)}, node(N), T=1..t.
% No shelf on 2 locations (robot, node)
:- shelfOn(S,object(node,_),T), shelfOn(S,object(robot,_),T).
% Shelf can not be put down on highway
:- putDownShelf(R,S,T), robotAt(R,object(node,N),T), highway(N). 
% Dissallow shelves on the highway
:- shelfOn(S,object(node,N),_), highway(N).

%%% Robot interaction with shelves and others
% Only one robot can pick a shelve
:- 2{pickUpShelf(R,S,T): robot(R)} , shelf(S).
% Robots don't move through a node with shelf
:- shelfOn(S1,object(node,N),T), shelfOn(S2,object(robot,R),T), robotAt(R,object(node,N),T), S1!=S2.
% A robot handles one shelf at atime.
:- pickUpShelf(R,S1,T), shelfOn(S2,object(robot,R),T).
:- pickUpShelf(R1,S,T), shelfOn(S,object(robot,R2),T).
% A robot only pick shelves in the same node as the robot
:- pickUpShelf(R,S,T), shelfOn(S,object(node,N),T), not robotAt(R,object(node,N),T). 
% Only one robot can putdown shelve
:-  2 {putDownShelf(R,S,T): robot(R)}, shelf(S).
% Robot has to have shelf to putdown
:- putDownShelf(R,S,T), not shelfOn(S,object(robot,R),T).
% No shelf on 2 robots
:- 2{shelfOn(S,object(robot,R),T): robot(R)}, shelf(S), T=1..t.
% No 2 shelves on the same robot
:- 2{shelfOn(S,object(robot,R),T): shelf(S)}, robot(R), T=1..t.
% Effect and law of inertia of picking up a shelf
shelfOn(S,object(robot,R),T+1):- pickUpShelf(R,S,T), shelfOn(S,object(node,N),T), robotAt(R,object(node,N),T).
shelfOn(S,object(node,N),T+1):-shelfOn(S,object(node,N),T), not pickUpShelf(_,S,T), T<t.
%Effect  and law of inertia of putting down a shelf
shelfOn(S,object(node,N),T+1):- putDownShelf(R,S,T), shelfOn(S,object(robot,R),T), robotAt(R,object(node,N),T).
shelfOn(S,object(robot,R),T+1):-shelfOn(S,object(robot,R),T), not putDownShelf(R,S,T), T<t.

%%% Orders
% Order must be fulfilled
:- not orderAt(O,object(node,_),contains(I,0),t), orderAt(O,object(node,_),contains(I,_),0).
% Delivery of the right item quantity and shelve at picking station
:- deliver(R,O,with(_,I,_),T), orderAt(O,object(node,N),contains(I,_),T), not robotAt(R,object(node, N),T).
:- deliver(R,O,with(S,I,_),T), productOn(I,object(shelf,S),with(quantity,_),T), not shelfOn(S,object(robot,R),T).
:- deliver(R,O,with(S,I,U1),T), orderAt(O,object(node,N),contains(I,U2),T), U1>U2.
:- deliver(R,O,with(S,I,U1),T), productOn(I,object(shelf,S),with(quantity,U2),T), U1>U2.
% Effect  and law of inertia of delivering a product
orderAt(O,object(node,N),contains(I,U1-U2),T+1):- deliver(R,O,with(S,I,U2),T), orderAt(O,object(node,N),contains(I,U1),T).
productOn(I,object(shelf,S),with(quantity,U1-U2),T+1):- deliver(R,O,with(S,I,U2),T), productOn(I,object(shelf,S),with(quantity,U1),T).
orderAt(O,object(node,N),contains(I,U),T+1):- orderAt(O,object(node,N),contains(I,U),T), productOn(I,object(shelf,S),with(quantity,U1),T), not deliver(_,O,with(S,I,_),T), T<t.
productOn(I,object(shelf,S),with(quantity,U),T+1):- productOn(I,object(shelf,S),with(quantity,U),T), not deliver(_,_,with(S,I,_),T), T<t.



%%%%%%%% Minimize time, show and constant %%%%%%%%
#minimize{T:occurs(O,A,T)}.
#show occurs/3.
#const t=20.

