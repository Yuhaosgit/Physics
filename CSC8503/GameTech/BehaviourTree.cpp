#include "BehaviourTree.h"

#include <iostream>

BehaviourTree::BehaviourTree(GameObject* obj): behaviourTimer(0.0){
	BehaviourSequence* rootSequence = new BehaviourSequence("Root Sequence");

	BehaviourSelector* Patrol = new BehaviourSelector("Patrol");

	BehaviourSelector* Escape = new BehaviourSelector("Escape");
	//action functions
	BehaviourAction* findKey = new BehaviourAction("Find Key",
		[&](float dt, BehaviourState state)->BehaviourState {
			if (state == Initialise) {
				std::cout << "Looking for a key!\n";
				behaviourTimer = rand() % 100;
				state = Ongoing;
			}

			else if (state == Ongoing) {
				behaviourTimer -= dt;
				if (behaviourTimer <= 0.0f) {
					std::cout << "Found a key!\n";
					return Success;
				}
			}
			return state; //will be ¡¯ongoing ¡¯ until success
		}
	);



	//link nodes
	Patrol->AddChild(RobPower);
	Patrol->AddChild(Attack);
	Patrol->AddChild(HangOut);

	Escape->AddChild(Run);

	rootSequence->AddChild(Patrol);
	rootSequence->AddChild(Escape);
}
void BehaviourTree::Implement(const float &dt) {
	BehaviourState state = Ongoing;
	std::cout << "We¡¯re going on an adventure !\n";
	while (state == Ongoing) {
		state = rootSequence->Execute(1.0f); //fake dt
	}
	if (state == Success) {
		std::cout << "What a successful adventure !\n";
	}
	else if (state == Failure) {
		std::cout << "What a waste of time!\n";
	}
}
