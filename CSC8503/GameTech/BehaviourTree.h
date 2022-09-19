#pragma once
#include "BehaviourAction.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "..//CSC8503Common/GameObject.h"

using namespace NCL;
using namespace CSC8503;

class BehaviourTree {
protected:
	float behaviourTimer;
	float distanceToTarget;

	BehaviourSequence* rootSequence;

	BehaviourSelector* Patrol;
	BehaviourAction* RobPower;
	BehaviourAction* Attack;
	BehaviourAction* HangOut;

	BehaviourSelector* Escape;
	BehaviourAction* Run;


public:
	BehaviourTree(GameObject* obj);
	void Implement(const float &dt);
};