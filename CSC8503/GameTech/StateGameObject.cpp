#include "StateGameObject.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/State.h"

 using namespace NCL;
 using namespace CSC8503;

#pragma region capsule
 StateGameObject::StateGameObject() {
	 counter = 0.0f;
	 stateMachine = new StateMachine();

	 State* stateA = new State([&](float dt)-> void
		 {
			 this->MoveLeft(dt);
		 }
	 );
	 State* stateB = new State([&](float dt)-> void
		 {
			 this->MoveRight(dt);
		 }
	 );
	 State* stateC = new State([&](float dt)-> void
		 {
			 this->Stop();
		 }
	 );

	 stateMachine->AddState(stateA);
	 stateMachine->AddState(stateB);
	 stateMachine->AddState(stateC);

	 stateMachine->AddTransition(new StateTransition(stateA, stateB,
		 [&]()-> bool
		 {
			 if (transform.GetPosition().z > 50.0f) {
				 GetPhysicsObject()->SetLinearVelocity(Vector3());
				 return true;
			 }
			 return false;
		 }
	 ));
	 stateMachine->AddTransition(new StateTransition(stateB, stateA,
		 [&]()-> bool
		 {
			 if (GetTransform().GetPosition().z < -50.0f) {
				 GetPhysicsObject()->SetLinearVelocity(Vector3());
				 return true;
			 }
			 return false;
		 }
	 ));

	 stateMachine->AddTransition(new StateTransition(stateA, stateC,
		 [&]()-> bool
		 {
			 if (IsCollision()) {
				 return true;
			 }
			 return false;
		 }
	 ));

	 stateMachine->AddTransition(new StateTransition(stateB, stateC,
		 [&]()-> bool
		 {
			 if (IsCollision()) {
				 return true;
			 }
			 return false;
		 }
	 ));
 }

StateGameObject ::~StateGameObject() {
	 delete stateMachine;
}
 
void StateGameObject::Update(float dt) {
	  stateMachine->Update(dt);
}

void StateGameObject::MoveLeft(float dt) {
	GetPhysicsObject()->AddForce({ 0 , 0, 20 });
}

void StateGameObject::MoveRight(float dt) {
	GetPhysicsObject()->AddForce({ 0, 0,-20 });
}
#pragma endregion