#include "TutorialGame.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/State.h"


SphereState::SphereState(TutorialGame* game) :g(game) {
	counter = 0.0f;
	stateMachine = new StateMachine();

	State* stateA = new State([&](float dt)-> void
		{
			this->FollowWay(dt);
		}
	);

	State* stateB = new State([&](float dt)-> void
		{
			this->RobMoney(dt);
		}
	);

	stateMachine->AddState(stateA);
	stateMachine->AddState(stateB);

	stateMachine->AddTransition(new StateTransition(stateA, stateB,
		[&]()-> bool
		{
			for (int i = 0; i < g->awards.size(); ++i) {
				Vector3 direction = g->enemy->GetTransform().GetPosition() -
					g->awards[i]->GetTransform().GetPosition();
				Debug::DrawLine(g->enemy->GetTransform().GetPosition(), g->awards[i]->GetTransform().GetPosition()
					, Vector4(0, 0, 1, 1));

				float dis = direction.LengthSquared();
			
				Ray view(g->enemy->GetTransform().GetPosition() + Vector3(1, 1, 1),
					(g->awards[i]->GetTransform().GetPosition() -
						g->enemy->GetTransform().GetPosition()).Normalised());
				RayCollision closestCollision;

				if (g->world->Raycast(view, closestCollision, true)) {
					if (((GameObject*)closestCollision.node)->IsBonus()) {
						Dir_to_Target = (g->awards[i]->GetTransform().GetPosition() -
							g->enemy->GetTransform().GetPosition()).Normalised();
						return true;
					}
				}
			}
			return false;
		}
	));

	stateMachine->AddTransition(new StateTransition(stateB, stateA,
		[&]()-> bool
		{
			for (int i = 0; i < g->awards.size(); ++i) {
				if (CollisionDetection::ObjectIntersection(g->enemy, g->awards[i], CollisionDetection::CollisionInfo())) {
					g->awards.erase(g->awards.begin() + i);
					return true;
				}
			}
			return false;
		}
	));
}

SphereState::~SphereState() {
	delete stateMachine;
}

void SphereState::Update(float dt) {
	stateMachine->Update(dt);
}

void SphereState::FollowWay(float dt) {
	if (g->good_path.size() > 0) {
		Vector3 position = g->enemy->GetTransform().GetPosition();
		float distance = (g->good_path[1] - position).Length();
		arrived = distance < 0.2;
	}
	if (arrived) {
		g->good_path.clear();
		g->FindPath();
	}

	if (g->good_path.size() > 1) {
		g->navigate_DEBUG();
		g->navigate_obj();
	}
}

void SphereState::RobMoney(float dt) {
	g->enemy->GetPhysicsObject()->SetLinearVelocity(Dir_to_Target * 15);
}
