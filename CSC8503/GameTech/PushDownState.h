#pragma once
#include "../../Common/Window.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Common/ShareVariable.h"
#include "TutorialGame.h"

#include <iostream>
#include <string>


namespace NCL {
	namespace CSC8503 {
		class PushdownState {
		public:
			enum PushdownResult { Push, Pop, NoChange };
			PushdownState() {}
			virtual ~PushdownState() {}

			virtual PushdownResult OnUpdate(float dt, PushdownState** pushFunc) = 0;

			virtual void OnAwake() {}
			virtual void OnSleep() {}
			virtual void PrintMessage() {}
			virtual void Update(){}
		};
	}
}

#pragma region PushDownMenu
using namespace NCL;
using namespace CSC8503;

#pragma region End
//PushDown menu
class EndScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::C)) {
			g->RESTART = true;
			return PushdownResult::Pop;
		}
		else {
			g->UpdateGame_main_menu(dt);
			return PushdownResult::NoChange;
		}
	}

	void OnAwake() override {
	}
	void PrintMessage() override {
		Debug::Print("Time:"+ std::to_string((int)time_count) + "s", Vector2(5, 5));
		Debug::Print("Score:" + std::to_string(score), Vector2(5, 15));

		if (g->LoseSign())
			Debug::Print("You Lose :(", Vector2(30, 50));
		else
			Debug::Print("You Get Win :)", Vector2(30, 50));

		Debug::Print("Press C to another round.", Vector2(5, 20));
		Debug::Print("Press ESC to quit.", Vector2(5, 30));
	}

	float time_count;
	TutorialGame* g;
public:
	EndScreen(TutorialGame* obj,float time) :g(obj),time_count(time) {  }
};
#pragma endregion

#pragma region Pause
//PushDown menu
class PauseScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::O)) {
			return PushdownResult::Pop;
		}
		else {
			g->UpdateGame_main_menu(dt);
			return PushdownResult::NoChange;
		}
	}

	void OnAwake() override {
	}
	void PrintMessage() override {
		Debug::Print("Pause Now", Vector2(30, 50));
		Debug::Print("GO ON! GO ON!", Vector2(30, 45));

		Debug::Print("Press O back to game.", Vector2(5, 20));
	}
	TutorialGame* g;
public:
	PauseScreen(TutorialGame* obj) :g(obj) {  }
};
#pragma endregion

#pragma region Game

class GameScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (g->END) {
			*newState = new EndScreen(g, time_count);
			return PushdownResult::Push;
		}

		if (g->RESTART) {
			g->RESTART = false;
			g->SetLoseSign(false);
			return PushdownResult::Pop;
		}

		if (g->LoseSign()) {
			*newState = new EndScreen(g, time_count);
			return PushdownResult::Push;
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::P)) {
			*newState = new PauseScreen(g);
			return PushdownResult::Push;
		}
		else {
			g->UpdateGame(dt);
			time_count += dt;
			return PushdownResult::NoChange;
		}
	}

	void OnAwake() override {
		g->InitWorld();
		g->selectionObject = nullptr;
		g->lockedObject = nullptr;
	}
	void PrintMessage() override {
		Debug::Print("Score:" + std::to_string(score), Vector2(5, 15));
		Debug::Print("Time:" + std::to_string((int)time_count) + "s", Vector2(5, 5));
		Debug::Print("Press P to pause", Vector2(5, 10));
	}

	TutorialGame* g;
	float time_count = 0;
public:
	GameScreen(TutorialGame* obj) :g(obj) {  }
};
#pragma endregion

#pragma region Maze
class GameMaze : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (g->END) {
			*newState = new EndScreen(g, time_count);
			return PushdownResult::Push;
		}
		if (g->RESTART) {
			g->RESTART = false;
			return PushdownResult::Pop;
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::P)) {
			*newState = new PauseScreen(g);
			return PushdownResult::Push;
		}

		else {
			g->UpdateGame_maze(dt);
			time_count += dt;
			return PushdownResult::NoChange;
		}
	}

	void OnAwake() override {
		g->InitMaze();
	}
	void PrintMessage() override {
		Debug::Print("Time:" + std::to_string((int)time_count) + "s", Vector2(5, 5));

		Debug::Print("Press P to pause", Vector2(5, 10));
	}

	TutorialGame* g;
	float time_count = 0;
public:
	GameMaze(TutorialGame* obj) :g(obj) {  }
};
#pragma endregion

#pragma region introduction

class IntroScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Z)) {
			*newState = new GameScreen(g);
			return PushdownResult::Push;
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::X)) {
			*newState = new GameMaze(g);
			return PushdownResult::Push;
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE)) {
			return PushdownResult::Pop;
		}
		else {
			g->UpdateGame_main_menu(dt);
			return PushdownResult::NoChange;
		}
	}

	void OnAwake() override {
	}

	void PrintMessage() override {
		Debug::Print("Press Z to enter game.", Vector2(5, 15));
		Debug::Print("Press X to enter maze.", Vector2(5, 25));
		Debug::Print("Press ESC to quit game.", Vector2(5, 35));
	}

public:
	IntroScreen(TutorialGame* obj) :g(obj) {  }

protected:
	TutorialGame* g;
};
#pragma endregion
