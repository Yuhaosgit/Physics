#include "../../Common/Window.h"

#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

#include "../CSC8503Common/NavigationGrid.h"

#include "TutorialGame.h"
#include "PushdownMachine.h"
#include "PushDownState.h"

#include "BehaviourTree.h"

using namespace NCL;
using namespace CSC8503;

/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead. 

This time, we've added some extra functionality to the window class - we can
hide or show the 

*/

int main() {

	Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720, false);
	if (!w->HasInitialised()) {
		return -1;
	}	
	srand(time(0));
	w->ShowOSPointer(false);
	w->LockMouseToWindow(true);

	TutorialGame* g = new TutorialGame();
	PushdownMachine machine(new IntroScreen(g));

	w->GetTimer()->GetTimeDeltaSeconds(); //Clear the timer so we don't get a larget first dt!
	while (w->UpdateWindow() && !Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) {
		
		float dt = w->GetTimer()->GetTimeDeltaSeconds();

		if (dt > 0.1f) {
			std::cout << "Skipping large time delta" << std::endl;
			continue; //must have hit a breakpoint or something to have a 1 second frame time!
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			w->ShowConsole(true);
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			w->ShowConsole(false);
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::T)) {
			w->SetWindowPosition(0, 0);
		}

		machine.Update(dt);

		w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));
	}
	Window::DestroyGameWindow();
}