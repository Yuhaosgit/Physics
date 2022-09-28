#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../CSC8503Common/PositionConstraint.h"
#include "../../Common/ShareVariable.h"
#include "../../Common/Assets.h"


using namespace NCL;
using namespace CSC8503;

//
int MainBall;
int Hill;
int EvilBall;
//

void ShowDebugInformation(GameObject* obj);

float among_time = 0;

GameObject* CoreBall;
GameObject* WinnerBall;

Vector3 init_enemy_pos;

GameObject* net[6];
GameObject* constraint_ball[3];

TutorialGame::TutorialGame()	{
	world		= new GameWorld();
	renderer	= new GameTechRenderer(*world);
	physics		= new PhysicsSystem(*world);
	navigation_map = new NavigationGrid("TestGrid1.txt");
	navigation_path = new NavigationPath();

	forceMagnitude	= 10.0f;
	useGravity		= false;
	inSelectionMode = false;

	Debug::SetRenderer(renderer);

	InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh"		 , &cubeMesh);
	loadFunc("sphere.msh"	 , &sphereMesh);
	loadFunc("Male1.msh"	 , &charMeshA);
	loadFunc("courier.msh"	 , &charMeshB);
	loadFunc("security.msh"	 , &enemyMesh);
	loadFunc("coin.msh"		 , &bonusMesh);
	loadFunc("capsule.msh"	 , &capsuleMesh);

	basicTex	= (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMeshA;
	delete charMeshB;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

float fresh = 0;
void TutorialGame::UpdateGame_maze(float dt) {
	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
		world->GetMainCamera()->CowHorse();
	}

	Debug::DrawLine(enemy->GetTransform().GetPosition(), player->GetTransform().GetPosition(), Vector4(1, 0, 0.3, 1));
	/*
	fresh += dt;
	if (fresh > 1.0f) {
		good_path.clear();
		FindPath();
		fresh = 0;
	}
	*/

	//path_record.emplace_back(good_path[1]);


	UpdateKeys(MAZE_MODE);
	fresh += dt;

	if (CollisionDetection::ObjectIntersection(enemy, player, CollisionDetection::CollisionInfo())) {
		enemy->GetTransform().SetPosition(Vector3(80, -4, 80));
		FindPath();
	}

	if (fresh > 0.5)
		enemy->Update(dt);

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::M)) 
	{return;}

	physics->Update(dt);

	world->UpdateWorld(dt);
	renderer->Update(dt);

	Debug::FlushRenderables(dt);
	renderer->Render();
	//good_path.clear();
}


void TutorialGame::navigate_DEBUG() {
	for (int i = 1; i < good_path.size(); ++i) {
		Vector3 start_point = good_path[i - 1];
		Vector3 end_point = good_path[i];

		if (i % 2)
			Debug::DrawLine(start_point, end_point, Vector4(0, 1, 0, 1));
		else
			Debug::DrawLine(start_point, end_point, Vector4(1, 0, 0, 1));
	}
}

void TutorialGame::navigate_obj() {
	path_counter = 0;
	Vector3 velocity;
	Vector3 start_point = enemy->GetTransform().GetPosition();
	//if (path_counter - 1 == good_path.size() - 1)return;

	Vector3 end_point = good_path[path_counter + 1];
	//if (good_path[path_counter].z != good_path[path_counter + 1].z)
		//return;
	//	velocity = (*(path_record.end()-1) - good_path[path_counter]).Normalised() * 10;
//	else
	velocity = (end_point - start_point).Normalised() * 15;
	//velocity = (good_path[path_counter + 1] - good_path[path_counter]).Normalised() * 10;


	enemy->GetPhysicsObject()->SetLinearVelocity(velocity);
	/*
	Vector3 position = enemy->GetTransform().GetPosition();
	float distance = (end_point - position).Length();
	arrived = distance < point_radius[path_counter] + enemy->GetTransform().GetScale().x;

	while (arrived) {
		point_radius[path_counter] = -2.1;
		++path_counter;
		arrived = false;
	}
	*/
}


void TutorialGame::UpdateGame(float dt) {
	if (WinnerBall != nullptr && CoreBall != nullptr &&
		CollisionDetection::ObjectIntersection(WinnerBall, CoreBall, CollisionDetection::CollisionInfo())) {
		END = true;
	}

	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
		world->GetMainCamera()->CowHorse();
	}

	UpdateKeys(GAME_MODE);

	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(5, 95));
	}
	else {
		Debug::Print("(G)ravity off", Vector2(5, 95));
	}

	SelectObject();
	MoveSelectedObject();

	if (!state_machine_objects.empty()) {
		for (int i = 0; i < state_machine_objects.size(); ++i)
			state_machine_objects[i]->Update(dt);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::M))
	{return;}

	physics->Update(dt);

	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
		Vector3 camPos = objPos + lockedOffset;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0,1,0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(angles.x);
		world->GetMainCamera()->SetYaw(angles.y);

		//Debug::DrawAxisLines(lockedObject->GetTransform().GetMatrix(), 2.0f);
	}

	if (net[0] != nullptr) {
		for (int i = 0; i < 5; ++i) {
			Vector3 front = net[i]->GetTransform().GetPosition();
			Vector3 back = net[i + 1]->GetTransform().GetPosition();

			Debug::DrawLine(front, back, Vector4(1, 1, 0, 1));
		}
	}

	if (constraint_ball[0] != nullptr) {
		for (int i = 0; i < 2; ++i) {
			Vector3 front = constraint_ball[i]->GetTransform().GetPosition();
			Vector3 back = constraint_ball[i + 1]->GetTransform().GetPosition();

			Debug::DrawLine(front, back, Vector4(1, 1, 0, 1));
		}
	}

	world->UpdateWorld(dt);
	renderer->Update(dt);

	Debug::FlushRenderables(dt);
	renderer->Render();
}

void TutorialGame::UpdateGame_main_menu(float dt) {
	Debug::FlushRenderables(dt);
	renderer->Render();
}

void TutorialGame::UpdateKeys(bool mode) {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		if (mode == GAME_MODE)
			InitWorld(); //We can reset the simulation at any time with F1
		else
			InitMaze();
		selectionObject = nullptr;
		lockedObject	= nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		if (mode == GAME_MODE)
			InitCamera(); //F2 will reset the camera to a specific default place
		else
			InitCamera_maze(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G) && (mode == GAME_MODE)) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		if (mode == GAME_MODE)
			LockedObjectMovement();
		else
			LockedObjectMovement_maze();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Matrix4 view		= world->GetMainCamera()->BuildViewMatrix();
	Matrix4 camWorld	= view.Inverse();

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis.Normalise();

	Vector3 charForward  = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);
	Vector3 charForward2 = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);

	float force = 100.0f;

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		lockedObject->GetPhysicsObject()->AddForce(-rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		Vector3 worldPos = selectionObject->GetTransform().GetPosition();
		lockedObject->GetPhysicsObject()->AddForce(rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		lockedObject->GetPhysicsObject()->AddForce(fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		lockedObject->GetPhysicsObject()->AddForce(-fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NEXT)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(0,-10,0));
	}
}

void TutorialGame::LockedObjectMovement_maze() {

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -20));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 20));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(20, 0, 0));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(-20, 0, 0));
	}
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}

}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(20, 20, 20));
	lockedObject = nullptr;
}

void TutorialGame::InitCamera_maze() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-63);
	world->GetMainCamera()->SetYaw(275);
	world->GetMainCamera()->SetPosition(Vector3(11.5688, 73.0647, 48.1136));
	lockedObject = nullptr;
}

void TutorialGame::LoadMaze() {
	for (int i = 0; i < navigation_map->NodeNumber(); ++i) {
		GridNode* current_node = &navigation_map->allNodes[i];
		AddFloorToWorld(current_node->position - Vector3(0, 7, 0), Vector3(5, 2, 5));

		if (current_node->type == 'x')
			AddFloorToWorld(current_node->position, Vector3(5, 5, 5));
	}
}

void TutorialGame::FindPath() {
	bool found = navigation_map->FindPath(enemy->GetTransform().GetPosition(), player->GetTransform().GetPosition(), *navigation_path);
	Vector3 pos;

	//point_radius.clear();

	while (navigation_path->PopWaypoint(pos)) {
		good_path.emplace_back(pos - Vector3(0, 4, 0));
		//point_radius.emplace_back(2);
	}
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();
	state_machine_objects.clear();
	END = false;
	inSelectionMode = false;
	InitCamera();
	useGravity = true;
	physics->UseGravity(useGravity);

	//InitMixedGridWorld(5, 5, 3.5f, 3.5f);
	//InitGameExamples();
	//BridgeConstraintTest();

	/****************demo*****************/
	InitDefaultFloor();
	InitBalls();
	InitCubeobstacles();
	InitBonus();
	InitNet();
	InitStateMachine();
	//InitDebug();
}

void TutorialGame::InitDebug() {
	AddCapsuleToWorld(Vector3(0, 10, 0), 4, 2);
	AddCubeToWorld(Vector3(0, 4, 4), Vector3(2, 2, 2));
	AddFloorToWorld(Vector3(0, -2, 0), Vector3(100, 2, 50), false, WOODEN, 0.8, Vector4(0.8, 0, 0.1, 1));
}

void TutorialGame::InitMaze() {
	world->ClearAndErase();
	physics->Clear();
	state_machine_objects.clear();
	END = false;
	inSelectionMode = false;
	InitCamera_maze();
	good_path.clear();
	useGravity = false;
	physics->UseGravity(useGravity);
	player = nullptr;
	enemy = nullptr;
	lockedObject = nullptr;


	LoadMaze();
	player = AddSphereToWorld_SM(Vector3(80, -4, 10), 1.0f, 0.5f, false, Vector4(0.6, 1, 0.8, 1));
	player->SetName("pl");
	lockedObject = player;

	enemy = AddSphereToWorld_SM(Vector3(80, -4, 80), 1.0f, 0.5f, false, Vector4(0.7, 0.2, 0.8, 1));
	enemy->SetName("en");
	FindPath();
	
	awards.emplace_back(AddBonusToWorld(Vector3(19.2446, -3, 34.8054)));
	awards.emplace_back(AddBonusToWorld(Vector3(15.9682, -3, 15.9421)));
	awards.emplace_back(AddBonusToWorld(Vector3(75.0297, -3, 43.1354)));
	awards.emplace_back(AddBonusToWorld(Vector3(8.81235, -3, 62.1983)));
}

void TutorialGame::InitStateMachine() {
	AddCapsuleToWorld_SM(Vector3(20, 2, 0), 2, 1, 1);
	AddCapsuleToWorld_SM(Vector3(0, 2, 0), 2, 1, 1);
	AddCapsuleToWorld_SM(Vector3(-20, 2, 0), 2, 1, 1);
}

SphereState* TutorialGame::AddSphereToWorld_SM(const Vector3& position, float radius, float inverseMass,
	bool selectable, const Vector4& colour, const Material& material) {
	SphereState* sphere = new SphereState(this);

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);

	sphere->SetSelectable(selectable);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	sphere->GetRenderObject()->SetColour(colour);
	sphere->SetMaterial(material);

	world->AddGameObject(sphere);

	return sphere;
}

StateGameObject* TutorialGame::AddCapsuleToWorld_SM
(const Vector3& position, float halfHeight, float radius, float inverseMass, const Material& material) {
	StateGameObject* capsule = new StateGameObject();

	CapsuleVolume* volume = new CapsuleVolume(2, 1);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(radius * 2, halfHeight, radius * 2))
		.SetPosition(position);

	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitCubeInertia();
	capsule->SetMaterial(material);

	world->AddGameObject(capsule);

	state_machine_objects.emplace_back(capsule);
	return capsule;
}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
void TutorialGame::InitDefaultFloor() {
	//floor
	AddFloorToWorld(Vector3(0, -2, 50), Vector3(100, 2, 50), false, WOODEN, 0.8, Vector4(0.8, 0, 0.1, 1));
	AddFloorToWorld(Vector3(0, -2, -50), Vector3(100, 2, 50), false, WOODEN, 0.2, Vector4(0.7, 1, 1, 1));
	//walls
	AddFloorToWorld(Vector3(102, 26, 0), Vector3(2, 10, 100));
	AddFloorToWorld(Vector3(102, 6, -75), Vector3(2, 10, 25));
	AddFloorToWorld(Vector3(102, 6, 75), Vector3(2, 10, 25));

	AddFloorToWorld(Vector3(0, 6, 102), Vector3(100, 10, 2));
	AddFloorToWorld(Vector3(-102, 6, 0), Vector3(2, 10, 100));
	AddFloorToWorld(Vector3(0, 6, -102), Vector3(100, 10, 2));
	//hill
	Hill = AddHillToWorld(Vector3(142, -27.1, 0), Vector3(50, 4, 50), -0.25f)->GetWorldID();
	//walls
	AddFloorToWorld(Vector3(150, -10, 54), Vector3(45, 50, 5));
	AddFloorToWorld(Vector3(150, -10, -54), Vector3(45, 50, 5));
	//floor
	AddFloorToWorld(Vector3(240, -52, 0), Vector3(50, 1, 50));
}

void TutorialGame::InitFrictionFloor() {
	AddFloorToWorld(Vector3(0, 2, 0), Vector3(50, 1, 50));
}


void TutorialGame::InitCubeobstacles() {
	AddCubeToWorld(Vector3(-39.0956, 3, -60.1608), Vector3(3, 3, 3), 0, false);
	//AddCubeToWorld(Vector3(33.2131, 3, -33.3821), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(34.3907, 3, 25.3634), Vector3(3, 3, 3), 0, false);
	//AddCubeToWorld(Vector3(-42.6696, 3, 87.8086), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(-53.6564, 3, 14.9262), Vector3(3, 3, 3), 0, false);

	//AddCubeToWorld(Vector3(-24.6363, 3, -36.5407), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(26.2937, 3, -53.2304), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(-12.0952, 3, 44.6059), Vector3(3, 3, 3), 0, false);
	//AddCubeToWorld(Vector3(-56.9423, 3, 35.0148), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(-76.2077, 3, -35.0301), Vector3(3, 3, 3), 0, false);

	AddCubeToWorld(Vector3(77.0449, 3, -43.6588), Vector3(3, 3, 3), 0, false);
	//AddCubeToWorld(Vector3(24.0634, 3, 66.4502), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(-74.06, 3, 53.5381), Vector3(3, 3, 3), 0, false);
	//AddCubeToWorld(Vector3(-72.1362, 3, -67.7148), Vector3(3, 3, 3), 0, false);
	AddCubeToWorld(Vector3(60.3237, 3, 1.87562), Vector3(3, 3, 3), 0, false);

	//lose game
	EvilBall = AddCubeToWorld(Vector3(90, 3, 90), Vector3(3, 3, 3), 0, false, Vector4(0.4f, 0.0f, 0.6f, 1))->GetWorldID();
	WinnerBall = AddCubeToWorld(Vector3(240, -47, 0), Vector3(3, 3, 3), 0, false, Vector4(0.6f, 1.0f, 1.0f, 1));
}

void TutorialGame::InitBalls() {
	CoreBall = AddSphereToWorld(Vector3(-70, 2, 0), 3, 1.0f, false, Vector4(1, 1, 0.1, 1));
	GameObject *Move = AddSphereToWorld(Vector3(-70, 2, 10), 2, 1);

	PositionConstraint* constraint_3 = new PositionConstraint(CoreBall, Move, 10);
	world->AddConstraint(constraint_3);

	MainBall = CoreBall->GetWorldID();
	//
	constraint_ball[0] = AddSphereToWorld(Vector3(-80, 2, 10), 2, 1);
	constraint_ball[1] = AddSphereToWorld(Vector3(-80, 2, 0), 2, 1);
	constraint_ball[2] = AddSphereToWorld(Vector3(-80, 2, -10), 2, 1);

	GameObject* previous = constraint_ball[0];

	PositionConstraint* constraint_1 = new PositionConstraint(constraint_ball[0], constraint_ball[1], 10);
	world->AddConstraint(constraint_1);

	PositionConstraint* constraint_2 = new PositionConstraint(constraint_ball[1], constraint_ball[2], 10);
	world->AddConstraint(constraint_2);

	//debug
	//AddSphereToWorld(Vector3(100, 2, -10), 1, 1);
}

void TutorialGame::InitBonus() {
	AddBonusToWorld(Vector3(-50.8839, 2, 53.262));
	AddBonusToWorld(Vector3(-14.3767, 2, 70.9904));
	AddBonusToWorld(Vector3(38.6655, 2, 69.7192));
	AddBonusToWorld(Vector3(65.0617, 2, 75.8327));
	AddBonusToWorld(Vector3(63.9995, 2, 31.4343));

	AddBonusToWorld(Vector3(37.3431, 2, 1.79287));
	AddBonusToWorld(Vector3(87.9327, 2, -11.153));
	AddBonusToWorld(Vector3(55.7177, 2, -44.2193));
	AddBonusToWorld(Vector3(84.5328, 2, -86.1322));
	AddBonusToWorld(Vector3(49.9206, 2, -83.7066));

	AddBonusToWorld(Vector3(6.36869, 2, -68.9976));
	AddBonusToWorld(Vector3(-16.1939, 2, -83.5141));
	AddBonusToWorld(Vector3(-63.3099, 2, -80.3847));
	AddBonusToWorld(Vector3(-34.8406, 2, -31.3791));
	AddBonusToWorld(Vector3(-13.1139, 2, 18.3644));
}

void TutorialGame::InitNet() {
	Vector3 offset = Vector3(0, 0, -30);

	//net[0] = AddCapsuleToWorld(Vector3(-30, 2, 0) + offset, 2, 1);
	net[0] = AddSphereToWorld(Vector3(-30, 2, 0) + offset, 2, 1);
	net[1] = AddCubeToWorld(Vector3(-30, 2, 10) + offset, Vector3(2, 2, 2));
	net[2] = AddSphereToWorld(Vector3(-30, 2, 20) + offset, 2, 1);

	//net[3] = AddCapsuleToWorld(Vector3(-30, 2, 30) + offset, 2, 1);
	net[3] = AddSphereToWorld(Vector3(-30, 2, 30) + offset, 2, 1);
	net[4] = AddCubeToWorld(Vector3(-30, 2, 40) + offset, Vector3(2, 2, 2), 1);
	net[5] = AddSphereToWorld(Vector3(-30, 2, 50) + offset, 2, 1);

	
	GameObject* previous = net[0];

	for (int i = 1; i < 6; ++i) {
		PositionConstraint* constraint = new PositionConstraint(previous, net[i], 5);
		world->AddConstraint(constraint);
		previous = net[i];
	}
}


GameObject* TutorialGame::AddHillToWorld(const Vector3& position, const Vector3& shape,
	const float& degree, const Material& material, const float& fri, const Vector4& colour) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = shape;
	OBBVolume* volume = new OBBVolume(floorSize);

	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetScale(floorSize * 2).SetPosition(position).SetOrientation(Quaternion(0, 0, degree, 1));

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	floor->SetMaterial(material);
	floor->Type = FLOOR;

	floor->friction_coe = fri;


	world->AddGameObject(floor);

	return floor;
}

GameObject* TutorialGame::AddFloorToWorld(const Vector3& position, const Vector3& shape, 
	bool selectable, const Material& material, const float& fri, const Vector4& colour) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = shape;
	AABBVolume* volume = new AABBVolume(floorSize);

	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetScale(floorSize * 2).SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	floor->SetMaterial(material);
	floor->Type = FLOOR;
	floor->SetSelectable(selectable);
	floor->GetRenderObject()->SetColour(colour);
	floor->friction_coe = fri;

	world->AddGameObject(floor);

	return floor;
}


/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass,
	bool selectable, const Vector4& colour, const Material& material) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);

	sphere->SetSelectable(selectable);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	sphere->GetRenderObject()->SetColour(colour);
	sphere->SetMaterial(material);

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCapsuleToWorld(const Vector3& position, float halfHeight,
	float radius, float inverseMass, const Material& material) {
	GameObject* capsule = new GameObject();

	CapsuleVolume* volume = new CapsuleVolume(halfHeight, radius);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(radius * 2, halfHeight, radius * 2))
		.SetPosition(position);

	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitCubeInertia();
	capsule->SetMaterial(material);

	world->AddGameObject(capsule);

	return capsule;

}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions,
	float inverseMass,bool selectable,const Vector4 &colour, const Material& material) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);
	cube->SetSelectable(selectable);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->GetRenderObject()->SetColour(colour);
	cube->SetMaterial(material);

	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0), Vector3(100, 2, 100));
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

void TutorialGame::InitGameExamples() {
	AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.85f, 0.3f) * meshSize);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	if (rand() % 2) {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshA, nullptr, basicShader));
	}
	else {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshB, nullptr, basicShader));
	}
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	//lockedObject = character;

	return character;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position,bool selectable) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(3.0f);
	apple->SetBoundingVolume((CollisionVolume*)volume);

	apple->SetSelectable(selectable);
	apple->SetIsBonus(true);

	apple->GetTransform()
		.SetScale(Vector3(0.4, 0.4, 0.4))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(0.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	apple->GetRenderObject()->SetColour(Vector4(1, 1, 0.4, 1));

	world->AddGameObject(apple);

	return apple;
}

/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		renderer->DrawString("Press Q to change to camera mode!", Vector2(5, 85));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
				lockedObject	= nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				if (!((GameObject*)closestCollision.node)->Selectable())
					return false;
				//select aim
				selectionObject = (GameObject*)closestCollision.node;
				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
		if (selectionObject != nullptr)
			ShowDebugInformation(selectionObject);
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(5, 85));
	}

	if (lockedObject) {
		renderer->DrawString("Press L to unlock object!", Vector2(5, 80));
	}

	else if(selectionObject){
		renderer->DrawString("Press L to lock selected object object!", Vector2(5, 80));
	}

	if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
		if (selectionObject) {
			if (lockedObject == selectionObject) {
				lockedObject = nullptr;
			}
			else {
				lockedObject = selectionObject;
			}
		}

	}

	return false;
}

void ShowDebugInformation(GameObject* obj) {
	Vector3 pos = obj->GetTransform().GetPosition();
	Quaternion orientation = obj->GetTransform().GetOrientation();
	Vector3 dir = orientation.ToEuler();
	Vector3 ang_vel = obj->GetPhysicsObject()->GetAngularVelocity();
	Vector3 linear_vel = obj->GetPhysicsObject()->GetLinearVelocity();

	Debug::Print("Position:"+
		std::to_string((int)pos.x)+"," + std::to_string((int)pos.y)+"," + std::to_string((int)pos.z),
		Vector2(60, 10));
	Debug::Print("Orientation:" +
		std::to_string((int)dir.x) + "," + std::to_string((int)dir.y) + "," + std::to_string((int)dir.z),
		Vector2(60, 15));
	Debug::Print("L Velocity:" +
		std::to_string((int)linear_vel.x) + "," + std::to_string((int)linear_vel.y) + "," + std::to_string((int)linear_vel.z),
		Vector2(60, 20));
	Debug::Print("A Velocity:" +
		std::to_string((int)ang_vel.x) + "," + std::to_string((int)ang_vel.y) + "," + std::to_string((int)ang_vel.z),
		Vector2(60, 25));
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/
void TutorialGame::MoveSelectedObject() {
	//Draw debug text at 10, 20
	renderer->DrawString("Click Force:" + std::to_string(forceMagnitude),Vector2(10, 20));

	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject)
		return;//we haven¡¯t selected anything!

	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

		RayCollision closestCollision;

		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject)
				selectionObject->GetPhysicsObject()->
				AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
		}
	}
}
