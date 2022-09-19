#pragma once
#include "GameTechRenderer.h"
#include "StateGameObject.h"

#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "../CSC8503Common/NavigationPath.h"

#define GAME_MODE 1
#define MAZE_MODE 0

#define STEEL 0.597
#define TENNIS 0.712
#define WOODEN 0.603
#define PLASTIC 0.688

#define FLOOR 1

namespace NCL {
	namespace CSC8503 {
		class SphereState;

		class TutorialGame		{
			friend class SphereState;
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);
			virtual void UpdateGame_maze(float dt);
			virtual void UpdateGame_main_menu(float dt);

			void InitWorld();
			void InitMaze();

			bool LoseSign() { return world->GetLoseGame(); }
			void SetLoseSign(bool factor) { world->SetLoseGame(factor); }

			bool END = false;
			bool RESTART = false;

			GameObject* lockedObject = nullptr;
			GameObject* selectionObject = nullptr;

		protected:
			NavigationGrid* navigation_map;
			NavigationPath* navigation_path;
			vector<Vector3> good_path;
			vector<Vector3> path_record;

			vector<GameObject*> awards;

			SphereState* enemy;
			GameObject* player;

			void LoadMaze();
			void FindPath();
			void navigate_obj();
			void navigate_DEBUG();
			int path_counter = 1;

			void LockedObjectMovement_maze();
			void InitCamera_maze();

			//
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys(bool mode);

			//
			void InitGameExamples();

			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitDefaultFloor();
			void InitFrictionFloor();
	
			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();
			
			//Initation funcions
			void InitCubeobstacles();
			void InitBalls();
			void InitBonus();
			void InitNet();
			void InitStateMachine();

			//state machine
			std::vector<StateGameObject*> state_machine_objects;
			StateGameObject* AddCapsuleToWorld_SM(const Vector3& position, float halfHeight, float radius,
				float inverseMass = 1.0f, const Material& material = STEEL);
			SphereState* AddSphereToWorld_SM(const Vector3& position, float radius, float inverseMass, 
				bool selectable, const Vector4& colour, const Material& material = TENNIS);


			//floor and walls
			GameObject* AddFloorToWorld(const Vector3& position,const Vector3& shape,bool selectable = false,
				const Material& material = WOODEN, const float& fri = 0.3, 
				const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

			GameObject* AddHillToWorld(const Vector3& position, const Vector3& shape,const float &degree,
				const Material& material = WOODEN, const float& fri = 0.3,
				const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

			//
			GameObject* AddSphereToWorld(const Vector3& position, float radius,
				float inverseMass = 10.0f, bool selectable = true,
				const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f),
				const Material& material = TENNIS);

			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f,
				bool selectable = true, const Vector4& color = Vector4(1, 1, 1, 1), 
				const Material& material = STEEL);
			
			GameObject* AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, 
				float inverseMass = 10.0f, const Material& material = PLASTIC);

			GameObject* AddPlayerToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			GameObject* AddBonusToWorld(const Vector3& position, bool selectable = false);

			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;

			OGLMesh*	capsuleMesh = nullptr;
			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	charMeshA	= nullptr;
			OGLMesh*	charMeshB	= nullptr;
			OGLMesh*	enemyMesh	= nullptr;
			OGLMesh*	bonusMesh	= nullptr;

			//Coursework Additional functionality	
			Vector3 lockedOffset		= Vector3(0, 14, 20);
			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}
		};

		class SphereState : public GameObject {
		public:
			SphereState(TutorialGame* game);
			~SphereState();

			virtual void Update(float dt);

		protected:

			void RobMoney(float dt);
			void FollowWay(float dt);
			void Escape(float dt);

			StateMachine* stateMachine;
			float counter;
			TutorialGame* g;

			bool arrived;
			Vector3 Dir_to_Target;
		};
	}
}
