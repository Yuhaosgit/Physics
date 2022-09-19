#pragma once
#include "Transform.h"
#include "CollisionVolume.h"

#include "PhysicsObject.h"
#include "RenderObject.h"

#include <vector>

#define Material float

using std::vector;

namespace NCL {
	namespace CSC8503 {

		class GameObject {
		public:
			GameObject(string name = "");
			~GameObject();

			void SetBoundingVolume(CollisionVolume* vol) {
				boundingVolume = vol;
			}

			const CollisionVolume* GetBoundingVolume() const {
				return boundingVolume;
			}

			bool IsActive() const {
				return isActive;
			}

			Transform& GetTransform() {
				return transform;
			}

			RenderObject* GetRenderObject() const {
				return renderObject;
			}

			PhysicsObject* GetPhysicsObject() const {
				return physicsObject;
			}

			void SetRenderObject(RenderObject* newObject) {
				renderObject = newObject;
			}

			void SetPhysicsObject(PhysicsObject* newObject) {
				physicsObject = newObject;
			}

			const string& GetName() const {
				return name;
			}

			void SetName(const string &name_) {
				name = name_;
			}

			virtual void OnCollisionBegin(GameObject* otherObject) {
				//std::cout << "OnCollisionBegin event occured!\n";
			}

			virtual void OnCollisionEnd(GameObject* otherObject) {
				//std::cout << "OnCollisionEnd event occured!\n";
			}

			bool GetBroadphaseAABB(Vector3& outsize) const;

			void UpdateBroadphaseAABB();

			void SetWorldID(int newID) {
				worldID = newID;
			}

			int		GetWorldID() const {
				return worldID;
			}

			bool Selectable() const {
				return selectable;
			}

			void SetSelectable(bool factor) {
				selectable = factor;
			}

			bool IsBonus() const {
				return isBonus;
			}

			void SetIsBonus(bool factor) {
				isBonus = factor;
			}

			bool IsCollision() const {
				return collision;
			}

			void SetCollision(bool factor) {
				collision = factor;
			}
			void SetMaterial(const Material& m) {
				restitution = m;
			}

			Material GetMaterial() {
				return restitution;
			}

			int Type;
			float friction_coe;
			bool powerup = false;
		protected:
			Transform			transform;

			CollisionVolume*	boundingVolume;
			PhysicsObject*		physicsObject;
			RenderObject*		renderObject;

			bool	isActive;
			int		worldID;
			string	name;

			Vector3 broadphaseAABB;

			Material restitution;

			bool selectable;
			bool isBonus;

			bool collision;
		};
	}
}

