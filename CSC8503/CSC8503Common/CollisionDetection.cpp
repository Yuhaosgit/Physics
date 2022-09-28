#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "../../Common/Vector2.h"
#include "../../Common/Window.h"
#include "../../Common/Maths.h"
#include "Debug.h"

#include <list>

using namespace NCL;

Vector3 ClosestPointOnLineSegment(Vector3 pointA, Vector3 pointB, Vector3 point);

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector3::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector3::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;
		case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	if (hasCollided)
		object.SetCollision(true);

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) {
		if (rayDir[i] > 0)
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		else if (rayDir[i] < 0)
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
	}

	float bestT = tVals.GetMaxElement();
	
	if (bestT < 0.0f)
		return false;

	Vector3 intersection = rayPos + (rayDir * bestT);
	const float epsilon = 0.0001f; //an amount of leeway in our calcs

	for (int i = 0; i < 3; ++i) {
		if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i])
			return false; //best intersection doesn touch the box!
	}

	collision.collidedAt = intersection;
	collision.rayDistance = bestT;

	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos = worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();

	return RayBoxIntersection(r, boxPos, boxSize, collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();

	Matrix3 transform = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;

	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);

	if (collided)
		collision.collidedAt = transform * collision.collidedAt + position;

	return collided;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {
	//transform coordination
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();

	Matrix3 transform = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;

	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());
	
	//capsule-ray
	const Vector3 &ray_origin = tempRay.GetPosition();
	const Vector3 &ray_direction = tempRay.GetDirection();
	//the two points of capsule
	Vector3 pointA = Vector3(0, volume.GetHalfHeight() / 2, 0);
	Vector3 pointB = Vector3(0, -volume.GetHalfHeight() / 2, 0);

	Vector3 line_segment = pointB - pointA;
	Vector3 ray_to_pointA = ray_origin - pointA;
	Vector3 ray_to_pointB = ray_origin - pointB;

	float radius = volume.GetRadius();

	//absolute value of middle line
	float ABS_line = Vector3::Dot(line_segment, line_segment);
	//absolute value of ray's position to point A
	float ABS_rayToA = Vector3::Dot(ray_to_pointA, ray_to_pointA);
	//the degree of middle line and ray's position to point A
	float proj_line_rayA = Vector3::Dot(line_segment, ray_to_pointA);
	//the degree of middle line and ray's direction
	float proj_line_rayDir= Vector3::Dot(line_segment, ray_direction);
	//the degree of ray's direction to ray's position to point A
	float proj_rayDir_rayA = Vector3::Dot(ray_direction, ray_to_pointA);

	float line_ = ABS_line - (proj_line_rayDir * proj_line_rayDir);

	float R_P = (ABS_line * proj_rayDir_rayA) - (proj_line_rayA * proj_line_rayDir);

	float c = (ABS_line * ABS_rayToA) - (proj_line_rayA * proj_line_rayA) - (radius * radius * ABS_line);

	float h = R_P * R_P - line_ * c;

	if (h >= 0.0)
	{
		float t = (-R_P - sqrt(h)) / line_;
		float y = proj_line_rayA + t * proj_line_rayDir;
		// body
		if (y > 0.0 && y < ABS_line) {
			collision.rayDistance = t;
			collision.collidedAt = tempRay.GetPosition() + (tempRay.GetDirection() * collision.rayDistance);
			collision.collidedAt = transform * collision.collidedAt + position;
			return true;
		}
		// caps
		Vector3 oc = (y <= 0.0) ? ray_to_pointA : ray_to_pointB;

		R_P = Vector3::Dot(ray_direction, oc);
		c = Vector3::Dot(oc, oc) - radius * radius;

		h = R_P * R_P - c;

		if (h > 0.0) {
			collision.rayDistance = -R_P - sqrt(h);
			collision.collidedAt = tempRay.GetPosition() + (tempRay.GetDirection() * collision.rayDistance);
			collision.collidedAt = transform * collision.collidedAt + position;
			return true;
		}
	}
	return false;
}

bool CollisionDetection::RaySphereIntersection(const Ray& r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos = worldTransform.GetPosition();
	float sphereRadius = volume.GetRadius();

	//Get the direction between the ray origin and the sphere origin
	Vector3 dir = (spherePos - r.GetPosition());

	//Then project the sphere origin onto our ray direction vector
	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false; // point is behind the ray!
	}

	//Get closest point on ray line to sphere
	Vector3 point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = (point - spherePos).Length();

	if (sphereDist > sphereRadius) {
		return false;
	}
	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);

	return true;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix4::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const Camera& cam) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	float aspect	= screenSize.x / screenSize.y;
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const Camera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2 screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c.Normalise();

	//std::cout << "Ray Direction:" << c << std::endl;

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0]  = aspect / h;
	m.array[5]  = tan(fov*PI_OVER_360);

	m.array[10] = 0.0f;
	m.array[11] = 1.0f / d;

	m.array[14] = 1.0f / e;

	m.array[15] = -c / (d*e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
Matrix4::Translation(position) *
Matrix4::Rotation(yaw, Vector3(0, 1, 0)) *
Matrix4::Rotation(pitch, Vector3(1, 0, 0));

return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const Camera &c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}

	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		if (SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo)) {
			a->SetCollision(true);
			return true;
		}
		return false;
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		if (SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo)) {
			b->SetCollision(true);
			return true;
		}
		return false;
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Capsule) {
		if (CapsuleIntersection((CapsuleVolume&)*volB, transformB,
			(CapsuleVolume&)*volA, transformA, collisionInfo)) {
			a->SetCollision(true);
			b->SetCollision(true);
			return true;
		}
		return false;
	}


	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Capsule) {
		return AABBCapsuleIntersection((AABBVolume&)*volA, transformA, 
			(CapsuleVolume&)*volB, transformB, collisionInfo);
	}
	
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((AABBVolume&)*volB, transformB, 
			(CapsuleVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA,
			(SphereVolume&)*volB, transformB, collisionInfo);
	}

	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB,
			(SphereVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB,
	const Vector3& halfSizeA, const Vector3& halfSizeB) {

	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x && abs(delta.y) < totalSize.y && abs(delta.z) < totalSize.z)
		return true;

	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 boxASize = volumeA.GetHalfDimensions();
	Vector3 boxBSize = volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	if (overlap) {
		static const Vector3 faces[6] = {
			Vector3(-1, 0, 0), Vector3(1, 0, 0),
			Vector3(0, -1, 0), Vector3(0, 1, 0),
			Vector3(0, 0, -1), Vector3(0, 0, 1),
		};
		
		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] = {
			(maxB.x - minA.x),// distance of box ¡¯b¡¯ to ¡¯left¡¯ of ¡¯a¡¯.
			(maxA.x - minB.x),// distance of box ¡¯b¡¯ to ¡¯right¡¯ of ¡¯a¡¯.
			(maxB.y - minA.y),// distance of box ¡¯b¡¯ to ¡¯bottom ¡¯ of ¡¯a¡¯.
			(maxA.y - minB.y),// distance of box ¡¯b¡¯ to ¡¯top¡¯ of ¡¯a¡¯.
			(maxB.z - minA.z),// distance of box ¡¯b¡¯ to ¡¯far¡¯ of ¡¯a¡¯.
			(maxA.z - minB.z) // distance of box ¡¯b¡¯ to ¡¯near¡¯ of ¡¯a¡¯.
		};

		float penetration = FLT_MAX;
		Vector3 bestAxis;

		for (int i = 0; i < 6; i++) {
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		collisionInfo.AddContactPoint(Vector3(), Vector3(), bestAxis, penetration);

		return true;
	}

	return false;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	const float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we¡¯re colliding!
	}

	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeA.GetHalfDimensions();
	
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	
	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();
	
	if (distance < volumeB.GetRadius()) {//yes , we¡¯re colliding!
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();//
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}

	return false;
}

bool CollisionDetection::OBBSphereIntersection(
	const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	//transform coordination
	Transform temp_sphere = worldTransformB;

	Quaternion orientation_obb = worldTransformA.GetOrientation();
	Vector3 position_obb = worldTransformA.GetPosition();
	Vector3 position_sphere = worldTransformB.GetPosition();

	Matrix3 transform_obb = Matrix3(orientation_obb);
	Matrix3 inv_transform_obb = Matrix3(orientation_obb.Conjugate());

	Vector3 local_sphere_pos = position_obb - position_sphere;
	
	temp_sphere.SetPosition(inv_transform_obb * local_sphere_pos);
	//
	Vector3 boxSize = volumeA.GetHalfDimensions();

	Vector3 delta = -temp_sphere.GetPosition();

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::I))
		return true;

	if (distance < volumeB.GetRadius()) {//yes , we¡¯re colliding!
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();
		Vector3 localB =  -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}

	return false;
}


bool CollisionDetection::OBBIntersection(
	const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	return false;
}

bool CollisionDetection::SphereCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float halfheight = volumeA.GetHalfHeight();
	float capsule_radius = volumeA.GetRadius();
	
	Vector3 pointA = Vector3(0, volumeA.GetHalfHeight() / 2, 0);
	Vector3 pointB = Vector3(0, -volumeA.GetHalfHeight() / 2, 0);
	Vector3 middle_line = worldTransformA.GetOrientation() * (pointA - pointB);
	middle_line.Normalise();

	Vector3 sphere_position = worldTransformB.GetPosition();
	Vector3 capsule_position = worldTransformA.GetPosition();

	Vector3 relevent_vector = sphere_position - capsule_position;

	float proj_relevent_to_middleline = (Vector3::Dot(middle_line, relevent_vector));

	Vector3 best_point;

	if (proj_relevent_to_middleline < (halfheight - capsule_radius)) {
		best_point = capsule_position - middle_line * (halfheight - capsule_radius);
	}
	else if (proj_relevent_to_middleline > (halfheight - capsule_radius)) {
		best_point = capsule_position + middle_line * (halfheight - capsule_radius);
	}
	else {
		best_point = capsule_position + middle_line * proj_relevent_to_middleline;
	}

	//sphere collide
	Vector3 delta = sphere_position - best_point;

	float deltaLength = delta.Length();
	float radii = volumeA.GetRadius() + volumeB.GetRadius();


	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();

		Vector3 localA = normal * capsule_radius + (best_point - worldTransformA.GetPosition());
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);

		return true;
	}

	return false;
}

bool CollisionDetection::CapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 capsule1_position = worldTransformA.GetPosition();
	Vector3 capsule2_position = worldTransformB.GetPosition();

	Vector3 pointA_cap1 = capsule1_position +
		worldTransformA.GetOrientation() * Vector3(0, volumeA.GetHalfHeight() / 2, 0);
	Vector3 pointB_cap1 = capsule1_position + 
		worldTransformA.GetOrientation() * Vector3(0, -volumeA.GetHalfHeight() / 2, 0);
	
	Vector3 pointA_cap2 = capsule2_position +
		worldTransformB.GetOrientation() * Vector3(0, volumeB.GetHalfHeight() / 2, 0);
	Vector3 pointB_cap2 = capsule2_position +
		worldTransformB.GetOrientation() * Vector3(0, -volumeB.GetHalfHeight() / 2, 0);

	Vector3 A1A2 = pointA_cap2 - pointA_cap1;
	Vector3 A1B2 = pointB_cap2 - pointA_cap1;

	Vector3 B1A2 = pointA_cap2 - pointB_cap1;
	Vector3 B1B2 = pointB_cap2 - pointB_cap1;

	Vector3 bestA, bestB;
	//
	float length_A1_A2 = A1A2.LengthSquared();
	float length_A1_B2 = A1B2.LengthSquared();
	float length_B1_A2 = B1A2.LengthSquared();
	float length_B1_B2 = B1B2.LengthSquared();

	if (length_A1_A2 > length_B1_B2) {
		bestA = pointB_cap1;
		bestB = ClosestPointOnLineSegment(pointA_cap2, pointB_cap2, bestA);
		bestA = ClosestPointOnLineSegment(pointA_cap1, pointB_cap1, bestB);
	}
	else {
		bestA = pointA_cap1;
		bestB = ClosestPointOnLineSegment(pointB_cap2, pointA_cap2, bestA);
		bestA = ClosestPointOnLineSegment(pointB_cap1, pointA_cap1, bestB);
	}

	//
	float radii = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = bestA - bestB;

	const float deltaLength = delta.Length();

	//debug
	//if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::I))
	//	return true;

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we¡¯re colliding!
	}

	return false;
}

Vector3 ClosestPointOnLineSegment(Vector3 pointA, Vector3 pointB, Vector3 point)
{
	Vector3 line = pointB - pointA;
	Vector3 pointA_to_point = point - pointA;

	float proj_to_line = Vector3::Dot(pointA_to_point, line) / Vector3::Dot(line, line);
	
	return pointA + line * Maths::Clamp(proj_to_line, 0.0f, 1.0f);
}

float GetT(const Vector3& planeNormal, const float& planeDistance, const Vector3& startPoint, const Vector3& endPoint) {
	return (planeDistance - Vector3::Dot(startPoint, planeNormal)) / (Vector3::Dot(planeNormal, endPoint - startPoint));
}

float GetBest(const float* values) {
	float result = INFINITE;
	for (int i = 0; i < 6; ++i) {
		if (fabsf(values[i]) < fabsf(result)) {
			result = values[i];
		}
	}
	return result;
}

bool CollisionDetection::AABBCapsuleIntersection(
	const AABBVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	const Vector3& boxSize = volumeA.GetHalfDimensions();

	Vector3 capsuleDirection = worldTransformB.GetOrientation() * Vector3(0, 1.0f, 0);
	Vector3 startPoint = worldTransformB.GetPosition() - capsuleDirection * volumeB.GetHalfHeight()*0.5
		- worldTransformA.GetPosition();
	Vector3 endPoint = worldTransformB.GetPosition() + capsuleDirection * volumeB.GetHalfHeight()*0.5
		- worldTransformA.GetPosition();

	float valueT[6];

	valueT[0] = GetT(Vector3(1, 0, 0), boxSize[0], startPoint, endPoint);
	valueT[1] = GetT(Vector3(-1, 0, 0), boxSize[0], startPoint, endPoint);
	valueT[2] = GetT(Vector3(0, 1, 0), boxSize[1], startPoint, endPoint);
	valueT[3] = GetT(Vector3(0, -1, 0), boxSize[1], startPoint, endPoint);
	valueT[4] = GetT(Vector3(0, 0, 1), boxSize[2], startPoint, endPoint);
	valueT[5] = GetT(Vector3(0, 0, -1), boxSize[2], startPoint, endPoint);

	float bestT = Clamp<float>(GetBest(valueT), 0, 1);

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::I))
		bool a = true;

	Vector3 startToEnd = endPoint - startPoint;
	Vector3 result = startPoint + startToEnd * bestT;
	
	Vector3 cloesetPoint = Clamp(result, -volumeA.GetHalfDimensions(), volumeA.GetHalfDimensions());
	bestT = Vector3::Dot((cloesetPoint - startPoint), startToEnd) / startToEnd.LengthSquared();
	bestT = Clamp<float>(bestT, 0, 1);
	result = startPoint + startToEnd * bestT;

	Vector3 localPoint = result - cloesetPoint;
	float distance = localPoint.Length();

	if (distance <= volumeB.GetRadius()) {//yes, we¡¯re colliding!
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();//
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}
	return false;
}