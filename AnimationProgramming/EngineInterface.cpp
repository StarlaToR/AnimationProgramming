#include "EngineInterface.h"



	void DrawLine(Vector3D Point1, Vector3D Point2, Color LineColor)
	{
		DrawLine(Point1.x, Point1.y, Point1.z, Point2.x, Point2.y, Point2.z, LineColor.x, LineColor.y, LineColor.z);
	}

	void GetSkeletonBoneLocalBindTransform(int boneIndex, Vector3D& position, Quaternion& rotation)
	{
		GetSkeletonBoneLocalBindTransform(boneIndex, position.x, position.y, position.z, rotation.w,rotation.x, rotation.y, rotation.z);
	}

	void GetAnimLocalBoneTransform(const char* animName, int boneIndex, int keyFrameIndex, Vector3D& position, Quaternion& rotation)
	{
		GetAnimLocalBoneTransform(animName, boneIndex, keyFrameIndex, position.x, position.y, position.z, rotation.w, rotation.x, rotation.y, rotation.z);
	}
