#pragma once
#include "Engine.h"
#include "mathLibrary.hpp"


void DrawLine(Vector3D Point1, Vector3D Point2, Color LineColor);

void GetSkeletonBoneLocalBindTransform(int boneIndex, Vector3D& position, Quaternion& rotation);

void GetAnimLocalBoneTransform(const char* animName, int boneIndex, int keyFrameIndex, Vector3D& position, Quaternion& rotation);


