#pragma once
#include <vector>

#include "Bone.h"
#include "EngineInterface.h"
#include "Animation.h"
#include "Animator.h"

namespace Animation
{
	class Skeleton
	{
	private:
		int m_boneCount = 62;
		std::vector<Bone> m_bones;
		Vector3D m_offsetBodyPosition;
		Color m_boneColor;
		Animator m_animator;
	public:
		bool isDrawing = false;
		Skeleton(Vector3D pos = Vector3D(), Vector4D col = Vector4D(0,1,0));
		~Skeleton();
		void Draw();
		void Init();
		void SetAnim(const char* name);
		
		void ChangeAnim(int index);
		void PlayAnimation(bool active);
		void FrameForward(bool forward);

		void Update(float mul);
		void CalculateBonesPositions( std::vector<Matrix4D>& outPose);
		int GetBoneCount() { return m_boneCount; }
		Animator GetAnimator () { return m_animator; }
	};
}