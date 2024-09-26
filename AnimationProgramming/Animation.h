#pragma once
#include "mathLibrary.hpp"
#include <vector>
namespace Animation
{
	class Anim
	{
	public:

		Anim();
		Anim(const char* animName, int boneCount);
		~Anim();
		Vector3D GetPosition(int keyFrame, int boneIndex);
		Quaternion GetRotation(int keyFrame, int boneIndex);
		void ChangeKeyFrame(bool forward =true);
		void UpdateAnimationTime(double multiplier, float animSpeed);
		int GetKeyFrame() { return m_currentKeyFrame; };
		int GetMaxKeyFrame() { return m_keyFrameCount - 1; };
		double animationTimer;
	private:
		int m_keyFrameCount = 0;
		int m_currentKeyFrame = 0;
		int m_boneCount = 0;
		std::string m_name;
		std::vector<Vector3D> m_position;
		std::vector<Quaternion>m_rotation;

		void InitAnimation();
	};

}