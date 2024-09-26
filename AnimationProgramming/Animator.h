#pragma once
#include "Animation.h"
#include <vector>

namespace Animation
{
	class Animator
	{
	public:
		Animator();
		~Animator();

		void Update(float mul);
		Anim* GetCurrentAnimation();
		Anim* GetNextAnimation() { if(m_nextAnim) return m_nextAnim; };
		Matrix4D GetAnimationMatrix(int boneIndex);
		void StartTransition(int nextAnimationIndex, float transitionTime);
		void AddAnimation(const char* name);
		void RunAnimation(float speedAnimation);
		void TransitionBetweenAnimation();
		void FinishTransition();
		void PlayAnimation(bool active) { m_activeAnimation = active; }
		void PlayOneFrame(bool forward);

	private:
		void GetAnimationTransform(int boneIndex, Vector3D& position, Quaternion& rotation, Anim* animation);
		std::vector<Anim> animations;
		Anim* m_currentAnim = nullptr;
		Anim* m_nextAnim = nullptr;

		float m_frameTimer = 0.f;
		float m_transitionTime = 0.f;
		float m_transitionCountdown = 0.f;

		int m_bonesCount = 64;
		int m_currentAnimationIndex = 0;
		int m_nextAnimationIndex = 0;
		bool m_onTransition = false;
		bool m_activeAnimation = true;
		float animSpeed = 1.f / 30.f;

		float m_test = 0;
		float m_frame = 0;
	};
}