#include "Time.h"

#include "Animator.h"
#include <algorithm>

namespace Animation
{
	Animator::Animator()
	{
		m_currentAnimationIndex = 0;
	}

	Animator::~Animator()
	{
	}

	void Animator::Update(float speedAnimation)
	{
		if (m_activeAnimation)
		{
			if (m_onTransition)
			{
				TransitionBetweenAnimation();
			}
			else
			{
				RunAnimation(speedAnimation);
			}
		}

	}

	Anim* Animator::GetCurrentAnimation()
	{
		return &animations[m_currentAnimationIndex];
	}

	void Animator::GetAnimationTransform(int boneIndex,Vector3D& position,Quaternion& rotation,Anim* animation)
	{
		int keyFrame = animation->GetKeyFrame();
		float t = animation->animationTimer / animSpeed;
		int nextKeyFrame = (keyFrame + 1) > animation->GetMaxKeyFrame() ? 0 : keyFrame + 1;
		position = Lerp<Vector3D>(animation->GetPosition(keyFrame,boneIndex),animation->GetPosition(nextKeyFrame,boneIndex),t);
		rotation = Slerp(animation->GetRotation(keyFrame,boneIndex),animation->GetRotation(nextKeyFrame,boneIndex),t);
	}

	Matrix4D Animator::GetAnimationMatrix(int boneIndex) // ----- Transformer ----------
	{
		Vector3D currentPosition;
		Quaternion currentRotation;
		GetAnimationTransform(boneIndex,currentPosition,currentRotation,GetCurrentAnimation());


		if (!m_onTransition) return Matrix4D::CreateTransformMatrix(currentPosition,currentRotation);

		Vector3D finalPosition;
		Quaternion finalRotation;

		Vector3D nextPosition;
		Quaternion nextRotation;
		GetAnimationTransform(boneIndex,nextPosition,nextRotation,m_nextAnim);

		finalPosition = Lerp<Vector3D>(currentPosition,nextPosition,m_transitionCountdown / m_transitionTime);
		finalRotation = Slerp(currentRotation,nextRotation,m_transitionCountdown / m_transitionTime);

		return Matrix4D::CreateTransformMatrix(finalPosition,finalRotation);

	}

	void Animator::StartTransition(int nextAnimationIndex,float transitionTime)
	{
		if (nextAnimationIndex == m_currentAnimationIndex)
		{
			std::cout << "This is the already the current animation" << std::endl;
			return;
		}
		m_transitionTime = transitionTime;
		if (animations.size() > nextAnimationIndex)
		{
			m_nextAnim = &animations[nextAnimationIndex];
			m_nextAnimationIndex = nextAnimationIndex;
		}
		else
		{
			std::cout << "This animation isn't contained in this animator" << std::endl;
		}
	}

	void Animator::AddAnimation(const char* name)
	{
		animations.push_back(Anim(name,m_bonesCount));
		m_currentAnim = &animations[0];
		m_currentAnimationIndex = 0;
	}
	void Animator::RunAnimation(float speedAnimation)
	{
		m_currentAnim->animationTimer += speedAnimation * Tools::Time::GetDeltaTime();
		if (m_currentAnim->animationTimer >= animSpeed)
		{
			Anim* anim = GetCurrentAnimation();
			anim->ChangeKeyFrame();
			if (anim->GetKeyFrame() == 0 && m_nextAnim) m_onTransition = true;
			m_currentAnim->animationTimer = 0;
		}
	}
	void Animator::TransitionBetweenAnimation()
	{
		double multiplier = ((double)((float)m_nextAnim->GetMaxKeyFrame())) / (double)m_currentAnim->GetMaxKeyFrame();

		double timeScale = Lerp<double>(multiplier,1.0,m_transitionCountdown / m_transitionTime);
		double timeScale2 = Lerp<double>(1.0,1.0 / multiplier,m_transitionCountdown / m_transitionTime);

		GetCurrentAnimation()->UpdateAnimationTime(timeScale2,animSpeed);
		m_nextAnim->UpdateAnimationTime(timeScale,animSpeed);

		m_transitionCountdown += Tools::Time::GetDeltaTime();

		if (m_transitionCountdown < m_transitionTime) return;

		FinishTransition();
	}
	void Animator::FinishTransition()
	{
		m_currentAnim = m_nextAnim;
		m_nextAnim = nullptr;
		m_onTransition = false;
		m_currentAnimationIndex = m_nextAnimationIndex;
		m_transitionCountdown = 0;
	}
	void Animator::PlayOneFrame(bool forward)
	{
		Anim* anim = GetCurrentAnimation();
		anim->ChangeKeyFrame(forward);
		if (anim->GetKeyFrame() == 0 && m_nextAnim) m_onTransition = true;
		m_currentAnim->animationTimer = 0;
	}
}