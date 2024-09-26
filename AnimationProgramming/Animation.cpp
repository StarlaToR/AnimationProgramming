#include "Bone.h"
#include "Animation.h"
#include "EngineInterface.h"
#include "Time.h"

namespace Animation
{

	Anim::Anim()
	{
		animationTimer = 0;
		std::cout << "Creation of a anim" << std::endl;
	}

	Anim::Anim(const char* animName, int boneCount)
	{
		std::cout << "Creation of a anim" << std::endl;
		m_name = animName;
		m_boneCount = boneCount;
		std::cout << "Use the anim file "<< m_name << " and the boneCount is "<< m_boneCount << '\n' << std::endl;
		
		InitAnimation();
	}

	Anim::~Anim()
	{
	}

	Vector3D Anim::GetPosition(int keyFrame, int boneIndex)
	{
		return m_position[keyFrame * m_boneCount + boneIndex];
	}

	Quaternion Anim::GetRotation(int keyFrame, int boneIndex)
	{
		return m_rotation[keyFrame * m_boneCount + boneIndex];
	}

	void Anim::ChangeKeyFrame(bool forward)
	{
		if (forward) m_currentKeyFrame++;
		else m_currentKeyFrame--;

		if (m_currentKeyFrame > m_keyFrameCount - 1)
		{
			m_currentKeyFrame = 0;
		}
		if (m_currentKeyFrame < 0)
		{
			m_currentKeyFrame = m_keyFrameCount - 1;
		}
	}

	void Anim::UpdateAnimationTime(double multiplier, float animSpeed)
	{
		animationTimer += multiplier *Tools::Time::GetDeltaTime();
		if (animationTimer > animSpeed)
		{
			ChangeKeyFrame();
			animationTimer = 0;
		}
	}

	void Anim::InitAnimation()
	{
		m_keyFrameCount = GetAnimKeyCount(m_name.c_str());
		std::cout << "Use the anim file contains " << m_keyFrameCount << " keyframes" << std::endl;

		for (int j = 0; j < m_keyFrameCount; j++)
		{
			for (int i = 0; i < m_boneCount; i++)
			{
				m_position.push_back(Vector3D::zero);
				m_rotation.push_back(Quaternion());
				GetAnimLocalBoneTransform(m_name.c_str(), i, j, m_position[i + m_boneCount * j], m_rotation[i + j * m_boneCount]);
			}
		}
	}
}