#include "AnimationGeneralData.h"

#include "Skeleton.h"

namespace Animation
{
	Skeleton::Skeleton(Vector3D pos, Vector4D col)
	{
		m_offsetBodyPosition = pos;
		m_boneColor = col;
	}

	Skeleton::~Skeleton()
	{

	}

	void Skeleton::Draw()
	{
		for (size_t j = 2; j < m_boneCount; j++)
		{
			DrawLine(m_bones[j].worldTransform.position + m_offsetBodyPosition, m_bones[j].GetBoneParent()->worldTransform.position + m_offsetBodyPosition, m_boneColor);
		}
	}

	void Skeleton::Init()
	{
		Vector3D vector;
		Quaternion quat;
		
		m_boneCount = GetSkeletonBoneCount();
		m_boneCount = ClampInt(m_boneCount,0,MAX_BONES);

		m_bones = std::vector<Bone>(m_boneCount);
		std::vector<Bone>::iterator iter = m_bones.begin();
		int i = 0;
		for (iter ; iter!= m_bones.end(); iter++,i++)
		{
			GetSkeletonBoneLocalBindTransform(i, vector, quat);
			*iter = Bone(i, GetSkeletonBoneName(i));
			iter->localTransform = Physics::Transform(vector, quat);

			int indexParent = GetSkeletonBoneParentIndex(i);
			if (indexParent == -1) continue;

			iter->SetBoneParent(&m_bones[indexParent]);
			iter->GetBoneParent()->AddChild(&m_bones[i]);
		}

		m_bones[0].InitTPoseMatrix();
	}

	void Skeleton::SetAnim(const char* name)
	{
		m_animator.AddAnimation(name);
	}

	void Skeleton::ChangeAnim(int index)
	{
		m_animator.StartTransition(index, 1.f);
	}

	void Skeleton::PlayAnimation(bool active)
	{
		m_animator.PlayAnimation(active);
	}

	void Skeleton::FrameForward(bool forward)
	{
		m_animator.PlayOneFrame(forward);
	}

	void Skeleton::Update(float speedAnimation)
	{

		m_animator.Update(speedAnimation);
		std::vector<Matrix4D> matrices(m_boneCount);
		CalculateBonesPositions(matrices);
		if(isDrawing) SetSkinningPose((float*)matrices.data(),m_boneCount);
	}

	void Skeleton::CalculateBonesPositions( std::vector<Matrix4D>& outPose)
	{
		std::vector<Matrix4D> matrices;
		std::vector<Matrix4D> modelMatrices;
		matrices.resize(m_boneCount);
		modelMatrices.resize(m_boneCount);

		for (size_t i = 0; i < matrices.size(); i++)
		{
		
			matrices[i] = m_bones[i].localMatrix * m_animator.GetAnimationMatrix(i);
		}

		modelMatrices[0] = matrices[0];
		
		for (size_t i = 1; i < matrices.size(); i++)
		{
			int indexParent = m_bones[i].GetBoneParent()->GetBoneIndex();
			modelMatrices[i] = modelMatrices[indexParent] * matrices[i];
		}

		for (size_t i = 0; i < matrices.size(); i++)
		{
			outPose [i]= modelMatrices[i] * m_bones[i].invModelMatrix;
			m_bones[i].worldTransform = Matrix4D::GetTransformFromMatrix(modelMatrices[i], m_bones[i].worldTransform);
		}
	}

}