#include "Bone.h"

namespace Animation
{
	Bone::Bone()
	{
		m_parent = nullptr;
	}


	Bone::Bone(const int boneIndex, const char* name, Bone* parent) :
		m_index(boneIndex), m_name(name), m_parent(parent)
	{
	}

	Bone::~Bone()
	{
	}

	int Bone::GetBoneIndex()
	{
		return m_index;
	}
	std::string Bone::GetBoneName()
	{
		return m_name;
	}
	Bone* Bone::GetBoneParent()
	{
		return m_parent;
	}

	Matrix4D Bone::GetTranformMatrix()
	{
		return Matrix4D::CreateTransformMatrix(localTransform);
	}

	void Bone::AddChild(Bone* bone)
	{
		children.push_back(bone);
	}

	void Bone::InitTPoseMatrix()
	{
		localMatrix = Matrix4D::CreateTransformMatrix(localTransform);
		// Multiply Local Matrix by parent world Matrix
		if (m_parent != nullptr)
		{
			modelMatrix = m_parent->modelMatrix * localMatrix;
		}
		else
		{
			modelMatrix = localMatrix;
		}

		// Set World matrix
		worldTransform = Matrix4D::GetTransformFromMatrix(modelMatrix, worldTransform);
		invModelMatrix = Matrix4D::InverseMatrix(modelMatrix);

		// Boucle to call all childrem;
		for (int i = 0; i < (int)children.size(); i++)
		{
			children[i]->InitTPoseMatrix();
		}
	}
}
