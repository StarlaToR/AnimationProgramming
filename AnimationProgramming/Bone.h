#pragma once
#include <vector>

#include "mathLibrary.hpp"
namespace Animation
{

	class Bone
	{
	public:
		Bone();
		Bone(const int boneIndex,const char* name,Bone* parent = nullptr);
		~Bone();

		Physics::Transform localTransform;
		Matrix4D localMatrix;
		Matrix4D invModelMatrix;

		Physics::Transform worldTransform;
		Matrix4D modelMatrix;

		int GetBoneIndex();
		std::string GetBoneName();
		Bone* GetBoneParent();
		void SetBoneParent(Bone* parent) { m_parent = parent; };
		Matrix4D GetTranformMatrix();
		void AddChild(Bone* bone);
		void InitTPoseMatrix();

	private:
		Bone* m_parent;
		std::vector<Bone*> children;
		int m_index = -1;
		std::string	m_name;

	};
}
