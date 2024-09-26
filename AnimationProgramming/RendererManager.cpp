#include "EngineInterface.h"
#include "RendererManager.h"
#include "Time.h"
#include "imgui.h"

RendererManager::RendererManager()
{
	Animation::Skeleton skeleton1(Vector3D(150,0,0),Color(1,0,0));
	Animation::Skeleton skeleton2(Vector3D(450,0,0),Color(0,0,1));
	Animation::Skeleton skeleton3(Vector3D(300,0,0),Color(0,1,0));

	skeletons.push_back(skeleton1);
	skeletons.push_back(skeleton2);
	skeletons.push_back(skeleton3);
}

RendererManager::~RendererManager()
{
}

void RendererManager::Init()
{
	skeletons[0].SetAnim("ThirdPersonWalk.anim");
	skeletons[2].SetAnim("ThirdPersonRun.anim");

	skeletons[1].isDrawing = true;
	skeletons[1].SetAnim("ThirdPersonWalk.anim");
	skeletons[1].SetAnim("ThirdPersonRun.anim");

	std::vector <Animation::Skeleton>::iterator iterator = skeletons.begin();
	for (iterator; iterator != skeletons.end(); iterator++)
	{
		iterator->Init();
	}

	ImGui::SetNextWindowSize({ 3000.0f,300.0f });
}



void RendererManager::Update()
{
	std::vector < Animation::Skeleton>::iterator iterator = skeletons.begin();
	for (iterator; iterator != skeletons.end(); iterator++)
	{
		iterator->Update(m_animationSpeed);
	}

	UiInterface.Update(skeletons[1],m_animationSpeed); 
}

void RendererManager::Draw()
{
	DrawWorldAxis();
	std::vector <Animation::Skeleton>::iterator iterator = skeletons.begin();
	for (iterator; iterator != skeletons.end(); iterator++)
	{
		iterator->Draw();
	}
}

void RendererManager::DrawWorldAxis()
{
	DrawLine(Vector3D(),Vector3D(100,0,0),Color(1,0,0));
	DrawLine(Vector3D(),Vector3D(0,100,0),Color(0,1,0));
	DrawLine(Vector3D(),Vector3D(0,0,100),Color(0,0,1));
}