// AnimationProgramming.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <chrono>
#include <thread>

#include "mathLibrary.hpp"
#include "Animation.h"
#include "EngineInterface.h"
#include "RendererManager.h"
#include "Skeleton.h"
#include "Time.h"
#include "imgui.h"
#include "Simulation.h"


#define MAX_BONES 64

class CSimulation : public ISimulation
{
	RendererManager m_rendererManager;
	Tools::Time time;

	virtual void Init() override
	{
		ImGui::SetCurrentContext((ImGuiContext*)GetImGUIContext());
		time.Init();
		m_rendererManager = RendererManager();
		m_rendererManager.Init();
	}

	virtual void Update(float frameTime) override
	{
		
		time.Update();
		m_rendererManager.Update();
		m_rendererManager.Draw();
	}
};



int main()
{
	CSimulation simulation;
	Run(&simulation, 1400, 800);
	return 0;
}

