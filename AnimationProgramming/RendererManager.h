#pragma once
#include <vector>
#include "Skeleton.h"
#include "UI.h"

class RendererManager
{
public:
	RendererManager();
	~RendererManager();
	void Init();
	void Update();
	void Draw();

	std::vector<Animation::Skeleton> skeletons;
	UI UiInterface;
private:
	void DrawWorldAxis();
	float m_animationSpeed = 1.0f;

};

