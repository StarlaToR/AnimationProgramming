#include "UI.h"
#include "imgui.h"


UI::UI()
{
}

UI::~UI()
{
}

void UI::Update(Animation::Skeleton& skeleton,float& animationSpeed)
{
	ImGui::Begin("Animation Timeline");
	Animation::Animator animator = skeleton.GetAnimator();
	Animation::Anim* anim = animator.GetCurrentAnimation();
	int currentAnim = anim->GetKeyFrame();
	ImGui::SliderInt("Animation frame",&currentAnim,0,anim->GetMaxKeyFrame());
	ImGui::SliderFloat("Animation Speed",&animationSpeed,0.f,2.f);

	if (ImGui::Button("Play",ImVec2(125,25))) skeleton.PlayAnimation(true);
	ImGui::SameLine();
	if (ImGui::Button("Pause",ImVec2(125,25))) skeleton.PlayAnimation(false);

	ImGui::NewLine();
	if (ImGui::Button("Frame Backward",ImVec2(125,25))) skeleton.FrameForward(false);
	ImGui::SameLine();
	if (ImGui::Button("Frame Upward",ImVec2(125,25))) skeleton.FrameForward(true);

	ImGui::NewLine();
	if (ImGui::Button("Run Animation",ImVec2(125,25))) skeleton.ChangeAnim(1);
	ImGui::SameLine();
	if (ImGui::Button("Walk Animation ",ImVec2(125,25))) skeleton.ChangeAnim(0);


	ImGui::End();
}
