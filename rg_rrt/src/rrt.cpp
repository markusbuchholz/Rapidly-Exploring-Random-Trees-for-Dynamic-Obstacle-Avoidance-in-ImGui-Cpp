// Markus Buchholz, 2023

#include <stdafx.hpp>
#include "imgui_helper.hpp"
#include <tuple>
#include <vector>
#include <math.h>
#include <random>
#include <algorithm>
#include <thread>
#include <chrono>
#include <list>
#include <tuple>

//---------------------------------------------------------------

int W = 600;
int H = 600;
int mesh_size = 60;
int step = 10;
//---------------------------------------------------------------

float MAX_ITERATION = 2000;
float MAX_DISTANCE = 10;
float GOAL;

//---------------------------------------------------------------

struct Node
{

	float posX;
	float posY;
	Node *prev;
	Node *next;
};

//------------------------------------------------------------------------------

class RRT
{

private:
	Node *start;
	Node *goal;
	std::vector<Node *> rrtNodes;
	std::vector<float> tempX;
	std::vector<float> tempY;

public:
	RRT(float startX, float startY, float goalX, float goalY)
	{

		Node *node = new Node;
		this->start = node;
		start->posX = startX;
		start->posY = startY;
		start->prev = nullptr;
		rrtNodes.push_back(node);

		node = new Node;
		this->goal = node;
		goal->posX = goalX;
		goal->posY = goalY;
	}

	//------------------------------------------------------------------------------

	bool checkObstacles(float x1, float y1, float x2, float y2, std::tuple<std::vector<float>, std::vector<float>> obs)
	{
		bool decision;

		std::vector<float> minX = std::get<0>(obs);
		std::vector<float> minY = std::get<1>(obs);

		for (int i = 0; i < minX.size(); i++)
		{

			// Completely outside.
			if ((x1 <= minX[i] && x2 <= minX[i]) || (y1 <= minY[i] && y2 <= minY[i]))
				decision = false;
		}
		for (int i = 0; i < minX.size(); i++)
		{

			float m = (y2 - y1) / (x2 - x1);

			float y = m * (minX[i] - x1) + y1;
			if (y > minY[i])
				decision = true;

			float x = (minY[i] - y1) / m + x1;
			if (x > minX[i])
				decision = true;
		}
		return decision;
	}

	//------------------------------------------------------------------------------

	Node *checkNearestNode(Node *new_node, float x0, float y0, float x1, float y1)
	{

		// std::cout << __FUNCTION__ << "\n";

		std::vector<float> obsXmin{x0, x1};
		std::vector<float> obsYmin{y0, y1};

		Node *near_node = new Node;
		float minDistance = std::numeric_limits<float>::max();
		float corrX = 0.0;
		float corrY = 0.0;
		bool check_obstacle;

		for (auto &ii : rrtNodes)
		{

			float distance = std::sqrt(std::pow((new_node->posX - ii->posX), 2) + std::pow((new_node->posY - ii->posY), 2));
			if (distance < minDistance)
			{

				minDistance = distance;
				near_node = ii;
			}
		}

		float dx = new_node->posX - near_node->posX;
		float dy = new_node->posY - near_node->posY;
		float angle = std::atan2(dy, dx) * 180 / M_PI;

		if (minDistance > MAX_DISTANCE)
		{

			corrX = std::abs(near_node->posX + std::cos(angle) * MAX_DISTANCE);
			corrY = std::abs(near_node->posY + std::sin(angle) * MAX_DISTANCE);
		}

		if (minDistance <= MAX_DISTANCE)
		{

			corrX = new_node->posX;
			corrY = new_node->posY;
		}
		if (rrtNodes.size() > 0)
		{
			check_obstacle = checkObstacles(near_node->posX, near_node->posY, corrX, corrY, std::make_tuple(obsXmin, obsYmin));
		}

		new_node->posX = corrX;
		new_node->posY = corrY;

		near_node->next = new_node;
		new_node->prev = near_node;

		if (rrtNodes.size() == 0)
		{

			new_node->prev = start;
		}

		if (check_obstacle == 0)
		{
			rrtNodes.push_back(new_node);
		}

		if (((float)new_node->posX == (float)this->goal->posX) && ((float)new_node->posY == (float)this->goal->posY))
		{

			std::cout << "The GOAL achive && GOLD path is ..." << std::endl;

			GOAL = 1;

			while (new_node->prev != nullptr)

			{
				new_node = new_node->prev;
				tempX.push_back(new_node->posX);
				tempY.push_back(new_node->posY);
			}
			return 0;
		}

		return new_node;
	}

	//------------------------------------------------------------------------------

	std::tuple<std::vector<float>, std::vector<float>> lookForPath(float x0, float y0, float x1, float y1)
	{

		std::random_device dev;
		std::mt19937 rng(dev());
		std::uniform_int_distribution<std::mt19937::result_type> dist100(0, 79);

		while (GOAL != 1)
		{
			Node *random_node = new Node;
			Node *last_node = new Node;

			float randX = dist100(rng);
			float randY = dist100(rng);
			random_node->posX = randX;
			random_node->posY = randY;

			last_node = checkNearestNode(random_node, x0, y0, x1, y1);

			if (GOAL == 1)
			{
				goal->prev = last_node;
				std::cout << "goal"
						  << "\n";
				GOAL = 0;
				return std::make_tuple(tempX, tempY);
			}
		}
		return std::make_tuple(tempX, tempY);
	}

	//------------------------------------------------------------------------------

	void printRRT(RRT &rrt)
	{

		Node *node = rrt.start;
		node = rrt.goal;
	}
};

//------------------------------------------------------------------------------

float randomPos()
{

	std::random_device device;
	std::mt19937 engine(device());
	std::uniform_real_distribution<float> dist(0.0, W);

	return dist(engine);
}

//---------------------------------------------------------------

std::tuple<float, float> CheckPosM(float a, float b)
{

	float xi;
	float yi;

	if (a > W || a < 0)
	{
		xi = 250.0f;
	}
	else
	{
		xi = a;
	}
	if (b > H || b < 0)
	{
		yi = 250.0f;
	}
	else
	{
		yi = b;
	}

	return std::make_tuple(xi, yi);
}

//---------------------------------------------------------------
struct Obstacle
{

	float x;
	float y;
};
//---------------------------------------------------------------

int main(int argc, char const *argv[])
{

	std::string title = "Rapid Exploring Random Trees";
	initImgui(W, H, title);

	bool show_demo_window = true;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.0f / 255.0, 0.0f / 255.0, 0.0f / 255.0, 1.00f);
	ImVec4 white_color = ImVec4(255.0f / 255.0, 255.0f / 255.0, 255.0f / 255.0, 1.00f);
	ImVec4 blue_color = ImVec4(0.0f / 255.0, 0.0f / 255.0, 170.0f / 255.0, 1.00f);
	ImVec4 pink_color = ImVec4(179.0f / 255.0, 12.0f / 255.0, 130.0f / 255.0, 1.00f);
	ImVec4 gray_color = ImVec4(150.0f / 255.0, 10.0f / 160.0, 170.0f / 255.0, 1.00f);
	ImVec4 black_color = ImVec4(0.0f / 255.0, 0.0f / 255.0, 0.0f / 255.0, 1.00f);

	bool flag = true;

	float dynamic_x0 = 550.0f;
	float dynamic_y0 = 200.0f;
	float dynamic_x1 = 650.0f;
	float dynamic_y1 = 300.0f;

	// float dynamic_x0 = 200.0f;
	// float dynamic_y0 = 100.0f;
	// float dynamic_x1 = 300.0f;
	// float dynamic_y1 = 200.0f;

	while (!glfwWindowShouldClose(window) && flag == true)
	{

		RRT *rrt = new RRT(1, 50, 60, 10);
		glfwPollEvents();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();

		ImGui::NewFrame();

		ImGuiIO io = ImGui::GetIO();
		ImVec2 mouse_posIMgui = io.MousePos;
		Obstacle obstacle_pos{mouse_posIMgui.x, mouse_posIMgui.y};

		auto posMouse = CheckPosM(obstacle_pos.x, obstacle_pos.y);

		dynamic_x0 -= 10.0f;
		dynamic_x1 -= 10.0f;

		// dynamic_y0 +=10.0f;
		// dynamic_y1 +=10.0f;

		ImGuiWindowFlags window_flags = 0;
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(W, H), ImGuiCond_FirstUseEver);
		window_flags |= ImGuiWindowFlags_NoMove;
		window_flags |= ImGuiWindowFlags_NoResize;
		window_flags |= ImGuiWindowFlags_NoCollapse;
		window_flags |= ImGuiWindowFlags_NoBackground;
		window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;

		ImGui::Begin("Simulation", nullptr, window_flags);
		ImDrawList *draw_list = ImGui::GetWindowDrawList();

		auto checkpath = rrt->lookForPath(dynamic_x0 / step, dynamic_y0 / step, dynamic_x1 / step, dynamic_y1 / step);

		int len = std::get<0>(checkpath).size();

		draw_list->AddCircleFilled({1 * step, 50 * step}, 5.0f, ImColor(white_color));
		draw_list->AddCircleFilled({60 * step, 10 * step}, 5.0f, ImColor(white_color));

		for (int ii = 0; ii < len - 1; ii++)
		{

			float x0 = std::get<0>(checkpath)[ii] * step;
			float y0 = std::get<1>(checkpath)[ii] * step;
			float x1 = std::get<0>(checkpath)[ii + 1] * step;
			float y1 = std::get<1>(checkpath)[ii + 1] * step;

			draw_list->AddLine({x0, y0}, {x1, y1}, ImColor(white_color), 1.0f);
		}

		// draw mesh

		for (int ii = 0; ii < mesh_size; ii++)
		{
			for (int jj = 0; jj < mesh_size; jj++)
			{
				draw_list->AddCircleFilled({(float)ii * step, (float)jj * step}, 0.3f, ImColor(white_color));
			}
		}

		draw_list->AddRectFilled({dynamic_x0, dynamic_y0}, {dynamic_x1, dynamic_y1}, ImColor(pink_color));

		delete rrt;

		ImGui::End();

		// Rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	termImgui();
	return 0;
}
