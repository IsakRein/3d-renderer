// DH2323 skeleton code, Lab2 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
#include "SDL2auxiliary.h"
#include "TestModel.h"
#include <algorithm>
#include <limits>

using namespace std;
using glm::inverse;
using glm::isnan;
using glm::mat3;
using glm::vec3;

struct Intersection
{
	vec3 position;
	float distance;
	int triangleIndex;
};

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
int t;
vector<Triangle> triangles;
float focalLength = SCREEN_HEIGHT / 2;
vec3 cameraPos(0, 0, -2);

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);
bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle> &triangles, Intersection &closestIntersection);

int main(int argc, char *argv[])
{
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks(); // Set start value for timer.
	LoadTestModel(triangles);

	while (!sdlAux->quitEvent())
	{
		Update();
		Draw();
	}

	sdlAux->saveBMP("screenshot.bmp");
	return 0;
}

bool ClosestIntersection(vec3 start,
						 vec3 dir,
						 const vector<Triangle> &triangles,
						 Intersection &closestIntersection)
{
	float m = std::numeric_limits<float>::max();

	for (int i = 0; i < triangles.size(); i++)
	{
		const Triangle &triangle = triangles[i];

		vec3 v0 = triangle.v0;
		vec3 v1 = triangle.v1;
		vec3 v2 = triangle.v2;

		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		vec3 b = start - v0;
		mat3 A(-dir, e1, e2);
		vec3 x = glm::inverse(A) * b;

		float t = x.x;
		float u = x.y;
		float v = x.z;

		if (t > 0 && u >= 0 && v >= 0 && u + v <= 1)
		{
			if (t < m)
			{
				m = t;
				vec3 intersection = dir;
				intersection *= t;
				intersection += start;

				closestIntersection.position = intersection;
				closestIntersection.distance = t;
				closestIntersection.triangleIndex = i;
			}
		}
	}

	return (m != std::numeric_limits<float>::max());
}

void Update(void)
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;
}

void Draw()
{
	sdlAux->clearPixels();
	Intersection closestIntersection;

	for (int y = 0; y < SCREEN_HEIGHT; ++y)
	{
		for (int x = 0; x < SCREEN_WIDTH; ++x)
		{
			vec3 dir(x - SCREEN_WIDTH / 2, y - SCREEN_HEIGHT / 2, focalLength);
			if (ClosestIntersection(cameraPos, dir, triangles, closestIntersection))
				sdlAux->putPixel(x, y, triangles[closestIntersection.triangleIndex].color);
			else
				sdlAux->putPixel(x, y, vec3(0, 0, 0));
		}
	}
	sdlAux->render();
}