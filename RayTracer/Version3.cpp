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

#define PI 3.14159265359f

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
mat3 R;
float yaw = 0.0f;
vec3 lightPos(0, -0.5, -0.7);
vec3 lightColor = 14.f * vec3(1, 1, 1);
vec3 indirectLight = 0.5f * vec3(1, 1, 1);

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);
bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle> &triangles, Intersection &closestIntersection);
bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle> &triangles, Intersection &closestIntersection, int ignoreIndex);
vec3 DirectLight(const Intersection &i);

int main(int argc, char *argv[])
{
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks(); // Set start value for timer.
	LoadTestModel(triangles);

	Intersection closestIntersection;
	ClosestIntersection(cameraPos, vec3(0.0, 0.0, 1), triangles, closestIntersection);

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
	return ClosestIntersection(start, dir, triangles, closestIntersection, -1);
}

bool ClosestIntersection(vec3 start,
						 vec3 dir,
						 const vector<Triangle> &triangles,
						 Intersection &closestIntersection,
						 int ignoreIndex)
{
	float m = std::numeric_limits<float>::max();

	for (int i = 0; i < triangles.size(); i++)
	{
		if (i == ignoreIndex)
			continue;

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

vec3 DirectLight(const Intersection &i)
{
	vec3 r_hat = glm::normalize(lightPos - i.position);
	vec3 n_hat = glm::normalize(triangles[i.triangleIndex].normal);
	float r = glm::distance(lightPos, i.position);

	vec3 B = lightColor / (4.0f * PI * r * r);
	vec3 P = B * std::max(0.0f, glm::dot(n_hat, r_hat));

	// Check if there is an object between the intersection and the light source
	Intersection j;
	bool isBlocked = ClosestIntersection(i.position, r_hat, triangles, j, i.triangleIndex);

	if (isBlocked && glm::distance(j.position, i.position) < r)
	{
		return vec3(0, 0, 0);
	}

	return P;
}

void Update(void)
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;
	// cout << "Render time: " << dt << " ms." << endl;

	// Update camera position:
	const Uint8 *keystate = SDL_GetKeyboardState(NULL);

	// Move camera
	if (keystate[SDL_SCANCODE_UP])
		cameraPos.z += 0.1;
	if (keystate[SDL_SCANCODE_DOWN])
		cameraPos.z -= 0.1;
	if (keystate[SDL_SCANCODE_LEFT])
		yaw -= 0.1;
	if (keystate[SDL_SCANCODE_RIGHT])
		yaw += 0.1;

	// Move light
	if (keystate[SDL_SCANCODE_W])
		lightPos.z += 0.1;
	if (keystate[SDL_SCANCODE_S])
		lightPos.z -= 0.1;
	if (keystate[SDL_SCANCODE_A])
		lightPos.x -= 0.1;
	if (keystate[SDL_SCANCODE_D])
		lightPos.x += 0.1;
	if (keystate[SDL_SCANCODE_Q])
		lightPos.y -= 0.1;
	if (keystate[SDL_SCANCODE_E])
		lightPos.y += 0.1;

	R[0][0] = cos(yaw);
	R[0][2] = -sin(yaw);
	R[2][0] = sin(yaw);
	R[2][2] = cos(yaw);
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

			dir = R * dir;

			if (ClosestIntersection(cameraPos, dir, triangles, closestIntersection))
			{
				vec3 p = triangles[closestIntersection.triangleIndex].color;
				vec3 D = DirectLight(closestIntersection);
				vec3 N = indirectLight;
				sdlAux->putPixel(x, y, p * (D + N));
			}
			else
				sdlAux->putPixel(x, y, vec3(0, 0, 0));
		}
	}
	sdlAux->render();
}