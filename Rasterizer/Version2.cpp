// DH2323 skeleton code, Lab3 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
#include "SDL2auxiliary.h"
#include "TestModel.h"
#include <algorithm> //for max()

using namespace std;
using glm::ivec2;
using glm::mat3;
using glm::vec2;
using glm::vec3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
float FOCAL_LENGTH = SCREEN_HEIGHT / 1;
SDL2Aux *sdlAux;
int t;
vector<Triangle> triangles;

vec3 cameraPos(0, 0, -3.1);
mat3 cameraRot(1, 0, 0, 0, 1, 0, 0, 0, 1);
const float speed = 0.01;

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);
void VertexShader(const vec3 &v, ivec2 &p);
void Interpolate(ivec2 a, ivec2 b, vector<ivec2> &result);
void DrawLineSDL(ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3> &vertices);

int main(int argc, char *argv[])
{
	LoadTestModel(triangles); // Load model
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks(); // Set start value for timer.

	while (!sdlAux->quitEvent())
	{
		Update();
		Draw();
	}
	sdlAux->saveBMP("screenshot.bmp");
	return 0;
}

void Update(void)
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;
	// cout << "Render time: " << dt << " ms." << endl;

	const Uint8 *keystate = SDL_GetKeyboardState(NULL);
	if (keystate[SDL_SCANCODE_UP])
		cameraPos.z += speed; // Move camera forward
	if (keystate[SDL_SCANCODE_DOWN])
		cameraPos.z -= speed; // Move camera backward
	if (keystate[SDL_SCANCODE_LEFT])
		cameraPos.x -= speed; // Move camera left
	if (keystate[SDL_SCANCODE_RIGHT])
		cameraPos.x += speed; // Move camera right

	// Rotate camera up
	if (keystate[SDL_SCANCODE_W])
		cameraRot = mat3(1, 0, 0, 0, cos(speed), -sin(speed), 0, sin(speed), cos(speed)) * cameraRot;
	// Rotate camera down
	if (keystate[SDL_SCANCODE_S])
		cameraRot = mat3(1, 0, 0, 0, cos(-speed), -sin(-speed), 0, sin(-speed), cos(-speed)) * cameraRot;
	// Rotate camera left
	if (keystate[SDL_SCANCODE_A])
		cameraRot = mat3(cos(speed), 0, sin(speed), 0, 1, 0, -sin(speed), 0, cos(speed)) * cameraRot;
	// Rotate camera right
	if (keystate[SDL_SCANCODE_D])
		cameraRot = mat3(cos(-speed), 0, sin(-speed), 0, 1, 0, -sin(-speed), 0, cos(-speed)) * cameraRot;
	// Rotate camera clockwise
	if (keystate[SDL_SCANCODE_Q])
		cameraRot = mat3(cos(speed), -sin(speed), 0, sin(speed), cos(speed), 0, 0, 0, 1) * cameraRot;
	// Rotate camera counter-clockwise
	if (keystate[SDL_SCANCODE_E])
		cameraRot = mat3(cos(-speed), -sin(-speed), 0, sin(-speed), cos(-speed), 0, 0, 0, 1) * cameraRot;
}

void Draw()
{
	sdlAux->clearPixels();

	for (int i = 0; i < triangles.size(); ++i)
	{
		vector<vec3> vertices(3);
		vertices[0] = triangles[i].v0;
		vertices[1] = triangles[i].v1;
		vertices[2] = triangles[i].v2;
		DrawPolygonEdges(vertices);
	}

	sdlAux->render();
}

void VertexShader(const vec3 &v, ivec2 &p)
{
	vec3 pprim = (v - cameraPos) * cameraRot;
	p.x = (FOCAL_LENGTH * pprim.x / pprim.z) + (SCREEN_WIDTH / 2.0f);
	p.y = (FOCAL_LENGTH * pprim.y / pprim.z) + (SCREEN_HEIGHT / 2.0f);
}

void Interpolate(ivec2 a, ivec2 b, vector<ivec2> &result)
{
	int N = result.size();
	vec2 step = vec2(b - a) / float(max(N - 1, 1));
	vec2 current(a);
	for (int i = 0; i < N; ++i)
	{
		result[i] = current;
		current += step;
	}
}

void DrawLineSDL(ivec2 a, ivec2 b, vec3 color)
{
	ivec2 delta = glm::abs(a - b);
	int pixels = glm::max(delta.x, delta.y) + 1;
	vector<ivec2> line(pixels);
	Interpolate(a, b, line);
	for (int i = 0; i < line.size(); ++i)
	{
		sdlAux->putPixel(line[i].x, line[i].y, color);
	}
}

void DrawPolygonEdges(const vector<vec3> &vertices)
{
	int V = vertices.size();
	// Transform each vertex from 3D world position to 2D image position:
	vector<ivec2> projectedVertices(V);
	for (int i = 0; i < V; ++i)
	{
		VertexShader(vertices[i], projectedVertices[i]);
	}
	// Loop over all vertices and draw the edge from it to the next vertex:
	for (int i = 0; i < V; ++i)
	{
		int j = (i + 1) % V; // The next vertex
		vec3 color(1, 1, 1);
		DrawLineSDL(projectedVertices[i], projectedVertices[j], color);
	}
}
