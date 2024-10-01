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

vec3 cameraPos(0, 0, -3.001);
mat3 cameraRot(1, 0, 0, 0, 1, 0, 0, 0, 1);
const float speed = 0.01;

vec3 currentColor;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

// ----------------------------------------------------------------------------
// FUNCTIONS
struct Pixel
{
  int x;
  int y;
  float zinv;
};
void Update(void);
void Draw(void);
void VertexShader(const vec3 &v, Pixel &p);
void Interpolate(Pixel a, Pixel b, vector<Pixel> &result);
void DrawLineSDL(ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3> &vertices);
void ComputePolygonRows(const vector<Pixel> &vertexPixels, vector<Pixel> &leftPixels, vector<Pixel> &rightPixels);
void DrawPolygonRows(const vector<Pixel> &leftPixels, const vector<Pixel> &rightPixels);
void DrawPolygon(const vector<vec3> &vertices);
void Interpolate(Pixel a, Pixel b, vector<Pixel> &result);

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
  cout << "Render time: " << dt << " ms." << endl;

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
  for (int y = 0; y < SCREEN_HEIGHT; ++y)
    for (int x = 0; x < SCREEN_WIDTH; ++x)
      depthBuffer[y][x] = 0;

  for (int i = 0; i < triangles.size(); ++i)
  {
    vector<vec3> vertices(3);
    currentColor = triangles[i].color;
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;
    DrawPolygon(vertices);
  }

  sdlAux->render();
}

void VertexShader(const vec3 &v, Pixel &p)
{
  vec3 pprim = (v - cameraPos) * cameraRot;
  p.zinv = 1 / pprim.z;
  p.x = (FOCAL_LENGTH * pprim.x / pprim.z) + (SCREEN_WIDTH / 2.0f);
  p.y = (FOCAL_LENGTH * pprim.y / pprim.z) + (SCREEN_HEIGHT / 2.0f);
}

void Interpolate(Pixel a, Pixel b, vector<Pixel> &result)
{
  int N = result.size();
  vec2 step = vec2(b.x - a.x, b.y - a.y) / float(max(N - 1, 1));
  float zStep = (b.zinv - a.zinv) / float(max(N - 1, 1));
  vec2 current(a.x, a.y);
  float currentZ = a.zinv;
  for (int i = 0; i < N; ++i)
  {
    result[i].x = current.x + 0.5f;
    result[i].y = current.y + 0.5f;
    result[i].zinv = currentZ;
    current += step;
    currentZ += zStep;
  }
}

void DrawLineSDL(Pixel a, Pixel b, vec3 color)
{
  Pixel delta;
  delta.x = glm::abs(a.x - b.x);
  delta.y = glm::abs(a.y - b.y);
  delta.zinv = glm::abs(a.zinv - b.zinv);

  int pixels = glm::max(delta.x, delta.y) + 1;
  vector<Pixel> line(pixels);
  Interpolate(a, b, line);
  for (int i = 0; i < line.size(); ++i)
  {
    if (line[i].zinv > depthBuffer[line[i].y][line[i].x])
    {
      depthBuffer[line[i].y][line[i].x] = line[i].zinv;
      sdlAux->putPixel(line[i].x, line[i].y, color);
    }
  }
}

void DrawPolygonEdges(const vector<vec3> &vertices)
{
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<Pixel> projectedVertices(V);
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

void ComputePolygonRows(const vector<Pixel> &vertexPixels, vector<Pixel> &leftPixels, vector<Pixel> &rightPixels)
{
  // 1. Find max and min y−value of the polygon and compute the number of rows it occupies.
  int minY = numeric_limits<int>::max();
  int maxY = numeric_limits<int>::min();
  for (int i = 0; i < vertexPixels.size(); ++i)
  {
    minY = std::min(minY, vertexPixels[i].y);
    maxY = std::max(maxY, vertexPixels[i].y);
  }
  int numRows = maxY - minY + 1;

  // 2. Resize leftPixels and rightPixels so that they have an element for each row.
  leftPixels.resize(numRows);
  rightPixels.resize(numRows);

  // 3. Initialize the x−coordinates in leftPixels to some really large value and the x−coordinates in rightPixels to some really small value.
  for (int i = 0; i < numRows; ++i) // Change condition from i <= numRows to i < numRows
  {
    leftPixels[i].x = numeric_limits<int>::max();
    leftPixels[i].y = minY + i;
    rightPixels[i].x = numeric_limits<int>::min();
    rightPixels[i].y = minY + i;
  }

  // 4. Loop through all edges of the polygon and use linear interpolation to find the x−coordinate for each row it occupies. Update the corresponding values in rightPixels and leftPixels .
  for (int i = 0; i < vertexPixels.size(); ++i)
  {
    int j = (i + 1) % vertexPixels.size();
    Pixel delta;
    delta.x = glm::abs(vertexPixels[i].x - vertexPixels[j].x);
    delta.y = glm::abs(vertexPixels[i].y - vertexPixels[j].y);

    int pixels = glm::max(delta.x, delta.y) + 1;
    vector<Pixel> line(pixels);
    Interpolate(vertexPixels[i], vertexPixels[j], line);

    for (Pixel pixel : line)
    {
      if (pixel.x < leftPixels[pixel.y - minY].x)
      {
        leftPixels[pixel.y - minY].x = pixel.x;
        leftPixels[pixel.y - minY].zinv = pixel.zinv;
      }
      if (pixel.x > rightPixels[pixel.y - minY].x)
      {
        rightPixels[pixel.y - minY].x = pixel.x;
        rightPixels[pixel.y - minY].zinv = pixel.zinv;
      }
    }
  }
}

void DrawPolygonRows(const vector<Pixel> &leftPixels, const vector<Pixel> &rightPixels)
{
  for (int i = 0; i < leftPixels.size(); ++i)
  {
    DrawLineSDL(leftPixels[i], rightPixels[i], currentColor);
  }
}

void DrawPolygon(const vector<vec3> &vertices)
{
  int V = vertices.size();
  vector<Pixel> vertexPixels(V);

  for (int i = 0; i < V; ++i)
  {
    VertexShader(vertices[i], vertexPixels[i]);
  }
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
  DrawPolygonRows(leftPixels, rightPixels);
}
