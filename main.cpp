#define PARALLEL
#define ANTIALIASING

#ifndef VLA
#include <malloc.h>
#endif

#include <cassert>
#include <cmath>
#include <ctime>
#include <execution>
#include <fstream>
#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_stdinc.h>
#include <sstream>
#include <vector>

/*
// Excerpt from "Ray Tracing in One Weekend"
// with some of my own modifications.
// https://github.com/RayTracing/raytracing.github.io/blob/release/src/InOneWeekend/vec3.h
*/
#include "vec3.h" 

typedef struct color {
	Uint8 red;
	Uint8 green;
	Uint8 blue;
} color_t;

const color_t backgroundColor = { 0, 0, 0 };

const Uint8 MAX_COLOR = 255;
const double COLOR_COEFFICIENT = 1.0 / 255.0;

const double CLOCK_COEFFICIENT = 1.0 / CLOCKS_PER_SEC;

enum MATERIAL
{
	DIFFUSE = 0,
	MIRROR,
	GLASS
};

typedef struct sphere {
	MATERIAL materialType;
	double ior;
	double radius;
	vec3 center;
	vec3 albedo;

	sphere(MATERIAL matType, double refIdx, double r, vec3 centre, vec3 reflectance) :
		materialType(matType),
		ior(refIdx),
		radius(r),
		center(centre),
		albedo(reflectance)
	{}
} sphere_t;

typedef struct plane {
	MATERIAL materialType;
	double ior;
	vec3 point;
	vec3 normal;
	vec3 albedo;

	plane(MATERIAL matType, double refIdx, vec3 distance, vec3 surfaceNormal, vec3 reflectance) :
		materialType(matType),
		ior(refIdx),
		point(distance),
		normal(surfaceNormal),
		albedo(reflectance)
	{}
} plane_t;

typedef struct triangle {
	MATERIAL materialType;
	double ior;
	vec3 v0;
	vec3 v1;
	vec3 v2;
	vec3 albedo;

	triangle(MATERIAL matType, double refIdx, vec3 vert0, vec3 vert1, vec3 vert2, vec3 reflectance) :
		materialType(matType),
		ior(refIdx),
		v0(vert0),
		v1(vert1),
		v2(vert2),
		albedo(reflectance)
	{}
} triangle_t;

typedef struct triangle_mesh {
	MATERIAL materialType;
	double ior;
	unsigned int ev0;
	unsigned int ev1;
	unsigned int ev2;
	vec3 centroid;
	vec3 albedo;

	triangle_mesh(MATERIAL matType, double refIdx, unsigned int vElm0, unsigned int vElm1, unsigned int vElm2,
		vec3 reflectance, vec3 barycentre = vec3(1.0)) :
		materialType(matType),
		ior(refIdx),
		ev0(vElm0),
		ev1(vElm1),
		ev2(vElm2),
		albedo(reflectance),
		centroid(barycentre)
	{}
} triangle_mesh_t;

std::vector<vec3> meshVertex;
std::vector<vec3> meshVertexNormal;

std::vector<triangle_mesh_t> triMesh;
std::vector<unsigned int> triMeshIdx;

void loadOBJ(const char* filePath, MATERIAL materialType, double ior, vec3 albedo, double mat4[]);

typedef struct distant_light {
	double intensity;
	vec3 dir;
	vec3 normalizedColor;

	distant_light(double Li, vec3 direction, vec3 lightColor) :
		intensity(Li), dir(direction), normalizedColor(lightColor)
	{}
} distant_light_t;

typedef struct spherical_light {
	double intensity;
	vec3 pos;
	vec3 normalizedColor;

	spherical_light(double Li, vec3 position, vec3 lightColor) :
		intensity(Li), pos(position), normalizedColor(lightColor)
	{}
} spherical_light_t;


const double EPSILON = 1.0e-6;
const double PI = 3.14159;
const double INV_PI = 1 / PI;

const double INF = 1.0e30;


std::vector<sphere_t> sphrList;
std::vector<plane_t> plnList;
std::vector<triangle_t> triList;


std::vector<distant_light_t> distantLightList;
unsigned int distantLightSize = 1;
std::vector<spherical_light_t> sphericalLightList;
unsigned int sphericalLightSize = 1;


double getRaySphrT(vec3* const rayOrigPtr, vec3* const rayDirPtr, sphere_t* const sphr);
double getRayPlnT(vec3* const rayOrigPtr, vec3* const rayDirPtr, plane_t* const pln);

typedef struct triangle_attribute {
	double t;
	double u;
	double v;
} triangle_attrib_t;

triangle_attrib_t getRayTriT(vec3* const rayOrigPtr, vec3* const rayDirPtr, vec3 vertices[]);


enum GEOMETRY
{
	SPHERE = 0,
	PLANE,
	TRIANGLE,
	TRIANGLE_MESH
};


typedef struct intersected {
	GEOMETRY geoType;
	MATERIAL materialType;
	unsigned int idx;
	double t;
	double ior;
	vec3 pos;
	vec3 normal;
} intersected_t;

void getDiffuseColor(unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr);
void getReflectColor(vec3* const rayDirPtr, unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr);
void getFresnelColor(vec3* const rayDirPtr, unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr);

const unsigned int SAMPLES_PER_PIXEL = 4;
const double INV_SAMPLES_PER_PIXEL = 1.0 / SAMPLES_PER_PIXEL;
const unsigned int MAX_RECURSION_DEPTH = 5;

void castRay(vec3* const rayOrigPtr, vec3* const rayDirPtr, color_t* const outColorPtr, unsigned int depth);
void castShadowRay(intersected_t* const intersectedPoint, vec3* const lightDir, unsigned int depth, double tFarthest, vec3* const normalizedPixelColor, vec3* const lightIntensityColor);
void intersects(vec3* const rayOrigPtr, vec3* const rayDirPtr, GEOMETRY geoType, intersected_t* const intersectedPoint);

bool intersectsBox(vec3* const rayOrigPtr, vec3* const rayDirPtr, vec3* const extMin, vec3* const extMax, double* const tPtr = NULL);

// Consulted https://github.com/jbikker/bvh_article for BVH
typedef struct BVHNode {
	vec3 extMin;
	vec3 extMax;
	unsigned int left;
	unsigned int cnt;

	BVHNode(unsigned int leftIdx, unsigned int primCnt, vec3 extentLow = vec3(INF), vec3 extentHigh = vec3(-INF)) :
		left(leftIdx),
		cnt(primCnt),
		extMin(extentLow),
		extMax(extentHigh)
	{}
} BVHNode_t;

std::vector<BVHNode_t> bvhNode;
// Skipped one node for more efficient GPU caching
unsigned int bvhCurrentNode = 2;

void updateExtent(unsigned int nodeIndex);
void updateExtent(vec3* const extMin, vec3* const extMax, vec3* const centroid);

void subdivide(unsigned int nodeIndex);
void populateBVH(void);

void intersectsBVH(vec3* const rayOrigPtr, vec3* const rayDirPtr, const unsigned int nodeIndex, intersected_t* intersectedPoint, triangle_attrib_t* const attrib = NULL);

double surfaceAreaHeuristic(BVHNode_t* const node, double divide, int axis);

bool isExitPressed = false;
bool isEscPressed = false;
bool isRendering();

int main(void)
{
	const double ASPECT_RATIO = 16.0 / 9.0;
	const int IMG_HEIGHT = 400;
	// const int IMG_HEIGHT = 30;
	const int IMG_WIDTH = static_cast<int>(IMG_HEIGHT * ASPECT_RATIO);


	// Spheres

	sphrList.reserve(3);

	sphrList.emplace_back(
		// Material
		DIFFUSE,
		// Index of Refraction
		0.0,
		// Radius
		1.5,
		// Center
		vec3(4.0, -0.5, -5.0),
		// vec3(0.0, 0.0, -7.0),
		// Albedo
		vec3(0.1, 0.5, 0.9)
	);


	sphrList.emplace_back(
		// Material
		MIRROR,
		// Index of Refraction
		0.0,
		// Radius
		1.3,
		// Center
		vec3(-3.8, 0.0, -4.0),
		// Reflectance but it's still called albedo
		vec3(0.8, 0.8, 0.8)
	);



	sphrList.emplace_back(
		// Material
		GLASS,
		// Index of Refraction
		1.3,
		// Radius
		0.6,
		// Center
		vec3(-1.5, 0.0, -3.2),
		// vec3(-1.5, 0.0, -3.5),
		// Reflectance but it's still called albedo
		vec3(1.0)
	);


	// Planes

	plnList.reserve(1);

	plnList.emplace_back(
		// Material
		DIFFUSE,
		// Index of Refraction
		0.0,
		// Point
		vec3(0.0, -2.0, -3.0),
		// Normal
		vec3(0.0, 1.0, 0.0),
		// Albedo
		vec3(0.5)
	);


	// Triangles
	// Vertices need to be ordered in CCW winding.

	triList.reserve(1);
	triMesh.reserve(6320);
	triMeshIdx.reserve(6320);

	triList.emplace_back(
		// Material
		DIFFUSE,
		// Index of Refraction
		0.0,
		// Vertex 0
		vec3(0.0, 0.5, -3.8),
		// Vertex 1
		vec3(3.0, 1.0, -4.0),
		// Vertex 2
		vec3(1.0, 1.5, -3.5),
		// Albedo
		vec3(0.0, 0.1, 0.9)
	);


	// Triangular Meshes

	double mat4[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -1.8,
		0.0, 0.0, 1.0, -6.0,
		0.0, 0.0, 0.0, 1.0
	};
	loadOBJ("teapot.obj", DIFFUSE, 0.0, vec3(0.8, 0.5, 0.1), mat4);
	std::clock_t beforePopulation = std::clock();
	populateBVH();
	std::clock_t afterPopulation = std::clock();

	// Light Sources

	distantLightList.reserve(1);

	distantLightList.emplace_back(
		// Intensity
		2.0 * PI,
		//Light direction
		unit_vector(vec3(-1.0, -1.0, -1.0)),
		// Normalized light color
		vec3(1.0)
	);

	sphericalLightList.reserve(1);

	sphericalLightList.emplace_back(
		// Intensity
		600.0 * PI,
		// Position
		vec3(1.0, 5.0, -3.0),
		// Normalized light color
		vec3(1.0, 1.0, 1.0)
	);

	// Frambuffer
	std::vector<color_t> img(IMG_HEIGHT * IMG_WIDTH);

	std::vector<unsigned int> verticalIterator;
	verticalIterator.reserve(IMG_HEIGHT);

	std::vector<unsigned int> horizontalIterator;
	verticalIterator.reserve(IMG_WIDTH);

	for (unsigned int i = 0; i < IMG_HEIGHT; i++)
	{
		verticalIterator.emplace_back(i);
	}
	for (unsigned int j = 0; j < IMG_WIDTH; j++)
	{
		horizontalIterator.emplace_back(j);
	}

	// Camera
	vec3 rayOrig(0.0, 0.0, 0.0);
	vec3 camTarget(0.0, 0.0, -1.0);

	double focalLength = (rayOrig - camTarget).length();

	vec3 worldUp = vec3(0.0, 1.0, 0.0);
	vec3 camZ = unit_vector(rayOrig - camTarget);
	vec3 camX = unit_vector(cross(worldUp, camZ));
	vec3 camY = cross(camZ, camX);
	double fov = 90.0;

	/*
	// Consulted "Ray Tracing in One Weekend" and "Scratch a Pixel" for viewport
	// https://raytracing.github.io/books/RayTracingInOneWeekend.html#rays,asimplecamera,andbackground
	// https://raytracing.github.io/books/RayTracingInOneWeekend.html#positionablecamera
	// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays.html
	*/

	vec3 viewpoint_u = 2.0 * focalLength * tan(fov * 0.5 * PI / 180.0) * ASPECT_RATIO * camX;
	vec3 viewpoint_v = 2.0 * focalLength * tan(fov * 0.5 * PI / 180.0) * camY;
	vec3 pixel_delta_u = viewpoint_u * 1.0 / IMG_WIDTH;
	vec3 pixel_delta_v = viewpoint_v * 1.0 / IMG_HEIGHT;

	vec3 viewpointUpperLeft = camTarget - 0.5 * viewpoint_u + 0.5 * viewpoint_v - focalLength * camZ;

	vec3 pixel00 = viewpointUpperLeft + 0.5 * (pixel_delta_u - pixel_delta_v);

	// Consulted https://wiki.libsdl.org/SDL3/SDL_CreateWindow for window creation
	SDL_Init(SDL_INIT_VIDEO);

	SDL_Window* window;

	window = SDL_CreateWindow(
		"Ray Tracer From Scratch",
		IMG_WIDTH,
		IMG_HEIGHT,
		0
	);

	if (NULL == window)
	{
		SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Failed to create window.\n%s\n", SDL_GetError());
		return -1;
	}

	SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);

	std::clock_t beforeRender = std::clock();
#ifdef PARALLEL
	// Consulted https://www.youtube.com/watch?v=46ddlUImiQA for multithreading
	std::for_each(std::execution::par, verticalIterator.begin(), verticalIterator.end(),
		[&](unsigned int i) {
			std::for_each(std::execution::par, horizontalIterator.begin(), horizontalIterator.end(),
				[&, i](unsigned int j) {

					color_t outColor = { 0, 0, 0 };

#ifdef ANTIALIASING

					for (int sample = 0; sample < SAMPLES_PER_PIXEL; sample++)
					{
						color_t sampleColor = { 0, 0, 0 };

						vec3 viewpointLoc = pixel00 + pixel_delta_u * (j + random_double(-0.5, 0.5)) - pixel_delta_v * (i + random_double(-0.5, 0.5));
						vec3 rayDir = unit_vector(viewpointLoc - rayOrig);

						castRay(&rayOrig, &rayDir, &sampleColor, 0);

						outColor.red += std::min(static_cast<Uint8>(sampleColor.red * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
						outColor.green += std::min(static_cast<Uint8>(sampleColor.green * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
						outColor.blue += std::min(static_cast<Uint8>(sampleColor.blue * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
					}
#endif
#ifndef ANTIALIASING
					vec3 viewpointLoc = pixel00 + pixel_delta_u * j - pixel_delta_v * i;

					vec3 rayDir = unit_vector(viewpointLoc - rayOrig);

					castRay(&rayOrig, &rayDir, &outColor, 0);
#endif


					img[i * IMG_WIDTH + j] = outColor;
				});
		});

	std::clock_t beforePrint = std::clock();
	for (int i = 0; i < IMG_HEIGHT; i++)
	{
		for (int j = 0; j < IMG_WIDTH; j++)
		{
			SDL_SetRenderDrawColor(renderer, img[i * IMG_WIDTH + j].red, img[i * IMG_WIDTH + j].green, img[i * IMG_WIDTH + j].blue, SDL_ALPHA_OPAQUE);
			SDL_RenderPoint(renderer, static_cast<float>(j), static_cast<float>(i));
		}
	}
	std::clock_t afterPrint = std::clock();
#endif
#ifndef PARALLEL
	for (int i = 0; i < IMG_HEIGHT; i++)
	{
		for (int j = 0; j < IMG_WIDTH; j++)
		{
			color_t outColor = { 0, 0, 0 };

#ifdef ANTIALIASING
			for (int sample = 0; sample < SAMPLES_PER_PIXEL; sample++)
			{
				color_t sampleColor = { 0, 0, 0 };

				vec3 viewpointLoc = pixel00 + pixel_delta_u * (j + random_double(-0.5, 0.5)) - pixel_delta_v * (i + random_double(-0.5, 0.5));
				vec3 rayDir = unit_vector(viewpointLoc - rayOrig);

				castRay(&rayOrig, &rayDir, &sampleColor, 0);

				outColor.red += std::min(static_cast<Uint8>(sampleColor.red * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
				outColor.green += std::min(static_cast<Uint8>(sampleColor.green * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
				outColor.blue += std::min(static_cast<Uint8>(sampleColor.blue * INV_SAMPLES_PER_PIXEL), MAX_COLOR);
			}
#endif
#ifndef ANTIALIASING
			vec3 viewpointLoc = pixel00 + pixel_delta_u * j - pixel_delta_v * i;

			vec3 rayDir = unit_vector(viewpointLoc - rayOrig);

			castRay(&rayOrig, &rayDir, &outColor, 0);
#endif

			SDL_SetRenderDrawColor(renderer, outColor.red, outColor.green, outColor.blue, SDL_ALPHA_OPAQUE);
			SDL_RenderPoint(renderer, static_cast<float>(j), static_cast<float>(i));
		}
	}
#endif
	std::clock_t afterRender = std::clock();

	SDL_RenderPresent(renderer);


	while (isRendering())
	{
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			isExitPressed = SDL_EVENT_QUIT == event.type;

			// https://wiki.libsdl.org/SDL3/BestKeyboardPractices
			isEscPressed = SDL_EVENT_KEY_DOWN == event.type && event.key.key == SDLK_ESCAPE;
		}
	}

	SDL_DestroyWindow(window);
	SDL_Quit();

	std::clog << "BVH Population took " << (afterPopulation - beforePopulation) * CLOCK_COEFFICIENT << " seconds.\n";
	std::clog << "Render took " << (afterRender - beforeRender) * CLOCK_COEFFICIENT << " seconds.\n";
#ifdef PARALLEL
	std::clog << "Print took " << (afterPrint - beforePrint) * CLOCK_COEFFICIENT << " seconds.\n" << std::flush;
#endif

	return 0;
}

double getRaySphrT(vec3* const rayOrigPtr, vec3* const rayDirPtr, sphere_t* const sphr)
{
	/*
	// Consulted "Computer Graphics from Scratch" by Gabriel Gambetta for formula
	// https://gabrielgambetta.com/computer-graphics-from-scratch/02-basic-raytracing.html
	*/

	vec3 CO = *rayOrigPtr - sphr->center;

	assert(std::fabs(1.0 - (*rayDirPtr).length()) < EPSILON);

	// *rayDir is normalized, so a = dot(*rayDir, *rayDir) = 1.0
	double a = 1.0;
	double b = 2.0 * dot(CO, *rayDirPtr);
	double c = dot(CO, CO) - (sphr->radius) * (sphr->radius);

	double t0 = -1.0;
	double t1 = -1.0;

	double discr = b * b - 4.0 * a * c;

	if (discr < 0.0)
	{
		// No solution, no intersection.
		return -1.0;
	}
	else if (discr == 0.0)
	{
		t0 = t1 = -0.5 * b / a;
	}
	else
	{
		// This prevents catastrophic cancellation.
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html
		double q = (b < 0.0) ? -0.5 * (b + std::sqrt(discr)) : -0.5 * (b - std::sqrt(discr));
		t0 = q / a;
		t1 = c / q;
	}


	if (t0 > t1)
	{
		double tempT0 = t0;
		double tempT1 = t1;
		t0 = tempT1;
		t1 = tempT0;
	}

	if (t0 < EPSILON)
	{
		return t1;
	}


	return t0;
}

double getRayPlnT(vec3* const rayOrigPtr, vec3* const rayDirPtr, plane_t* const pln)
{
	;
	double denom = dot(*rayDirPtr, pln->normal);

#ifdef CULLING
	/*
	// if the dot(*rayDir, pln->normal) is 0 (or close to 0),
	// the ray is parallel to the plane or the plane completely coincides with the ray.
	*/
	bool intersectsBackface = denom > EPSILON;
	bool isParallelToPlane = std::abs(denom) <= EPSILON;
	if (intersectsBackface || isParallelToPlane)
	{
		return -1.0;
	}
#endif
#ifndef CULLING
	// Disabled back-face culling for refraction

	bool isParallelToPlane = std::abs(denom) <= EPSILON;
	if (isParallelToPlane)
	{
		return -1.0;
	}
#endif
	double t = dot(pln->point - *rayOrigPtr, pln->normal) / denom;
	return t;
}

triangle_attrib_t getRayTriT(vec3* const rayOrigPtr, vec3* const rayDirPtr, vec3 vertices[])
{
	/*
	// Consulted "Fast, Minimum Storage Ray/Triangle Intersection" by Möller–Trumbore for the equation
	// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
	*/
	vec3 E1 = vertices[1] - vertices[0];
	vec3 E2 = vertices[2] - vertices[0];
	vec3 T = *rayOrigPtr - vertices[0];
	vec3 P = cross(*rayDirPtr, E2);
	vec3 Q = cross(T, E1);
	double det = dot(P, E1);

	triangle_attrib_t attrib;

#ifdef CULLING
	// Back Face Culling for Triangles
	/*
	// If the determinant is 0.0, there is no solution.
	// If the determinant is less than 0.0, then dot(-rayDir, cross(E1, E2)) < 0.0
	// and dot(rayDir, cross(E1, E2)) > 0, which means the triangle is back-facing.
	*/
	if (0.0 >= det)
	{
		attrib.t = -1.0;
		return attrib;
	}
#endif
#ifndef CULLING
	// Disabled back-face culling for refraction

	if (std::abs(det) <= EPSILON)
	{
		attrib.t = -1.0;
		return attrib;
	}
#endif

	double invDet = 1 / det;

	double u = invDet * dot(P, T);
	if (0.0 > u || 1.0 < u)
	{
		attrib.t = -1.0;
		return attrib;
	}

	attrib.u = u;

	double v = invDet * dot(Q, *rayDirPtr);
	if (0.0 > v || 1.0 < u + v)
	{
		attrib.t = -1.0;
		return attrib;
	}

	attrib.v = v;

	attrib.t = invDet * dot(Q, E2);
	return attrib;
}

void getDiffuseColor(unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr)
{
	vec3 normalizedPixelColor = vec3(0.0);

	for (unsigned int sphericalLight = 0; sphericalLight < sphericalLightSize; sphericalLight++)
	{
		spherical_light_t* sphrLightPtr = &sphericalLightList[sphericalLight];

		vec3 lightDir = sphrLightPtr->pos - intersectedPoint->pos;

		double radiusSquared = lightDir.length_squared();
		double radius = std::sqrt(radiusSquared);
		lightDir = lightDir / radius;

		assert(std::abs(1.0 - lightDir.length()) <= EPSILON);

		vec3 lightIntensityColor = sphrLightPtr->intensity * sphrLightPtr->normalizedColor;

		castShadowRay(intersectedPoint, &lightDir, depth + 1, radius, &normalizedPixelColor, &lightIntensityColor);
	}
	for (unsigned int distantLight = 0; distantLight < distantLightSize; distantLight++)
	{
		distant_light_t* distLightPtr = &distantLightList[distantLight];

		vec3 lightDir = -(distLightPtr->dir);
		assert(std::abs(1.0 - lightDir.length()) <= EPSILON);

		vec3 lightIntensityColor = distLightPtr->intensity * distLightPtr->normalizedColor;

		castShadowRay(intersectedPoint, &lightDir, depth + 1, INF, &normalizedPixelColor, &lightIntensityColor);
	}

	normalizedPixelColor = INV_PI * normalizedPixelColor;

	if (TRIANGLE_MESH == intersectedPoint->geoType)
	{
		normalizedPixelColor = (&triMesh[triMeshIdx[intersectedPoint->idx]])->albedo * normalizedPixelColor;
	}
	else if (TRIANGLE == intersectedPoint->geoType)
	{
		normalizedPixelColor = (&triList[intersectedPoint->idx])->albedo * normalizedPixelColor;
	}
	else if (SPHERE == intersectedPoint->geoType)
	{
		normalizedPixelColor = (&sphrList[intersectedPoint->idx])->albedo * normalizedPixelColor;
	}
	else if (PLANE == intersectedPoint->geoType)
	{
		normalizedPixelColor = (&plnList[intersectedPoint->idx])->albedo * normalizedPixelColor;
	}

	outColorPtr->red = (normalizedPixelColor[0] >= 1.0) ? MAX_COLOR : static_cast<Uint8>(normalizedPixelColor[0] * 256.0);
	outColorPtr->green = (normalizedPixelColor[1] >= 1.0) ? MAX_COLOR : static_cast<Uint8>(normalizedPixelColor[1] * 256.0);
	outColorPtr->blue = (normalizedPixelColor[2] >= 1.0) ? MAX_COLOR : static_cast<Uint8>(normalizedPixelColor[2] * 256.0);
}

void getReflectColor(vec3* const rayDirPtr, unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr)
{
	assert(std::abs(1.0 - intersectedPoint->normal.length()) <= EPSILON);
	vec3 reflectionRayOrig = intersectedPoint->pos + 0.002 * intersectedPoint->normal;
	vec3 reflectionRayDir = *rayDirPtr - 2.0 * dot(*rayDirPtr, intersectedPoint->normal) * intersectedPoint->normal;

	castRay(&reflectionRayOrig, &reflectionRayDir, outColorPtr, depth + 1);
	if (TRIANGLE_MESH == intersectedPoint->geoType)
	{
		triangle_mesh_t* triMeshInstPtr = &triMesh[triMeshIdx[intersectedPoint->idx]];

		outColorPtr->red = static_cast<Uint8>(triMeshInstPtr->albedo[0] * outColorPtr->red);
		outColorPtr->green = static_cast<Uint8>(triMeshInstPtr->albedo[1] * outColorPtr->green);
		outColorPtr->blue = static_cast<Uint8>(triMeshInstPtr->albedo[2] * outColorPtr->blue);
	}
	else if (TRIANGLE == intersectedPoint->geoType)
	{
		triangle_t* tri = &triList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(tri->albedo[0] * outColorPtr->red);
		outColorPtr->green = static_cast<Uint8>(tri->albedo[1] * outColorPtr->green);
		outColorPtr->blue = static_cast<Uint8>(tri->albedo[2] * outColorPtr->blue);
	}
	else if (SPHERE == intersectedPoint->geoType)
	{
		sphere_t* sphr = &sphrList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(sphr->albedo[0] * outColorPtr->red);
		outColorPtr->green = static_cast<Uint8>(sphr->albedo[1] * outColorPtr->green);
		outColorPtr->blue = static_cast<Uint8>(sphr->albedo[2] * outColorPtr->blue);
	}
	else if (PLANE == intersectedPoint->geoType)
	{
		plane_t* pln = &plnList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(pln->albedo[0] * outColorPtr->red);
		outColorPtr->green = static_cast<Uint8>(pln->albedo[1] * outColorPtr->green);
		outColorPtr->blue = static_cast<Uint8>(pln->albedo[2] * outColorPtr->blue);
	}
}

void getFresnelColor(vec3* const rayDirPtr, unsigned int depth, intersected_t* const intersectedPoint, color_t* const outColorPtr)
{
	double etai = 1.0;
	double etat = intersectedPoint->ior;
	vec3 surfaceNormal = intersectedPoint->normal;

	double cosi = dot(-(*rayDirPtr), surfaceNormal);

	bool isBackFacing = 0 >= cosi;

	if (isBackFacing)
	{
		double tempEtai = etai;
		double tempEtat = etat;
		etai = tempEtat;
		etat = tempEtai;
		surfaceNormal = -surfaceNormal;
		cosi = -cosi;
	}

	assert(std::abs(etat) > EPSILON);

	double refractiveIndex = etai / etat;

	if (cosi <= 0.0)
	{
		std::clog << "cosi == " << cosi << '\n';
	}
	assert(cosi > 0.0);

	double costSquared = 1.0 - refractiveIndex * refractiveIndex * (1.0 - cosi * cosi);
	if (0.0 > costSquared)
	{
		getReflectColor(rayDirPtr, depth, intersectedPoint, outColorPtr);
		return;
	}

	double cost = std::sqrt(costSquared);

	double frParallel = (etat * cosi - etai * cost) / (etat * cosi + etai * cost);
	frParallel *= frParallel;

	double frPerpendicular = (etat * cost - etai * cosi) / (etat * cost + etai * cosi);
	frPerpendicular *= frPerpendicular;

	double fr = 0.5 * (frParallel + frPerpendicular);
	double ft = 1.0 - fr;

	vec3 transmissionHorizontal = ((*rayDirPtr) + cosi * surfaceNormal) * refractiveIndex;
	vec3 transmissionVertical = -cost * surfaceNormal;
	vec3 transmissionRayDir = transmissionVertical + transmissionHorizontal;

	assert(std::abs(1.0 - transmissionRayDir.length()) <= EPSILON);

	vec3 transmissionRayOrig = intersectedPoint->pos - 0.002 * surfaceNormal;

	color_t refractedColor = { 0, 0, 0 };
	color_t reflectedColor = { 0, 0, 0 };

	if (ft > 0.0)
	{
		castRay(&transmissionRayOrig, &transmissionRayDir, &refractedColor, depth + 1);
	}
	getReflectColor(rayDirPtr, depth, intersectedPoint, &reflectedColor);

	if (TRIANGLE_MESH == intersectedPoint->geoType)
	{
		triangle_mesh_t* triMeshInstPtr = &triMesh[triMeshIdx[intersectedPoint->idx]];

		outColorPtr->red = static_cast<Uint8>(triMeshInstPtr->albedo[0] * (ft * refractedColor.red + fr * reflectedColor.red));
		outColorPtr->green = static_cast<Uint8>(triMeshInstPtr->albedo[1] * (ft * refractedColor.green + fr * reflectedColor.green));
		outColorPtr->blue = static_cast<Uint8>(triMeshInstPtr->albedo[2] * (ft * refractedColor.blue + fr * reflectedColor.blue));
	}
	else if (TRIANGLE == intersectedPoint->geoType)
	{
		triangle_t* tri = &triList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(tri->albedo[0] * (ft * refractedColor.red + fr * reflectedColor.red));
		outColorPtr->green = static_cast<Uint8>(tri->albedo[1] * (ft * refractedColor.green + fr * reflectedColor.green));
		outColorPtr->blue = static_cast<Uint8>(tri->albedo[2] * (ft * refractedColor.blue + fr * reflectedColor.blue));
	}
	else if (SPHERE == intersectedPoint->geoType)
	{
		sphere_t* sphr = &sphrList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(sphr->albedo[0] * (ft * refractedColor.red + fr * reflectedColor.red));
		outColorPtr->green = static_cast<Uint8>(sphr->albedo[1] * (ft * refractedColor.green + fr * reflectedColor.green));
		outColorPtr->blue = static_cast<Uint8>(sphr->albedo[2] * (ft * refractedColor.blue + fr * reflectedColor.blue));
	}
	else if (PLANE == intersectedPoint->geoType)
	{
		plane_t* pln = &plnList[intersectedPoint->idx];

		outColorPtr->red = static_cast<Uint8>(pln->albedo[0] * (ft * refractedColor.red + fr * reflectedColor.red));
		outColorPtr->green = static_cast<Uint8>(pln->albedo[1] * (ft * refractedColor.green + fr * reflectedColor.green));
		outColorPtr->blue = static_cast<Uint8>(pln->albedo[2] * (ft * refractedColor.blue + fr * reflectedColor.blue));
	}
}

void castRay(vec3* const rayOrigPtr, vec3* const rayDirPtr, color_t* const outColorPtr, unsigned int depth)
{
	assert(std::abs(1.0 - (*rayDirPtr).length()) < EPSILON);

	assert(MAX_RECURSION_DEPTH >= depth);
	if (MAX_RECURSION_DEPTH == depth)
	{
		outColorPtr->red = backgroundColor.red;
		outColorPtr->green = backgroundColor.green;
		outColorPtr->blue = backgroundColor.blue;
		return;
	}

	intersected_t intersectedPoint;
	intersectedPoint.t = INF;

	assert(intersectedPoint.t == INF);

	triangle_attrib_t attrib;
	intersectsBVH(rayOrigPtr, rayDirPtr, 0, &intersectedPoint, &attrib);

	intersects(rayOrigPtr, rayDirPtr, TRIANGLE, &intersectedPoint);
	intersects(rayOrigPtr, rayDirPtr, SPHERE, &intersectedPoint);
	intersects(rayOrigPtr, rayDirPtr, PLANE, &intersectedPoint);

	// If ray doesn't intersect anything
	if (INF == intersectedPoint.t)
	{
		outColorPtr->red = backgroundColor.red;
		outColorPtr->green = backgroundColor.green;
		outColorPtr->blue = backgroundColor.blue;
		return;
	}

	// General computations about the intersected point
	intersectedPoint.pos = *rayOrigPtr + intersectedPoint.t * (*rayDirPtr);

	// Geometry specific computations
	if (TRIANGLE_MESH == intersectedPoint.geoType)
	{
		triangle_mesh_t* triMeshInstPtr = &triMesh[triMeshIdx[intersectedPoint.idx]];

		intersectedPoint.materialType = triMeshInstPtr->materialType;
		intersectedPoint.ior = triMeshInstPtr->ior;

		double w = 1.0 - (attrib.u + attrib.v);
		assert(0.0 <= w && 1.0 >= w);
		intersectedPoint.normal = unit_vector(w * meshVertexNormal[triMeshInstPtr->ev0] + attrib.u * meshVertexNormal[triMeshInstPtr->ev1] + attrib.v * meshVertexNormal[triMeshInstPtr->ev2]);
	}
	else if (TRIANGLE == intersectedPoint.geoType)
	{
		triangle_t* tri = &triList[intersectedPoint.idx];

		intersectedPoint.normal = unit_vector(cross(tri->v2 - tri->v0, tri->v1 - tri->v0));

		intersectedPoint.materialType = tri->materialType;
		intersectedPoint.ior = tri->ior;
	}
	else if (SPHERE == intersectedPoint.geoType)
	{
		sphere_t* sphr = &sphrList[intersectedPoint.idx];

		double invRadius = 1.0 / sphr->radius;
		intersectedPoint.normal = invRadius * (intersectedPoint.pos - sphr->center);

		intersectedPoint.materialType = sphr->materialType;
		intersectedPoint.ior = sphr->ior;
	}
	else if (PLANE == intersectedPoint.geoType)
	{
		plane_t* pln = &plnList[intersectedPoint.idx];

		assert(std::abs(1.0 - pln->normal.length()) < EPSILON);

		intersectedPoint.normal = pln->normal;

		intersectedPoint.materialType = pln->materialType;
		intersectedPoint.ior = pln->ior;
	}

	assert(std::abs(1.0 - intersectedPoint.normal.length()) < EPSILON);

	if (DIFFUSE == intersectedPoint.materialType)
	{
		getDiffuseColor(depth, &intersectedPoint, outColorPtr);
	}
	else if (MIRROR == intersectedPoint.materialType)
	{
		getReflectColor(rayDirPtr, depth, &intersectedPoint, outColorPtr);
	}
	else if (GLASS == intersectedPoint.materialType)
	{
		getFresnelColor(rayDirPtr, depth, &intersectedPoint, outColorPtr);
	}
}

void intersects(vec3* const rayOrigPtr, vec3* const rayDirPtr, GEOMETRY geoType, intersected_t* const intersectedPoint)
{
	assert(std::abs(1.0 - (*rayDirPtr).length()) < EPSILON);

	union geometry_pointer {
		triangle_t* tri;
		sphere_t* sphr;
		plane_t* pln;
	} geoPtr = {};

	int ptrSize = 0;

	if (TRIANGLE == geoType)
	{
		ptrSize = static_cast<int>(triList.size());
	}
	else if (SPHERE == geoType)
	{
		ptrSize = static_cast<int>(sphrList.size());
	}
	else if (PLANE == geoType)
	{
		ptrSize = static_cast<int>(plnList.size());
	}

	for (int cnt = 0; cnt < ptrSize; cnt++)
	{
		double tTemp = -1.0;
		bool intersectsCloserGeometry = false;

		if (TRIANGLE == geoType)
		{
			geoPtr.tri = &triList[cnt];
			vec3 vertices[3] = { geoPtr.tri->v0, geoPtr.tri->v1, geoPtr.tri->v2 };
			tTemp = getRayTriT(rayOrigPtr, rayDirPtr, vertices).t;
		}
		else if (SPHERE == geoType)
		{
			geoPtr.sphr = &sphrList[cnt];
			tTemp = getRaySphrT(rayOrigPtr, rayDirPtr, geoPtr.sphr);
		}
		else if (PLANE == geoType)
		{
			geoPtr.pln = &plnList[cnt];
			tTemp = getRayPlnT(rayOrigPtr, rayDirPtr, geoPtr.pln);
		}

		intersectsCloserGeometry = EPSILON < tTemp && tTemp < intersectedPoint->t;

		if (intersectsCloserGeometry)
		{
			if (TRIANGLE == geoType)
			{
				intersectedPoint->idx = cnt;
			}
			else if (SPHERE == geoType)
			{
				intersectedPoint->idx = cnt;
			}
			else if (PLANE == geoType)
			{
				intersectedPoint->idx = cnt;
			}

			intersectedPoint->t = tTemp;
			intersectedPoint->geoType = geoType;
		}
	}
}

void castShadowRay(intersected_t* const intersectedPoint, vec3* const lightDir, unsigned int depth, double tFarthest, vec3* const normalizedPixelColor, vec3* const lightIntensityColor)
{
	assert(MAX_RECURSION_DEPTH >= depth);
	if (MAX_RECURSION_DEPTH == depth)
	{
		return;
	}

	intersected_t intersectedShadowRay;
	intersectedShadowRay.t = tFarthest;

	intersectsBVH(&(intersectedPoint->pos), lightDir, 0, &intersectedShadowRay);

	intersects(&(intersectedPoint->pos), lightDir, TRIANGLE, &intersectedShadowRay);
	intersects(&(intersectedPoint->pos), lightDir, SPHERE, &intersectedShadowRay);
	intersects(&(intersectedPoint->pos), lightDir, PLANE, &intersectedShadowRay);

	bool isDirectlyIlluminated = intersectedShadowRay.t >= tFarthest;
	if (isDirectlyIlluminated == false)
	{
		color_t diffuseColor = { 0, 0, 0 };
		*lightDir = random_on_hemisphere(intersectedPoint->normal);
		castRay(&(intersectedPoint->pos), lightDir, &diffuseColor, depth + 1);

		(*lightIntensityColor)[0] = static_cast<double>(diffuseColor.red * COLOR_COEFFICIENT);
		(*lightIntensityColor)[1] = static_cast<double>(diffuseColor.green * COLOR_COEFFICIENT);
		(*lightIntensityColor)[2] = static_cast<double>(diffuseColor.blue * COLOR_COEFFICIENT);
	}

	bool isDistantLight = std::abs(INF - tFarthest) < EPSILON;
	double attenuation = isDistantLight ? 1.0 : 1.0 / (4.0 * PI * tFarthest * tFarthest);

	*normalizedPixelColor = *normalizedPixelColor + attenuation * std::fmax(0.0, dot(*lightDir, intersectedPoint->normal)) * (*lightIntensityColor);
}

void loadOBJ(const char* filePath, MATERIAL materialType, double ior, vec3 albedo, double mat4[])
{
	// Based on pseudo-code from https://illinois-cs419.github.io/resources
	// Partially excerpt from https://www.scratchapixel.com/lessons/3d-basic-rendering/obj-file-format/obj-file-format.html
	meshVertex.reserve(6320);
	meshVertexNormal.reserve(6320);

	std::ifstream fileStream(filePath);
	std::string line;

	unsigned int triMeshIdxCnt = 0;

	while (std::getline(fileStream, line))
	{
		std::istringstream lineStream(line);
		std::string attrib;
		lineStream >> attrib;

		if (attrib == "v")
		{
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			lineStream >> x >> y >> z;

			meshVertex.emplace_back(
				x * mat4[0] + y * mat4[1] + z * mat4[2] + mat4[3],
				x * mat4[4] + y * mat4[5] + z * mat4[6] + mat4[7],
				x * mat4[8] + y * mat4[9] + z * mat4[10] + mat4[11]
			);
			meshVertexNormal.emplace_back(0.0, 0.0, 0.0);
		}
		else if (attrib == "f")
		{
			int ev0 = -1;
			int ev1 = -1;
			int ev2 = -1;

			lineStream >> ev0 >> ev1 >> ev2;

			ev0--;
			ev1--;
			ev2--;

			assert(ev0 < meshVertex.size() && ev1 < meshVertex.size() && ev2 < meshVertex.size());

			// Clockwise winding
			vec3 faceNormal = cross(meshVertex[ev1] - meshVertex[ev0], meshVertex[ev2] - meshVertex[ev0]);
			meshVertexNormal[ev0] = (meshVertexNormal[ev0] + 0.5 * faceNormal);
			meshVertexNormal[ev1] = (meshVertexNormal[ev1] + 0.5 * faceNormal);
			meshVertexNormal[ev2] = (meshVertexNormal[ev2] + 0.5 * faceNormal);

			triMesh.emplace_back(
				materialType,
				ior,
				ev0,
				ev1,
				ev2,
				albedo
			);
			triMeshIdx.emplace_back(triMeshIdxCnt++);
		}
		else if (attrib == "#")
		{
			continue;
		}
	}

	for (int normal = 0; normal < meshVertexNormal.size(); normal++)
	{
		meshVertexNormal[normal] = unit_vector(meshVertexNormal[normal]);
	}

}

bool intersectsBox(vec3* const rayOrigPtr, vec3* const rayDirPtr, vec3* const extMin, vec3* const extMax, double* const tPtr)
{
	// Pseudo-code from https://education.siggraph.org/static/HyperGraph/raytrace/rtinter3.htm
	double tNear = -INF;
	double tFar = INF;
	for (int axis = 0; axis < 3; axis++)
	{
		bool isParallelToPlane = (*rayDirPtr)[axis] == 0.0
			&& ((*extMin)[axis] > (*rayOrigPtr)[axis]
				|| (*extMax)[axis] < (*rayOrigPtr)[axis]);
		if (isParallelToPlane)
		{
			return false;
		}

		double invDir = 1.0 / (*rayDirPtr)[axis];

		double t0 = ((*extMin)[axis] - (*rayOrigPtr)[axis]) * invDir;
		double t1 = ((*extMax)[axis] - (*rayOrigPtr)[axis]) * invDir;

		if (t0 > t1)
		{
			double tempT0 = t0;
			double tempT1 = t1;
			t0 = tempT1;
			t1 = tempT0;
		}

		if (tNear < t0)
		{
			tNear = t0;
		}

		if (tFar > t1)
		{
			tFar = t1;
		}

		if (tNear > tFar || tFar < 0.0)
		{
			return false;
		}
	}

	if (NULL != tPtr)
	{
		*tPtr = tNear;
	}

	return true;
}


void updateExtent(unsigned int nodeIndex)
{
	BVHNode_t* node = &bvhNode[nodeIndex];

	for (unsigned int primIndex = node->left; primIndex < node->left + node->cnt; primIndex++)
	{
		vec3* const v0 = &meshVertex[triMesh[triMeshIdx[primIndex]].ev0];
		vec3* const v1 = &meshVertex[triMesh[triMeshIdx[primIndex]].ev1];
		vec3* const v2 = &meshVertex[triMesh[triMeshIdx[primIndex]].ev2];

		node->extMin[0] = std::min(node->extMin[0], std::min((*v0)[0], std::min((*v1)[0], (*v2)[0])));
		node->extMin[1] = std::min(node->extMin[1], std::min((*v0)[1], std::min((*v1)[1], (*v2)[1])));
		node->extMin[2] = std::min(node->extMin[2], std::min((*v0)[2], std::min((*v1)[2], (*v2)[2])));

		node->extMax[0] = std::max(node->extMax[0], std::max((*v0)[0], std::max((*v1)[0], (*v2)[0])));
		node->extMax[1] = std::max(node->extMax[1], std::max((*v0)[1], std::max((*v1)[1], (*v2)[1])));
		node->extMax[2] = std::max(node->extMax[2], std::max((*v0)[2], std::max((*v1)[2], (*v2)[2])));
	}
}

void updateExtent(vec3* const extMin, vec3* const extMax, vec3* const centroid)
{
	(*extMin)[0] = std::min((*extMin)[0], (*centroid)[0]);
	(*extMin)[1] = std::min((*extMin)[1], (*centroid)[1]);
	(*extMin)[2] = std::min((*extMin)[2], (*centroid)[2]);

	(*extMax)[0] = std::max((*extMax)[0], (*centroid)[0]);
	(*extMax)[1] = std::max((*extMax)[1], (*centroid)[1]);
	(*extMax)[2] = std::max((*extMax)[2], (*centroid)[2]);
}

void populateBVH(void)
{
	int triMeshTotal = static_cast<int>(triMesh.size());

	bvhNode.reserve(2 * triMeshTotal);

	double oneThird = 1.0 / 3.0;
	for (int triMeshCnt = 0; triMeshCnt < triMeshTotal; triMeshCnt++)
	{
		triMesh[triMeshIdx[triMeshCnt]].centroid = (meshVertex[triMesh[triMeshIdx[triMeshCnt]].ev0]
			+ meshVertex[triMesh[triMeshIdx[triMeshCnt]].ev1]
			+ meshVertex[triMesh[triMeshIdx[triMeshCnt]].ev2]) * oneThird;
	}


	bvhNode.emplace_back(
		// left
		0,
		// cnt
		triMeshTotal
	);
	BVHNode_t* root = &bvhNode[0];

	// Aggregate 2 nodes in the same GPU cache
	bvhNode.emplace_back(
		// left
		0,
		// cnt
		0
	);

	updateExtent(0);

	subdivide(0);
}

void subdivide(unsigned int nodeIndex)
{
	BVHNode_t* currentNode = &bvhNode[nodeIndex];

	double bestCost = INF;
	double bestDivide = 0;

	int longestAxis = -1;
	double longestAxisLength = -1.0;

	for (unsigned int axis = 0; axis < 3; axis++)
	{
		double currentAxisLength = currentNode->extMax[axis] - currentNode->extMin[axis];
		if (longestAxisLength < currentAxisLength)
		{
			longestAxis = axis;
			longestAxisLength = currentAxisLength;
		}
	}

	assert(-1.0 != longestAxis);

	double divideStride = longestAxisLength / 4.0;
	for (unsigned int divideIndex = 0; 4 > divideIndex; divideIndex++)
	{
		double divide = currentNode->extMin[longestAxis] + divideIndex * divideStride;

		double currentCost = surfaceAreaHeuristic(currentNode, divide, longestAxis);

		if (bestCost > currentCost)
		{
			bestDivide = divide;

			bestCost = currentCost;
		}
	}

	int nCurrentNode = currentNode->cnt;
	vec3 diffCurrentNode = currentNode->extMax - currentNode->extMin;
	double areaCurrentNode = diffCurrentNode[0] * diffCurrentNode[1] + diffCurrentNode[0] * diffCurrentNode[2] + diffCurrentNode[1] * diffCurrentNode[2];

	double currentNodeCost = nCurrentNode * areaCurrentNode;
	if (bestCost >= currentNodeCost)
	{
		return;
	}

	int left = currentNode->left;
	int right = left + currentNode->cnt - 1;

	while (left <= right)
	{
		if (triMesh[triMeshIdx[left]].centroid[longestAxis] < bestDivide)
		{
			left++;
			continue;
		}

		unsigned int tempLeftTriMeshIdx = triMeshIdx[left];
		unsigned int tempRightTriMeshIdx = triMeshIdx[right];

		triMeshIdx[right] = tempLeftTriMeshIdx;
		triMeshIdx[left] = tempRightTriMeshIdx;

		right--;
	}

	unsigned int leftCnt = left - currentNode->left;

	bool hasOneChildNode = (0 == leftCnt || currentNode->cnt == leftCnt);
	if (hasOneChildNode)
	{
		return;
	}

	// Left node
	bvhNode.emplace_back(
		// left
		currentNode->left,
		// cnt
		leftCnt
	);

	bvhCurrentNode++;

	// Right node
	bvhNode.emplace_back(
		// left
		left,
		// cnt
		currentNode->cnt - leftCnt
	);

	bvhCurrentNode++;


	currentNode->left = bvhCurrentNode - 2;
	currentNode->cnt = 0;

	unsigned int bvhLeftChildNode = bvhCurrentNode - 2;
	unsigned int bvhRightChildNode = bvhCurrentNode - 1;

	updateExtent(bvhLeftChildNode);
	updateExtent(bvhRightChildNode);

	subdivide(bvhLeftChildNode);
	subdivide(bvhRightChildNode);
}


double surfaceAreaHeuristic(BVHNode_t* const node, double divide, int axis)
{
	// Consulted https://github.com/jbikker/bvh_article for surface area heuristic
	double cost = 0.0;

	int nLeft = 0;
	int nRight = 0;

	vec3 leftMin = vec3(INF);
	vec3 leftMax = vec3(-INF);

	vec3 rightMin = vec3(INF);
	vec3 rightMax = vec3(-INF);

	for (unsigned int triMeshCnt = 0; triMeshCnt < node->cnt; triMeshCnt++)
	{
		triangle_mesh_t* triMeshInstPtr = &triMesh[triMeshIdx[node->left + triMeshCnt]];
		vec3* const currentVertex = &triMeshInstPtr->centroid;
		bool isOnLeftBox = (*currentVertex)[axis] < divide;
		if (isOnLeftBox)
		{
			updateExtent(&leftMin, &leftMax, &triMeshInstPtr->centroid);

			nLeft++;
		}
		else
		{
			updateExtent(&rightMin, &rightMax, &triMeshInstPtr->centroid);

			nRight++;
		}
	}

	vec3 leftDiff = leftMax - leftMin;
	vec3 rightDiff = rightMax - rightMin;

	double areaLeft = leftDiff[0] * leftDiff[1] + leftDiff[0] * leftDiff[2] + leftDiff[1] * leftDiff[2];
	double areaRight = rightDiff[0] * rightDiff[1] + rightDiff[0] * rightDiff[2] + rightDiff[1] * rightDiff[2];

	cost = static_cast<double>(nLeft * areaLeft + nRight * areaRight);

	return cost;
}


void intersectsBVH(vec3* const rayOrigPtr, vec3* const rayDirPtr, const unsigned int nodeIndex, intersected_t* intersectedPoint, triangle_attrib_t* const attrib)
{
#ifndef VLA
	BVHNode_t* node = &bvhNode[nodeIndex];
	int stackSize = static_cast<int>(triMesh.size()) * 2;
	BVHNode_t** stack = (BVHNode_t**)alloca(sizeof(BVHNode_t*) * stackSize);
	unsigned int currentStackIndex = 0;
#endif
#ifdef VLA
	BVHNode_t* node = &bvhNode[nodeIndex];
	BVHNode_t* stack[2 * triMesh.size()];
	unsigned int currentStackIndex = 0;
#endif

	while (true)
	{
		bool isLeaf = 0 < node->cnt;
		if (isLeaf)
		{
			for (unsigned int left = node->left; left < node->left + node->cnt; left++)
			{
				vec3 vertices[3] = {
					meshVertex[triMesh[triMeshIdx[left]].ev0],
					meshVertex[triMesh[triMeshIdx[left]].ev1],
					meshVertex[triMesh[triMeshIdx[left]].ev2]
				};

				triangle_attrib_t tempAttrib;
				tempAttrib = getRayTriT(rayOrigPtr, rayDirPtr, vertices);

				bool intersectsCloserTriangularMesh = tempAttrib.t < intersectedPoint->t && EPSILON < tempAttrib.t;

				if (intersectsCloserTriangularMesh)
				{
					triangle_mesh_t* triMeshInstPtr = &triMesh[triMeshIdx[left]];

					intersectedPoint->idx = left;
					intersectedPoint->t = tempAttrib.t;

					intersectedPoint->geoType = TRIANGLE_MESH;

					if (NULL != attrib)
					{
						*attrib = tempAttrib;
					}
				}
			}

			if (currentStackIndex == 0)
			{
				break;
			}

			node = stack[--currentStackIndex];
			continue;
		}

		BVHNode_t* leftChild = &bvhNode[node->left];
		BVHNode_t* rightChild = &bvhNode[node->left + 1];

		double tLeft = INF;
		double tRight = INF;

		bool intersectsLeftBox = intersectsBox(rayOrigPtr, rayDirPtr, &leftChild->extMin, &leftChild->extMax, &tLeft);
		bool intersectsRightBox = intersectsBox(rayOrigPtr, rayDirPtr, &rightChild->extMin, &rightChild->extMax, &tRight);

		if (!intersectsLeftBox && !intersectsRightBox)
		{
			if (currentStackIndex == 0)
			{
				break;
			}

			node = stack[--currentStackIndex];
			continue;
		}

		if (tLeft > tRight)
		{
			double tTempLeft = tLeft;
			double tTempRight = tRight;
			tLeft = tTempRight;
			tRight = tTempLeft;

			BVHNode_t* tempLeftChild = leftChild;
			BVHNode_t* tempRightChild = rightChild;
			leftChild = tempRightChild;
			rightChild = tempLeftChild;

			bool tempIntersectsLeftBox = intersectsLeftBox;
			bool tempIntersectsRightBox = intersectsRightBox;
			intersectsLeftBox = tempIntersectsRightBox;
			intersectsRightBox = tempIntersectsLeftBox;
		}

		node = leftChild;
		if (intersectsRightBox)
		{
			stack[currentStackIndex++] = rightChild;
		}
	}
}

bool isRendering()
{
	return !isExitPressed && !isEscPressed;
}