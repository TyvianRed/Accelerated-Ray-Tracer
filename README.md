# Accelerated Ray Tracer
![Accelerated Ray Tracer](https://github.com/user-attachments/assets/01931eb4-0e8f-47a9-a256-030792210454)
<br/>
This ray tracer uses BVH and parallel execution policy to output this cool scene. It doesn't use graphics API.
# Build
You need to add PATH=%PATH%;$(ProjectDir)lib\x64 (x86 or arm64 depending on your platform) to the enviornment variable. If your platform isn't x64, you need to adjust include and library directories as well.
# External dependencies
- [Teapot model](https://raw.githubusercontent.com/UIllinoisGraphics/CS296/master/Meshes/teapot.obj)
- [SDL3](https://github.com/libsdl-org/SDL/releases/tag/preview-3.1.3) for window creation
- [vec3.h from Ray Tracing in One Weekend series](https://github.com/RayTracing/raytracing.github.io/blob/release/src/InOneWeekend/vec3.h) for vector math
# Some cool resources I consulted for this project
- [CS 419 Production Computer Graphics](https://illinois-cs419.github.io/)
- [Scratch a Pixel](https://www.scratchapixel.com/index.html)
- [How to Build a BVH](https://github.com/jbikker/bvh_article)
- [Computer Graphics from Scratch](https://gabrielgambetta.com/computer-graphics-from-scratch/)
- [Ray Tracing by The Cherno](https://www.youtube.com/playlist?list=PLlrATfBNZ98edc5GshdBtREv5asFW3yXl)
- [Fast, Minimum Storage Ray/Triangle Intersection](https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf)
- [SDL3 Wiki](https://wiki.libsdl.org/SDL3/FrontPage)
- [Ray Tracing in One Weekend Series](https://github.com/RayTracing/raytracing.github.io/)
- [Ray - Box Intersection](https://education.siggraph.org/static/HyperGraph/raytrace/rtinter3.htm)
