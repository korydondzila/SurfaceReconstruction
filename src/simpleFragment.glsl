/* 
SimpleFragment.glsl

Fragment shader with color input and output.

Mike Barnes
8/16/2014
*/

# version 330 core

in vec4 color;
in vec3 vs_worldPos;
out vec4 fragColor;

const float PI = 3.1415926535897932384626433832795;
const float HPI = PI / 2;

uniform vec3 cameraPos = vec3(0.0f, 0.0f, 0.0f);
uniform float radiusOffset = 0.0f;

void main()
{
	float dist = distance(cameraPos, vs_worldPos) - radiusOffset;
	dist = min(2.0f, dist);
	dist = max(0.0f, dist);
	fragColor = vec4(max(sin(dist * HPI + HPI), 0), max(sin(dist * HPI - 0.0f), 0), max(sin(dist * HPI - HPI), 0), 1.0f);
}