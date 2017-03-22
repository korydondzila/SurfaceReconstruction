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

uniform vec3 cameraPos = vec3(0.0f, 0.0f, 0.0f);

void main()
{
	float dist = distance(cameraPos.z, vs_worldPos.z);
	dist = min(5.0f, dist);
	fragColor = vec4(max(sin(dist * 1.4f), 0), max(sin(dist * 1.4f - 1.0f), 0), max(sin(dist * 1.4f - 2.0f), 0), 1.0f) ;
}