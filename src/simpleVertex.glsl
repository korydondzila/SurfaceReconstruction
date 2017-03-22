/* 
SimpleVertex.glsl

Vertex shader with position, color, normal and ModelViewProject
input and color output.

Mike Barnes
8/17/2013
*/

# version 330 core

in vec4 vPosition, vColor;
out vec3 vs_worldPos;
out vec4 color;

uniform mat4 ModelView;
uniform mat4 MVP;

void main(void)
{
	vec4 position = MVP * vPosition;
	gl_Position = position;
	vs_worldPos = (ModelView * vPosition).xyz;
	color = vColor;
}