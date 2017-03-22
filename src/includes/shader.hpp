/*
shader465.hpp

Functions to load glsl shaders for use with core
OpenGL programs.

Mike Barnes
8/17/2013
*/

#ifndef __SHADER__
#define __SHADER__

// Load shader from disk into a null-terminated string
inline GLchar * readShaderSource(const char *fileName) {
	GLchar *shaderText = NULL;
	GLint shaderLength = 0;
	FILE *fp;
	fp = fopen(fileName, "r");
	if (fp != NULL) {
		while (fgetc(fp) != EOF) shaderLength++;
		rewind(fp);
		shaderText = (GLchar *)malloc(shaderLength + 1);
		if (shaderText != NULL) fread(shaderText, 1, shaderLength, fp);
		shaderText[shaderLength] = '\0';  // NULL termination
		fclose(fp);
	}
	else {
		printf("error readShaderSource of %s\n", fileName);
		exit(1);
	}
	// printf("shader source: allocated %d size %d \n %s\n", shaderLength + 1, strlen(shaderText), shaderText);
	return shaderText;
}

// check create calls
inline void checkCreate(GLint object, const char * msg) {
	if (object == 0) printf("\t failed:  create %s \n", msg);
	else printf("success:  create %s\n", msg);
}

// check and report status of GLSL functions 
inline void checkShaderStatus(GLint object, GLint status, const char * msg) {
	int len;
	if (!status) {
		GLchar infoLog[MAX_INFO_LOG_SIZE];
		glGetShaderInfoLog(object, MAX_INFO_LOG_SIZE, &len, infoLog);
		printf("failed:  %s \n", msg);
		printf("info log:  %s \n", infoLog);
		exit(EXIT_FAILURE);
	}
	else printf("success:  %s\n", msg);
}

inline void checkProgramStatus(GLint object, GLint status, const char * msg) {
	int len;
	if (!status) {
		GLchar infoLog[MAX_INFO_LOG_SIZE];
		glGetProgramInfoLog(object, MAX_INFO_LOG_SIZE, &len, infoLog);
		printf("failed:  %s \n", msg);
		printf("info log:  %s \n", infoLog);
		exit(EXIT_FAILURE);
	}
	else printf("success:  %s\n", msg);
}

// GLSL initialization, return shaderProgram handle
inline GLuint loadShaders(const GLchar* vShaderFile, const GLchar* fShaderFile) {
	bool result = true;
	GLint vShader = 0;
	GLint fShader = 0;
	GLint status = 0;
	GLint shaderProgram = 0;  // shader program

	printf("load shaders: %s and %s\n", vShaderFile, fShaderFile);
	//read shader files -- make null terminated strings
	GLchar* vSource = readShaderSource(vShaderFile);
	GLchar* fSource = readShaderSource(fShaderFile);
	// printf("create shader\n");
	// create shaderProgram and shaders
	vShader = glCreateShader(GL_VERTEX_SHADER);
	checkCreate(vShader, "vertex shader");
	fShader = glCreateShader(GL_FRAGMENT_SHADER);
	checkCreate(fShader, "fragment shader");

	shaderProgram = glCreateProgram();
	checkCreate(shaderProgram, "shaderProgram");

	// read shader source
	glShaderSource(vShader, 1, (const GLchar**)&vSource, NULL);
	// printf("read vertex shader source \n");   
	glShaderSource(fShader, 1, (const GLchar**)&fSource, NULL);
	// printf("read fShader source\n");   

	// compile shaders
	glCompileShader(vShader);
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &status);
	checkShaderStatus(vShader, status, "vertex shader compile");

	glCompileShader(fShader);
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &status);
	checkShaderStatus(fShader, status, "fragment shader compile");

	// attach shaders to shaderProgram 
	glAttachShader(shaderProgram, vShader);
	printf("attached: vertex shader\n");
	glAttachShader(shaderProgram, fShader);
	printf("attached: fragment shader\n");

	// link shaderProgram
	glLinkProgram(shaderProgram);
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &status);
	checkProgramStatus(shaderProgram, status, "linking");

	glValidateProgram(shaderProgram);
	glGetProgramiv(shaderProgram, GL_VALIDATE_STATUS, &status);
	checkProgramStatus(shaderProgram, status, "validation");

	return shaderProgram;
}

#endif
