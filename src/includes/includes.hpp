/*
includes465.hpp

Include file to set up OS compilation for 465 projects.
comment / uncomment for your operating system

Assumes the "including application) has defined one of the following
"target" operating systems.

Contains several cpp defined constants for project builds.
All *465 constants start and end with 2 underscores.

__Linux__      // Ubuntu or other distro
__MAC__        // Mac OSX
__MinGW__      // Windows, Minimalist Gnu for Windows
__Windows__    // Windows, Visual Studio 201?)

Includes utility functions to load glsl shaders and
AC3D *.tri models.

Mike Barnes
8/25/2016
*/

// defines and includes
#ifndef __INCLUDES__
# define __INCLUDES__
# define GLM_FORCE_PURE  // for latest version of GLM w/ error C2719: ,,,  __declspec(align('16')) won't be aligned
# define BUFFER_OFFSET(x)  ((const GLvoid *) (x))  // OpenGL PG 8th ed. code, in vgl.h 
# define MAX_INFO_LOG_SIZE 2048  // for error messages in loadShaders(...)

#pragma warning(disable: 4996)

# include <stdio.h>  // My examples use printf, I'm not a fan of cin, cout...
# include <stdlib.h>
# include <time.h>
# include <algorithm>
# include <map>
# include <set>
# include <vector>
# include <iterator>
# include <queue>
# include <sys/stat.h>
# include <cstring>

# include <GL/glew.h>
# include <GL/freeglut.h>

// include the glm shader-like math library
# define GLM_FORCE_RADIANS  // use radians not angles
# define GLM_MESSAGES   // compiler messages
# include <glm/glm.hpp>
# include <glm/gtc/constants.hpp>
# include <glm/gtc/matrix_transform.hpp>
# include <glm/gtc/type_ptr.hpp>
# include <glm/gtx/quaternion.hpp>
# include "glmUtils.hpp"  // print matrices and vectors, ... 
# include "shader.hpp"    // load vertex and fragment shaders
# include "pointCloudLoader.hpp"  // load AC3D *.tri model 

// PI to 10 digits
const float PI = glm::pi<float>();

template<class T>
inline void SetDifference(std::set<T>*& s1, std::set<T>* s2)
{
	std::set<T>* result;
	std::set_difference(s1->begin(), s1->end(), s2->begin(), s2->end(), std::inserter(result, result->begin()));
	*s1 = result;
}

inline bool StringICompare(const std::string& str1, const std::string& str2)
{
	if (str1.size() != str2.size())
	{
		return false;
	}

	std::string str1Cpy(str1);
	std::string str2Cpy(str2);
	std::transform(str1Cpy.begin(), str1Cpy.end(), str1Cpy.begin(), ::tolower);
	std::transform(str2Cpy.begin(), str2Cpy.end(), str2Cpy.begin(), ::tolower);

	return (str1Cpy == str2Cpy);
}

#endif
