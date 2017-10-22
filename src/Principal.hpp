// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef PRINCIPAL_H
#define PRINCIPAL_H

#include "includes/includes.hpp"

namespace HuguesHoppe
{
	// Given points pa[], compute the principal component frame f and the (redundant) eigenvalues eimag.
	// The frame f is guaranteed to be invertible and orthogonal, as the axis
	//   will always have non-zero (albeit very small) lengths.
	// The values eimag will be zero for the axes that should be zero.
	// The frame f is also guaranteed to be right-handed.
	void principal_components(std::vector<glm::vec3> pa, glm::mat4x3& f, glm::vec3& eimag);
	void principal_components2(std::vector<glm::vec3> va, const glm::vec3& avgp, glm::mat4x3& f, glm::vec3& eimag);
	void make_right_handed(glm::mat4x3& f);

} // namespace hh

#endif // PRINCIPAL_H
