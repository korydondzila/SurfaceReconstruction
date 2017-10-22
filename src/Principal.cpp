// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Principal.hpp"

namespace HuguesHoppe
{
	inline void rotate(float& v1, float& v2, float tau, float vsin)
	{
		float t1 = v1, t2 = v2;
		v1 -= vsin*(t2 + t1*tau);
		v2 += vsin*(t1 - t2*tau);
	}

	// Swap the contents of two ranges.
	void swap_ranges(glm::vec3& range1, glm::vec3& range2)
	{
		std::swap(range1[0], range2[0]);
		std::swap(range1[1], range2[1]);
		std::swap(range1[2], range2[2]);
	}

	void principal_components(std::vector<glm::vec3> pa, glm::mat4x3& f, glm::vec3& eimag)
	{
		assert(pa.size() > 0);
		glm::vec3 hp = glm::vec3(); for_int(i, pa.size()) { hp += pa[i]; }
		glm::vec3 avgp = hp / float(pa.size());
		principal_components2(pa, avgp, f, eimag);
	}

	void principal_components2(std::vector<glm::vec3> va, const glm::vec3& avgp, glm::mat4x3& f, glm::vec3& eimag)
	{
		// Note that this builds on version of compute_eigenvectors() specialized to n = 3.
		const int n = 3;
		glm::mat3 a;
		for_int(c0, n) {
			for_int(c1, c0 + 1) {
				double sum = 0.; for_int(i, va.size()) { sum += (va[i][c0] - avgp[c0])*(va[i][c1] - avgp[c1]); }
				a[c0][c1] = a[c1][c0] = float(sum / va.size());
			}
		}
		glm::vec3 val; for_int(i, n) { val[i] = a[i][i]; }
		glm::mat3 vec = { { 1.f, 0.f, 0.f },{ 0.f, 1.f, 0.f },{ 0.f, 0.f, 1.f } };
		for_int(iter, INT_MAX) {
			{
				float sum = 0.f; for_int(i, n - 1) for_intL(j, i + 1, n) sum += std::abs(a[i][j]);
				if (!sum) break;
			}
			for_int(i, n - 1) {
				for_intL(j, i + 1, n) {
					float thresh = 1e2f * std::abs(a[i][j]);
					if (std::abs(val[i]) + thresh == std::abs(val[i]) &&
						std::abs(val[j]) + thresh == std::abs(val[j]))
					{
						a[i][j] = 0.f;
					}
					else if (std::abs(a[i][j])>0.f)
					{
						float vtan;
						{
							float dd = val[j] - val[i];
							if (std::abs(dd) + thresh == std::abs(dd))
							{
								vtan = a[i][j] / dd;
							}
							else
							{
								float theta = 0.5f*dd / a[i][j];
								vtan = 1.f / (std::abs(theta) + sqrt(1.f + square(theta)));
								if (theta<0.f) vtan = -vtan;
							}
						}
						float vcos = 1.f / sqrt(1.f + square(vtan)), vsin = vtan*vcos;
						float tau = vsin / (1.f + vcos);
						val[i] -= vtan*a[i][j];
						val[j] += vtan*a[i][j];
						a[i][j] = 0.f;
						for_int(k, i) { rotate(a[k][i], a[k][j], tau, vsin); }
						for_intL(k, i + 1, j) { rotate(a[i][k], a[k][j], tau, vsin); }
						for_intL(k, j + 1, n) { rotate(a[i][k], a[j][k], tau, vsin); }
						for_int(k, n) { rotate(vec[i][k], vec[j][k], tau, vsin); }
					}
				}
			}
			assert(iter<10);
		}
		// Insertion sort: eigenvalues in descending order.
		for_int(i, n) {
			int imax = i;
			float vmax = val[i];
			for_intL(j, i + 1, n) { if (val[j] >= vmax) { imax = j; vmax = val[j]; } }
			if (imax == i) continue;
			std::swap(val[i], val[imax]);
			swap_ranges(vec[i], vec[imax]);
		}
		for_int(i, n) {
			float v = val[i];
			if (v<0.f) v = 0.f;     // for numerics
			v = sqrt(v);
			eimag[i] = v;
			if (!v) v = 1e-15f;     // very small but non-zero vector
			f[i] = v*vec[i];
		}
		f[3] = glm::vec3(avgp[0], avgp[1], avgp[2]);
		make_right_handed(f);
	}

	void make_right_handed(glm::mat4x3& f)
	{
		if (glm::dot( glm::cross(f[0], f[1]), f[2] ) < 0)
			f[0] = -f[0];
	}
	
} // namespace HuguesHoppe
