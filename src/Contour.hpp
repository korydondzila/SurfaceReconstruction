// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef CONTOUR_H
#define CONTOUR_H

#include "includes\includes.hpp"
//#include "Flags.hpp"
//using namespace HuguesHoppe;
#include "Mesh.hpp"
//#include "MeshOp.h"             // triangulate_face()

namespace HuguesHoppe
{

	// Contour3DMesh/Contour3D compute a piecewise linear approximation to the zeroset of a scalar function:
	//   - surface triangle mesh in the unit cube   (Contour3DMesh)
	//   - surface triangle stream in the unit cube (Contour3D)

	constexpr float k_Contour_undefined = 1e31f; // represents undefined distance, to introduce surface boundaries

	// Protected content in this class just factors functions common to Contour3DMesh, and Contour3D.
	template<typename VertexData = Vec0<int>> class ContourBase
	{
	public:
		void set_vertex_tolerance(float tol) // if nonzero, do binary search; tol is absolute distance in domain
		{
			_vertex_tol = tol;
		}
	protected:
		static constexpr float k_not_yet_evaled = BIGFLOAT;
		using DPoint = glm::vec3; // domain point
		using IPoint = glm::ivec3;   // grid point
		static constexpr int k_max_gn = 1024; // bits/coordinate==10 for 3D, 16
		ContourBase(int gn) : _gn(gn), _gni(1.f / gn)
		{
			assert(_gn > 0);
			assert(_gn < k_max_gn);  // must leave room for [0.._gn] inclusive
			set_vertex_tolerance(_vertex_tol);
		}
		~ContourBase()
		{
			assert(_queue.empty());
			if (true)
			{
				printf("March:\n");
				printf("visited %d cubes (%d were undefined, %d contained nothing)\n",
					_ncvisited, _ncundef, _ncnothing);
				printf("evaluated %d vertices (%d were zero, %d were undefined)\n",
					_nvevaled, _nvzero, _nvundef);
				printf("encountered %d tough edges\n",
					_nedegen);
			}
		}

		int _gn;
		float _gni;                 // 1.f/_gn
		float _vertex_tol{ 0.f };    // note: 0.f is special: infinite tolerance
									 // Model: the domain [0.f, 1.f]^3 is partitioned into _gn^3 cubes.
									 // These cubes are indexed by nodes with indices [0, _gn-1].
									 // The cube vertices are indexed by nodes with indices [0, _gn].  See get_point().
									 // So there are no "+.5f" roundings anywhere in the code.
		struct Node : VertexData
		{
			explicit Node(unsigned pen) : _en(pen) { }
			enum class ECubestate { nothing, queued, visited };
			unsigned _en;                                // encoded vertex index
			ECubestate _cubestate{ ECubestate::nothing }; // cube info
			float _val{ k_not_yet_evaled };               // vertex value
			DPoint _p;                                   // vertex point position in grid
														 // Note that for 3D, base class contains Vec3<Vertex> _verts.
		};

		struct hash_Node { size_t operator()(const Node& n) const { return n._en; } };
		struct equal_Node { bool operator()(const Node& n1, const Node& n2) const { return n1._en == n2._en; } };
		std::unordered_set<Node, hash_Node, equal_Node> _m;
		// (std::unordered_set<> : References and pointers to key stored in the container are only
		//   invalidated by erasing that element.  So it's OK to keep pointers to Node* even as more are added.)
		std::queue<unsigned> _queue;     // cubes queued to be visited
		int _ncvisited{ 0 };
		int _ncundef{ 0 };
		int _ncnothing{ 0 };
		int _nvevaled{ 0 };
		int _nvzero{ 0 };
		int _nvundef{ 0 };
		int _nedegen{ 0 };
		std::vector<DPoint> _tmp_poly;
		//
		// Determine if a coordinate u lies within the bounds [uL, uU] on each dimension.
		bool in_bounds(const glm::ivec3& u, const glm::ivec3& uL, const glm::ivec3& uU)
		{
			for_int(c, 3) { if (u[c]<uL[c] || u[c] >= uU[c]) return false; } return true;
		}

		// Determine if a coordinate u lies within the bounds [0, dims).
		bool in_bounds(const glm::ivec3& u, const glm::ivec3& dims)
		{
			return in_bounds(u, glm::ivec3(0), dims);
		}

		bool cube_inbounds(const IPoint& ci) { return in_bounds(ci, glm::ivec3(_gn)); }
		DPoint get_point(const IPoint& ci) const
		{
			// Note: less strict than cube_inbounds() because ci[c]==_gn is OK for a vertex.
			// DPoint dp; for_int(c, D) { assert(ci[c]>=0 && ci[c]<=_gn); dp[c] = min(ci[c]*_gni, 1.f); }
			DPoint dp; for_int(c, 3) { assert(ci[c] >= 0 && ci[c] <= _gn); dp[c] = ci[c] < _gn ? ci[c] * _gni : 1.f; }
			return dp;
		}

		DPoint interp(const DPoint& g1, const DPoint& g2, float f1 = 0.5f) {
			DPoint g = DPoint(); for_int(i, 3) { g[i] = f1*g1[i] + (1.f - f1)*g2[i]; } return g;
		}

		template<bool avoid_degen, typename Eval = float(const DPoint&)>
		DPoint compute_point(const DPoint& pp, const DPoint& pn, float vp, float vn, Eval& eval)
		{
			DPoint pm; float fm;
			if (!_vertex_tol)
			{
				fm = vp / (vp - vn);
				pm = interp(pn, pp, fm);
			}
			else
			{
				float v0 = vp, v1 = vn;
				DPoint p0 = pp, p1 = pn;
				float f0 = 0.f, f1 = 1.f;
				int neval = 0;
				for (;;)
				{
					assert(v0 >= 0.f && v1 < 0.f && f0 < f1);
					float b1 = v0 / (v0 - v1);
					b1 = clamp(b1, .05f, .95f); // guarantee quick convergence
					fm = f0*(1.f - b1) + f1*b1;
					pm = interp(p1, p0, b1);
					float vm = eval(pm);
					neval++;
					if (neval>20) break;
					if (vm<0.f)
					{
						f1 = fm; p1 = pm; v1 = vm;
					}
					else
					{
						f0 = fm; p0 = pm; v0 = vm;
					}
					if (glm::distance2(p0, p1) <= square(_vertex_tol)) break;
				}
			}
			if (avoid_degen)
			{
				// const float fs = _gn>500 ? .05f : _gn >100 ? .01f : .001f;
				const float fs = 2e-5f*_gn; // sufficient precision for HashFloat with default nignorebits==8
				if (fm<fs)
				{
					_nedegen++; pm = interp(pn, pp, fs);
				}
				else if (fm>1.f - fs)
				{
					_nedegen++; pm = interp(pp, pn, fs);
				}
			}
			return pm;
		}
	};

	// *** Contour3D

	struct VertexData3DMesh
	{
		Vec3<Vertex> _verts{ Vec3<Vertex>(Vertex(nullptr), Vertex(nullptr), Vertex(nullptr)) };
	};

	template<
		typename VertexData = Vec0<int>,
		typename Derived = void, // for contour_cube()
		typename Eval = float(const Vec3<float>&)>
		class Contour3DBase : public ContourBase<VertexData>
	{
		protected:
			using base = ContourBase<VertexData>;
			using typename base::DPoint;
			using typename base::IPoint;
			using typename base::Node;
			using base::get_point; using base::cube_inbounds;
			using base::_gn; using base::k_max_gn;
			using base::_queue; using base::_m; using base::_tmp_poly;
			using base::_ncvisited; using base::_ncundef; using base::_ncnothing;
			using base::_nvevaled; using base::_nvzero; using base::_nvundef;
			Derived& derived() { return *static_cast<Derived*>(this); }
			const Derived& derived() const { return *static_cast<const Derived*>(this); }
		public:
			Contour3DBase(int gn, Eval eval) : base(gn), _eval(eval) { }
			~Contour3DBase() { }
			// ret number of new cubes visited: 0=revisit_cube, 1=no_surf, >1=new
			int march_from(const DPoint& startp) { return march_from_i(startp); }
			// call march_from() on all cells near startp; ret num new cubes visited
			int march_near(const DPoint& startp) { return march_near_i(startp); }
		protected:
			Eval _eval;
			using Node222 = Vec2<Vec2<Vec2<Node*>>>;
			using base::k_not_yet_evaled;
			//
			unsigned encode(const IPoint& ci) const
			{
				assert(k_max_gn <= 1024);
				return (((unsigned(ci[0]) << 10) | unsigned(ci[1])) << 10) | unsigned(ci[2]);
			}

			IPoint decode(unsigned en) const
			{
				assert(k_max_gn <= 1024);
				return IPoint(int(en >> 20), int((en >> 10)&((1u << 10) - 1)), int(en&((1u << 10) - 1)));
			}

			int march_from_i(const DPoint& startp)
			{
				// Assert in unit cube, need to change
				for_int(d, 3) assert(startp[d] >= 0.f && startp[d] <= 1.f);
				IPoint cc; for_int(d, 3) { cc[d] = std::min(int(startp[d] * _gn), _gn - 1); }
				return march_from_aux(cc);
			}

			int march_near_i(const DPoint& startp)
			{
				for_int(d, 3) assert(startp[d] >= 0.f && startp[d] <= 1.f);
				IPoint cc; for_int(d, 3) { cc[d] = min(int(startp[d] * _gn), _gn - 1); }
				int ret = 0;
				IPoint ci;
				for_intL(i, -1, 2)
				{
					ci[0] = cc[0] + i; if (ci[0]<0 || ci[0] >= _gn) continue;
					for_intL(j, -1, 2)
					{
						ci[1] = cc[1] + j; if (ci[1]<0 || ci[1] >= _gn) continue;
						for_intL(k, -1, 2)
						{
							ci[2] = cc[2] + k; if (ci[2]<0 || ci[2] >= _gn) continue;
							ret += march_from_aux(ci);
						}
					}
				}
				return ret;
			}

			int march_from_aux(const IPoint& cc)
			{
				int oncvisited = _ncvisited;
				{
					unsigned en = encode(cc);
					auto p = _m.insert(Node(en));
					bool is_new = p.second;
					Node* n = const_cast<Node*>(&*p.first);
					if (n->_cubestate == base::Node::ECubestate::visited) return 0;
					bool nothing = n->_cubestate == base::Node::ECubestate::nothing;
					assert(nothing);
					_queue.push(en);
					n->_cubestate = base::Node::ECubestate::queued;
				}
				while (!_queue.empty())
				{
					unsigned en = _queue.front();
					_queue.pop();
					consider_cube(en);
				}
				int cncvisited = _ncvisited - oncvisited;
				if (cncvisited == 1) _ncnothing++;
				return cncvisited;
			}

			void consider_cube(unsigned encube)
			{
				_ncvisited++;
				IPoint cc = decode(encube);
				Node222 na;
				bool cundef = false;
				for_int(i, 2) for_int(j, 2) for_int(k, 2)
				{
					IPoint cd(i, j, k);
					IPoint ci = cc + cd;
					unsigned en = encode(ci);
					auto p = _m.insert(Node(en));
					bool is_new = p.second;
					Node* n = const_cast<Node*>(&*p.first);
					na[i][j][k] = n;
					if (n->_val == k_not_yet_evaled)
					{
						n->_p = get_point(ci);
						n->_val = _eval(n->_p);
						_nvevaled++;
						if (!n->_val) _nvzero++;
						if (n->_val == k_Contour_undefined) _nvundef++;
					}

					if (n->_val == k_Contour_undefined) cundef = true;
				}

				Node* n = na[0][0][0];
				bool queued = n->_cubestate == base::Node::ECubestate::queued;
				assert(queued);
				n->_cubestate = base::Node::ECubestate::visited;
				if (cundef)
				{
					_ncundef++;
				}
				else
				{
					derived().contour_cube(cc, na);
				}
				for_int(d, 3) for_int(i, 2) // push neighbors
				{
					int d1 = (d + 1) % 3, d2 = (d + 2) % 3;
					IPoint cd; cd[d] = i;
					float vmin = BIGFLOAT, vmax = -BIGFLOAT;
					for (cd[d1] = 0; cd[d1]<2; cd[d1]++)
					{
						for (cd[d2] = 0; cd[d2]<2; cd[d2]++)
						{
							float v = na[cd[0]][cd[1]][cd[2]]->_val;
							assert(v != k_not_yet_evaled);
							if (v<vmin) vmin = v;
							if (v>vmax) vmax = v;
						}
					}
					cd[d] = i ? 1 : -1;
					cd[d1] = cd[d2] = 0;
					IPoint ci = cc + cd;  // indices of node for neighboring cube;
										  // note: vmin<0 since 0 is arbitrarily taken to be positive
					if (vmax != k_Contour_undefined && vmin<0 && vmax >= 0 && cube_inbounds(ci))
					{
						unsigned en = encode(ci);
						auto p = _m.insert(Node(en));
						bool is_new = p.second;
						Node* n2 = const_cast<Node*>(&*p.first);
						if (n2->_cubestate == base::Node::ECubestate::nothing)
						{
							n2->_cubestate = base::Node::ECubestate::queued;
							_queue.push(en);
						}
					}
				}
			}
	};

	template<typename Eval = float(const Vec3<float>&)>
	class Contour3DMesh : public Contour3DBase<VertexData3DMesh, Contour3DMesh<Eval>, Eval>
	{
		using base = Contour3DBase<VertexData3DMesh, Contour3DMesh<Eval>, Eval>;
		public:
			Contour3DMesh(int gn, Mesh* pmesh, Eval eval = Eval())
				: base(gn, eval), _pmesh(pmesh)
			{
				assert(_pmesh);
			}
			void big_mesh_faces() { _big_mesh_faces = true; }
		private:
			// Need to friend base class for callback access to contour_cube().
			friend base;
			using typename base::IPoint;
			using typename base::Node222;
			using typename base::Node;
			using base::compute_point; using base::_eval; using base::decode;
			Mesh* _pmesh;
			bool _big_mesh_faces{ false };
			static int mod4(int j) { assert(j >= 0); return j & 0x3; }
			void contour_cube(const IPoint& cc, const Node222& na) 
			{
				// Based on Wyvill et al.
				dummy_use(cc);
				std::unordered_map<Vertex, Vertex> mapsucc;
				for_int(d, 3) for_int(v, 2) // examine each of 6 cube faces
				{
					Vec4<Node*> naf;
					{
						int d1 = (d + 1) % 3, d2 = (d + 2) % 3;
						IPoint cd; cd[d] = v;
						int i = 0;
						// Gather 4 cube vertices in a consistent order
						for (cd[d1] = 0; cd[d1]<2; cd[d1]++)
						{
							int sw = cd[d] ^ cd[d1]; // 0 or 1
							for (cd[d2] = sw; cd[d2] == 0 || cd[d2] == 1; cd[d2] += (sw ? -1 : 1))
							{
								naf[i++] = na[cd[0]][cd[1]][cd[2]];
							}
						}
					}
					int nneg = 0;
					double sumval = 0.;
					for_int(i, 4)
					{
						float val = naf[i]->_val;
						if (val<0) nneg++;
						sumval += val;  // If pedantic, could sort the vals before summing.
					}
					for_int(i, 4)
					{
						int i1 = mod4(i + 1), i2 = mod4(i + 2), i3 = mod4(i + 3);
						if (!(naf[i]->_val<0 && naf[i1]->_val >= 0)) continue;
						// have start of edge
						assert(nneg >= 1 && nneg <= 3);
						int ie;                      // end of edge
						if (nneg == 1)
						{
							ie = i3;
						}
						else if (nneg == 3)
						{
							ie = i1;
						}
						else if (naf[i2]->_val >= 0)
						{
							ie = i2;
						}
						else if (sumval<0)
						{
							ie = i1;
						}
						else
						{
							ie = i3;
						}
						Vertex v1 = get_vertex_onedge(naf[i1], naf[i]);
						Vertex v2 = get_vertex_onedge(naf[ie], naf[mod4(ie + 1)]);
						mapsucc.emplace(v2, v1);  // to get face order correct
					}
				}
				std::vector<Vertex> va = std::vector<Vertex>();
				while (!mapsucc.empty())
				{
					Vertex vf = nullptr; int minvi = INT_MAX; // find min to be portable
					std::vector<Vertex> keys(mapsucc.size());
					get_keys(mapsucc, keys);
					for (Vertex v : keys)
					{
						int vi = _pmesh->vertex_id(v);
						if (vi < minvi) { minvi = vi; vf = v; }
					}
					int nv = 0;
					for (Vertex v = vf; ; )
					{
						va[nv++] = v;
						v = mapsucc.at(v);
						assert(mapsucc.erase(v));
						if (v == vf) break;
					}
					Face f = _pmesh->create_face(std::vector<Vertex>(va));
					if (nv>3 && !_big_mesh_faces)
					{
						// If 6 or more edges, may have 2 edges on same cube face, then must introduce new vertex to be safe.
						if (nv >= 6) _pmesh->center_split_face(f);
						else assert(triangulate_face(*_pmesh, f));
					}
				}
			}

			struct hash_edge
			{
				hash_edge(const Mesh& mesh) : _mesh(mesh) { }
				size_t operator()(Edge e) const
				{
					return _mesh.vertex_id(_mesh.vertex1(e)) + size_t(_mesh.vertex_id(_mesh.vertex2(e))) * 76541;
				}
				const Mesh& _mesh;
			};
			using SetEdge = std::unordered_set<Edge, hash_edge>; // hashing does not use pointer values, for portable random.
			using EDGEF = bool(Contour3DMesh::*)(const Mesh&, Edge);

			float dihedral_angle_cos(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& po1, const glm::vec3& po2)
			{
				float ves1x, ves1y, ves1z;
				{
					float v1x = p2[0] - p1[0], v1y = p2[1] - p1[1], v1z = p2[2] - p1[2];
					float v2x = po1[0] - p1[0], v2y = po1[1] - p1[1], v2z = po1[2] - p1[2];
					ves1x = v1y*v2z - v1z*v2y; ves1y = v1z*v2x - v1x*v2z; ves1z = v1x*v2y - v1y*v2x;
					float sum2 = ves1x*ves1x + ves1y*ves1y + ves1z*ves1z;
					if (!sum2) return -2.f;
					float fac = 1.f / sqrt(sum2);
					ves1x *= fac; ves1y *= fac; ves1z *= fac;
				}
				float ves2x, ves2y, ves2z;
				{
					float v1x = po2[0] - p1[0], v1y = po2[1] - p1[1], v1z = po2[2] - p1[2];
					float v2x = p2[0] - p1[0], v2y = p2[1] - p1[1], v2z = p2[2] - p1[2];
					ves2x = v1y*v2z - v1z*v2y; ves2y = v1z*v2x - v1x*v2z; ves2z = v1x*v2y - v1y*v2x;
					float sum2 = ves2x*ves2x + ves2y*ves2y + ves2z*ves2z;
					if (!sum2) return -2.f;
					float fac = 1.f / sqrt(sum2);
					ves2x *= fac; ves2y *= fac; ves2z *= fac;
				}
				float d = ves1x*ves2x + ves1y*ves2y + ves1z*ves2z;
				if (d<-1.f) d = -1.f;
				if (d>+1.f) d = +1.f;
				return d;
			}

			int retriangulate(Mesh& mesh, SetEdge& sete, bool recurse, std::unordered_set<Vertex>* setvr,
				float mincos, EDGEF fdoswap)
			{
				assert(fdoswap);
				int neswapped = 0;
				while (!sete.empty())
				{
					Edge e = *(sete.begin()); sete.erase(e);
					assert(!mesh.is_boundary(e));
					const glm::vec3& p1 = mesh.vertex1(e)->point;
					const glm::vec3& p2 = mesh.vertex2(e)->point;
					const glm::vec3& po1 = mesh.side_vertex1(e)->point;
					const glm::vec3& po2 = mesh.side_vertex2(e)->point;
					if (dihedral_angle_cos(p1, p2, po1, po2) < mincos) continue;
					if (dihedral_angle_cos(po1, po2, p2, p1) < mincos) continue;
					if (!mesh.legal_edge_swap(e)) continue;
					if (!(this->*(fdoswap))(mesh, e)) continue;
					for (Face f : mesh.faces(e))
					{
						for (Edge ee : mesh.edges(f)) { sete.erase(ee); }
					}
					Edge ne = mesh.swap_edge(e);
					// e = nullptr; // now undefined
					neswapped++;
					if (!recurse) continue;
					for (Face f : mesh.faces(ne))
					{
						for (Edge ee : mesh.edges(f))
						{
							if (ee == ne || mesh.is_boundary(ee) ||
								(setvr && (setvr->find(mesh.side_vertex1(ee)) == setvr->end() ||
									setvr->find(mesh.side_vertex2(ee)) == setvr->end())))
								continue;
							sete.insert(ee);
						}
					}
				}
				return neswapped;
			}

			float circum_radius(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2)
			{
				double a = glm::distance(p0, p1), b = glm::distance(p1, p2), c = glm::distance(p2, p0);
				double s = (a + b + c)*.5;
				double d2 = s*(s - a)*(s - b)*(s - c);
				if (d2 <= 0.) { return 1e10f; }
				return float(a*b*c*.25 / glm::sqrt(d2));
			}

			bool circum_radius_swap_criterion(const Mesh& mesh, Edge e)
			{
				assert(!mesh.is_boundary(e));
				const glm::vec3& p1 = mesh.vertex1(e)->point;
				const glm::vec3& p2 = mesh.vertex2(e)->point;
				const glm::vec3& po1 = mesh.side_vertex1(e)->point;
				const glm::vec3& po2 = mesh.side_vertex2(e)->point;
				float rc1 = circum_radius(p1, p2, po1);
				float rc2 = circum_radius(p1, po2, p2);
				float rs1 = circum_radius(p1, po2, po1);
				float rs2 = circum_radius(p2, po1, po2);
				return std::max(rs1, rs2) < std::max(rc1, rc2);
			}

			bool triangulate_face(Mesh& mesh, Face f)
			{
				std::vector<Vertex> va; mesh.get_vertices(f, va);
				int nv = va.size();
				if (!(nv > 3)) return true;
				for_intL(i, 2, nv - 1)
				{
					if (!mesh.query_edge(va[0], va[i])) return false;
				}
				mesh.destroy_face(f);
				for_int(i, nv - 2) {
					mesh.create_face(va[0], va[i + 1], va[i + 2]);
				}
				std::unordered_set<Vertex> setvr = std::unordered_set<Vertex>(); // vertices on ring of original face
				for_int(i, nv) { setvr.insert(va[i]); }
				hash_edge he = hash_edge(mesh);
				SetEdge sete(0, he);           // initially, inner edges
				for_intL(i, 2, nv - 1) { sete.insert(mesh.edge(va[0], va[i])); }
				retriangulate(mesh, sete, true, &setvr, -2, &Contour3DMesh::circum_radius_swap_criterion);
				return true;
			}

			Vertex get_vertex_onedge(Node* n1, Node* n2)
			{
				bool is_new; Vertex* pv;
				{
					IPoint cc1 = decode(n1->_en);
					IPoint cc2 = decode(n2->_en);
					int d = -1;
					for_int(c, 3) { if (cc1[c] != cc2[c]) { assert(d < 0); d = c; } }
					assert(d >= 0);
					assert(std::abs(cc1[d] - cc2[d]) == 1);
					Node* n = (cc1[d] < cc2[d]) ? n1 : n2;
					VertexData3DMesh v;
					pv = &v._verts[d];
					//pv = &n->_verts[d];
					is_new = !*pv;
				}
				Vertex& v = *pv;
				if (is_new)
				{
					v = _pmesh->create_vertex();
					v->point = this->template compute_point<false>(n1->_p, n2->_p, n1->_val, n2->_val, _eval);
				}
				return v;
			}
	};

	template<typename Eval = float(const glm::vec3<float>&),
		typename Contour = float(const std::vector<glm::vec3>&)>
	class Contour3D : public Contour3DBase<Vec0<int>, Contour3D<Eval, Contour>, Eval>
	{
		using base = Contour3DBase<Vec0<int>, Contour3D<Eval, Contour>, Eval>;
		public:
			Contour3D(int gn, Contour contour = Contour(), Eval eval = Eval())
				: base(gn, eval), _contour(contour) {}
		private:
			// Need to friend base class for callback access to contour_cube().
			// friend base; // somehow insufficient on mingw and clang (whereas somehow sufficient in Contour3DMesh)
			template<typename, typename, typename> friend class Contour3DBase;
			Contour _contour;
			using typename base::DPoint;
			using typename base::IPoint;
			using typename base::Node222;
			using typename base::Node;
			using base::_tmp_poly; using base::_eval; using base::compute_point;
			void contour_cube(const IPoint& cc, const Node222& na)
			{
				dummy_use(cc);
				// do Kuhn 6-to-1 triangulation of cube
				contour_tetrahedron(Vec4<Node*>(na[0][0][0], na[0][0][1], na[1][0][1], na[0][1][0]));
				contour_tetrahedron(Vec4<Node*>(na[0][0][0], na[1][0][1], na[1][0][0], na[0][1][0]));
				contour_tetrahedron(Vec4<Node*>(na[1][0][1], na[1][1][0], na[1][0][0], na[0][1][0]));
				contour_tetrahedron(Vec4<Node*>(na[0][1][0], na[0][1][1], na[0][0][1], na[1][0][1]));
				contour_tetrahedron(Vec4<Node*>(na[1][1][1], na[0][1][1], na[0][1][0], na[1][0][1]));
				contour_tetrahedron(Vec4<Node*>(na[1][1][1], na[0][1][0], na[1][1][0], na[1][0][1]));
			}
			void contour_tetrahedron(Vec4<Node*> n4)
			{
				int nposi = 0; for_int(i, 4) { if (n4[i]->_val >= 0) nposi++; }
				if (nposi == 0 || nposi == 4) return;
				for (int i = 0, j = 3; i<j; )
				{
					if (n4[i]->_val >= 0) { i++; continue; }
					if (n4[j]->_val<0) { --j; continue; }
					std::swap(n4[i], n4[j]);
					i++; --j;
				}
				switch (nposi)
				{
					case 1:
						output_triangle(Vec3<Vec2<Node*>>(Vec2<Node*>(n4[0], n4[1]), Vec2<Node*>(n4[0], n4[2]), Vec2<Node*>(n4[0], n4[3])));
						break;
					case 2:
						output_triangle(Vec3<Vec2<Node*>>(Vec2<Node*>(n4[0], n4[2]), Vec2<Node*>(n4[0], n4[3]), Vec2<Node*>(n4[1], n4[3])));
						output_triangle(Vec3<Vec2<Node*>>(Vec2<Node*>(n4[0], n4[2]), Vec2<Node*>(n4[1], n4[3]), Vec2<Node*>(n4[1], n4[2])));
						break;
					case 3:
						output_triangle(Vec3<Vec2<Node*>>(Vec2<Node*>(n4[0], n4[3]), Vec2<Node*>(n4[1], n4[3]), Vec2<Node*>(n4[2], n4[3])));
						break;
					default:
						assert(false);
				}
			}
			void output_triangle(const Vec3<Vec2<Node*>>& n3)
			{
				auto& poly = _tmp_poly; //poly.init(3);
				for_int(i, 3)
				{
					Node* np = n3[i][0]; Node* nn = n3[i][1];
					poly[i] = this->template compute_point<true>(np->_p, nn->_p, np->_val, nn->_val, _eval);
				}
				glm::vec3 normal = vec3Cross(poly[0], poly[1], poly[2]);
				// swap might be unnecessary if we carefully swapped above?
				if (dot(normal, n3[0][0]->_p - n3[0][1]->_p)<0.f) std::swap(poly[0], poly[1]);
				_contour = poly;
			}

			glm::vec3 vec3Cross(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3)
			{
				// return cross(p2-p1, p3-p1);
				//
				// I once thought that "double" was necessary in next 3 lines to
				//  overcome an apparent problem with poor computed surface normals.
				// However, the problem lay in the geometry.
				// Prefiltering with "Filtermesh -taubinsmooth 4" solved it.
				float p1x = p1[0], p1y = p1[1], p1z = p1[2];
				float v1x = p2[0] - p1x, v1y = p2[1] - p1y, v1z = p2[2] - p1z;
				float v2x = p3[0] - p1x, v2y = p3[1] - p1y, v2z = p3[2] - p1z;
				return glm::vec3(v1y*v2z - v1z*v2y, v1z*v2x - v1x*v2z, v1x*v2y - v1y*v2x);
			}
	};

} // namespace HuguesHoppe

#endif // CONTOUR_H
