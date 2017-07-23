// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_H
#define MESH_H

#include "includes\includes.hpp"

namespace HuguesHoppe
{

	class Random;

	// Mesh: a set of Vertices, Faces, and Edges and their topological relations.
	// Properties:
	//   (1a) - vertices appear at most once per face.
	//   (1b) - an oriented edge (2 consecutive vertices of a face) appears in at most one face; hence mesh is orientable.
	// A Mesh must always satisfy (1a) and (1b); such a mesh is called "legal".
	//
	//   (2a) - vertices are nice (contain at most 1 (possibly partial) face ring).
	//   (2b) - faces are nice: (a, b, c) implies no (a, c, b).
	// A Mesh is "nice" if it also satisfies (2a) and (2b) -- basically, if it is everywhere a 2D manifold.
	//
	//   (3a) - all faces are triangular.
	// A Mesh is a "nice triangular mesh" if in addition it satisfies (3a).
	//
	// MVertex allocates space for Point, which is used later in GMesh.
	// MVertex, MFace, MEdge, MHEdge allocate space for string, also used in GMesh.

	class Mesh : noncopyable
	{
	public:
		struct MHEdge; struct MVertex; struct MFace; using MCorner = MHEdge; struct MEdge; // private
		using HEdge = MHEdge*; using Vertex = MVertex*; using Face = MFace*; using Corner = HEdge; using Edge = MEdge*;
		friend void swap(Mesh& l, Mesh& r) noexcept;
	private:
		template<typename T> class cvalues_iterator : public std::iterator<std::forward_iterator_tag, const T>
		{
		public:
			cvalues_iterator() = default;
			cvalues_iterator(typename std::unordered_map<int, T>::const_iterator it) : _it(it) { }
			bool operator!=(const cvalues_iterator& rhs) const {
				return _it != rhs._it;
			}
			bool operator==(const cvalues_iterator& rhs) const {
				return _it == rhs._it;
			}
			const T& operator*() const { return _it->second; }
			cvalues_iterator& operator++() {
				++_it; return *this;
			}
		private:
			typename std::unordered_map<int, T>::const_iterator _it;
		};
		template<typename T> class cvalues_range
		{
		public:
			cvalues_range(const std::unordered_map<int, T>& t) : _t(t) { }
			cvalues_iterator<T> begin() const {
				return cvalues_iterator<T>(_t.begin());
			}
			cvalues_iterator<T> end() const {
				return cvalues_iterator<T>(_t.end());
			}
		private:
			const std::unordered_map<int, T>& _t;
		};
		using Vertices_range = cvalues_range<Vertex>;
		using Faces_range = cvalues_range<Face>;
		struct Edges_range; struct OrderedVertices_range; struct OrderedFaces_range;
		struct VV_range; struct VF_range; struct VE_range; struct VC_range;
		struct FV_range; struct FF_range; struct FE_range; struct FC_range;
		struct EV_range; struct EF_range;
		struct WV_range; struct WF_range; struct WE_range; struct WC_range;
	public:
		Mesh() {}
		virtual ~Mesh() {}

		// Raw manipulation functions, may lead to non-nice Meshes.
		// always legal
		Vertex create_vertex() {
			return create_vertex_private(_vertexnum);
		}
		// die if !legal_create_face()
		Face create_face(std::vector<Vertex> va) {
			return create_face_private(_facenum, va);
		}
		Face create_face(Vertex v1, Vertex v2, Vertex v3)
		{
			std::vector<Vertex> va(3);
			va[0] = v1; va[1] = v2; va[2] = v3;
			return create_face(va);
		}
		// always legal
		virtual void destroy_face(Face f);
		// Vertex
		bool is_triangle(Face f) const {
			HEdge he = herep(f); return he->next->next->next == he;
		}
											   // ccw order
		void get_vertices(Face f, std::vector<Vertex>& va) const;
		void triangle_vertices(Face f, Vec3<Vertex>& va) const;
		// move about a face
		Edge clw_edge(Face f, Edge e) const {
			return hedge_from_ef(e, f)->prev->edge;
		}
		Edge ccw_edge(Face f, Edge e) const {
			return hedge_from_ef(e, f)->next->edge;
		}
		Vertex clw_vertex(Face f, Vertex v) const {
			return get_hedge(v, f)->prev->vert;
		} // slow
		Vertex ccw_vertex(Face f, Vertex v) const {
			return get_hedge(v, f)->next->vert;
		} // slow
		Edge clw_edge(Face f, Vertex v) const {
			return get_hedge(v, f)->edge;
		}       // slow
		Edge ccw_edge(Face f, Vertex v) const {
			return get_hedge(v, f)->next->edge;
		} // slow
																					  // Edge
		bool is_boundary(Edge e) const {
			return !herep(e)->sym;
		}
		Vertex vertex1(Edge e) const {
			return herep(e)->prev->vert;
		}
		Vertex vertex2(Edge e) const {
			return herep(e)->vert;
		}
		Vertex vertex(Edge e, int i) const {
			assert(i == 0 || i == 1); return i == 0 ? vertex1(e) : vertex2(e);
		}
		Face face1(Edge e) const {
			return herep(e)->face;
		}
		Face face2(Edge e) const {
			HEdge he = herep(e); return he->sym ? he->sym->face : nullptr;
		}
		Face face(Edge e, int i) const {
			assert(i == 0 || i == 1); return i == 0 ? face1(e) : face2(e);
		}
		// i==0 or i==1; ret nullptr if i==1 && is_boundary(e)
		Vertex side_vertex1(Edge e) const {
			return opp_vertex(e, face1(e));
		} // is_triangle(face1())
		Vertex side_vertex2(Edge e) const {
			return face2(e) ? opp_vertex(e, face2(e)) : nullptr;
		}
		Vertex side_vertex(Edge e, int i) const {
			assert(i == 0 || i == 1); return !i ? side_vertex1(e) : side_vertex2(e);
		}
		Vertex opp_vertex(Edge e, Face f) const;   // is_triangle(f)
		// Corner
		Corner corner(Vertex v, Face f) const {
			return get_hedge(v, f);
		}
		Vertex corner_vertex(Corner c) const {
			return c->vert;
		}
		Face corner_face(Corner c) const {
			return c->face;
		}
		Corner ccw_corner(Corner c) const {
			return c->sym ? c->sym->prev : nullptr;
		} // around vertex
		Corner clw_corner(Corner c) const {
			return c->next->sym;
		}                    // around vertex
		Corner ccw_face_corner(Corner c) const {
			return c->next;
		}                         // around face
		Corner clw_face_corner(Corner c) const {
			return c->prev;
		}                         // around face
		Corner ccw_corner(Vertex v, Edge e) const {
			HEdge he = hedge_from_ev1(e, v); return he ? he->prev : nullptr;
		}
		Corner clw_corner(Vertex v, Edge e) const {
			return hedge_from_ev2(e, v);
		}
		Edge ccw_face_edge(Corner c) const {
			return c->next->edge;
		}
		Edge clw_face_edge(Corner c) const {
			return c->edge;
		} // (fastest)
															   // Other associations
															   // obtain edge from vertices
		Edge query_edge(Vertex v, Vertex w) const;
		Edge edge(Vertex v, Vertex w) const {
			Edge e = query_edge(v, w); assert(e != NULL); return e;
		}
		// Counting routines (fast)
		int num_vertices() const {
			return _id2vertex.size();
		}
		int num_faces() const {
			return _id2face.size();
		}
		int num_edges() const {
			return _nedges;
		}

		bool legal_edge_swap(Edge e) const;
		// die if !legal_edge_swap(e)
		// remove f1, f2, (v1, v2)
		// add 2 faces, (vo1, vo2)
		virtual Edge swap_edge(Edge e);
		// introduce one center vertex and triangulate face
		virtual Vertex center_split_face(Face f); // always legal

		// Mesh
		Vertex id_vertex(int i) const {
			return _id2vertex.at(i);
		}
		int vertex_id(Vertex v) const {
			return v->id;
		}
		Face id_face(int i) const {
			return _id2face.at(i);
		}
		int face_id(Face f) const {
			return f->id;
		}
		Vertex id_retrieve_vertex(int i) const {
			auto v = _id2vertex.find(i); if (v == _id2vertex.end()) return Vertex(); return v->second;
		}
		Face id_retrieve_face(int i) const {
			auto f = _id2face.find(i); if (f == _id2face.end()) return Face(); return f->second;
		}

									// Iterators; can crash if continued after any change in the Mesh.
									// These mesh iterators do not define an order.
		Vertices_range vertices() const {
			return cvalues_range<Vertex>(_id2vertex);
		}
		Faces_range faces() const {
			return cvalues_range<Face>(_id2face);
		}
		std::vector<Face> facesVector() const {
			std::vector<Face> faces(_id2face.size());
			get_values<int, Face>(_id2face, faces);
			return faces;
		}
		Edges_range edges() const {
			return Edges_range(*this); }
		// These vertex iterators do not specify order, and work correctly even on non-nice vertices.
		VV_range vertices(Vertex v) const {
			return VV_range(*this, v); }
		VF_range faces(Vertex v) const {
			return VF_range(*this, v); }
		VE_range edges(Vertex v) const {
			return VE_range(*this, v); }
		VC_range corners(Vertex v) const {
			return VC_range(*this, v); }
		// Face iterators all go CCW
		FV_range vertices(Face f) const {
			return FV_range(*this, f); }
		FF_range faces(Face f) const {
			return FF_range(*this, f); }
		FE_range edges(Face f) const {
			return FE_range(*this, f); }
		FC_range corners(Face f) const {
			return FC_range(*this, f); }
		// Edge iterators do not define an order.
		EV_range vertices(Edge e) const {
			return EV_range(*this, e); }
		EF_range faces(Edge e) const {
			return EF_range(*this, e); }
	private:
		friend class GMesh;
		// Mesh Iter
		class Edges_iterator : public std::iterator<std::forward_iterator_tag, Edge>
		{
		public:
			Edges_iterator(const Mesh& m, bool beg)
			{
				_vend = m.vertices().end();
				_vcur = beg ? m.vertices().begin() : _vend;
				next();
			}
			bool operator!=(const Edges_iterator& rhs) const { return _hcur != rhs._hcur || _vcur != rhs._vcur; }
			Edge operator*() const { assert(_hcur != _hend); return (*_hcur)->edge; }
			Edges_iterator& operator++() { assert(_hcur != _hend); ++_hcur; next(); return *this; }
		private:
			std::vector<HEdge>::iterator _hcur, _hend; // _hcur points at current element
			cvalues_iterator<Vertex> _vcur, _vend;                // _vcur points one vertex ahead
			void next()
			{
				auto& range = (*_vcur)->_arhe;
				_hcur = range.end(); _hend = range.end();
				for (;;)
				{
					if (_hcur != _hend)
					{
						if ((*_hcur)->edge->herep != *_hcur) { ++_hcur; continue; }
						return;
					}
					if (_vcur == _vend) break;
					range = (*_vcur)->_arhe; ++_vcur; _hcur = range.begin(); _hend = range.end();
				}
				_hcur = range.end(); _hend = range.end();    // no element found
			}
		};
		struct Edges_range
		{
			Edges_range(const Mesh& m) : _m(m) { }
			Edges_iterator begin() const {
				return Edges_iterator(_m, true);
			}
			Edges_iterator end() const {
				return Edges_iterator(_m, false);
			}
			const Mesh& _m;
		};
		// Vertex Iter
		class VV_iterator : public std::iterator<std::forward_iterator_tag, Vertex>
		{
		public:
			VV_iterator(std::vector<HEdge>::const_iterator it) : _it(it) { }
			bool operator!=(const VV_iterator& rhs) const { assert(!rhs._extrav); return _extrav || _it != rhs._it; }
			Vertex operator*() const { return _extrav ? _extrav : (*_it)->vert; }
			VV_iterator& operator++()
			{
				if (_extrav) { _extrav = nullptr; return *this; }
				if (!(*_it)->prev->sym) _extrav = (*_it)->prev->prev->vert;
				++_it;
				return *this;
			}
		private:
			std::vector<HEdge>::const_iterator _it;
			Vertex _extrav{ nullptr };
		};
		struct VV_range
		{
			VV_range(const Mesh&, Vertex v) : _ar(v->_arhe) { }
			VV_iterator begin() const {
				return VV_iterator(_ar.begin());
			}
			VV_iterator end() const {
				return VV_iterator(_ar.end());
			}
			std::vector<HEdge> _ar;
		};
		class VF_iterator : public std::iterator<std::forward_iterator_tag, Face>
		{
		public:
			VF_iterator(std::vector<HEdge>::const_iterator it) : _it(it) { }
			bool operator!=(const VF_iterator& rhs) const { return _it != rhs._it; }
			Face operator*() const { return (*_it)->face; }
			VF_iterator& operator++() { ++_it; return *this; }
		private:
			std::vector<HEdge>::const_iterator _it;
		};
		struct VF_range
		{
			VF_range(const Mesh&, Vertex v) : _ar(v->_arhe) { }
			VF_iterator begin() const { return VF_iterator(_ar.begin()); }
			VF_iterator end() const { return VF_iterator(_ar.end()); }
			std::vector<HEdge> _ar;
		};
		class VE_iterator : public std::iterator<std::forward_iterator_tag, Edge>
		{
		public:
			VE_iterator(std::vector<HEdge>::const_iterator it) : _it(it) { }
			bool operator!=(const VE_iterator& rhs) const { assert(!rhs._extrae); return _extrae || _it != rhs._it; }
			Edge operator*() const { return _extrae ? _extrae : (*_it)->edge; }
			VE_iterator& operator++()
			{
				if (_extrae) { _extrae = nullptr; return *this; }
				if (!(*_it)->prev->sym) _extrae = (*_it)->prev->edge;
				++_it;
				return *this;
			}
		private:
			std::vector<HEdge>::const_iterator _it;
			Edge _extrae{ nullptr };
		};
		struct VE_range
		{
			VE_range(const Mesh&, Vertex v) : _ar(v->_arhe) { }
			VE_iterator begin() const { return VE_iterator(_ar.begin()); }
			VE_iterator end() const { return VE_iterator(_ar.end()); }
			std::vector<HEdge> _ar;
		};
		class VC_iterator : public std::iterator<std::forward_iterator_tag, Corner>
		{
		public:
			VC_iterator(std::vector<HEdge>::const_iterator it) : _it(it) { }
			bool operator!=(const VC_iterator& rhs) const { return _it != rhs._it; }
			Corner operator*() const { return (*_it)->prev; }
			VC_iterator& operator++() { ++_it; return *this; }
		private:
			std::vector<HEdge>::const_iterator _it;
		};
		struct VC_range
		{
			VC_range(const Mesh&, Vertex v) : _ar(v->_arhe) { }
			VC_iterator begin() const { return VC_iterator(_ar.begin()); }
			VC_iterator end() const { return VC_iterator(_ar.end()); }
			std::vector<HEdge> _ar;
		};
		// Face Iter
		class FV_iterator : public std::iterator<std::forward_iterator_tag, Vertex>
		{
		public:
			FV_iterator(HEdge he, bool beg) : _it(he), _beg(beg) { }
			bool operator!=(const FV_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Vertex operator*() const { return _it->vert; }
			FV_iterator& operator++() { _beg = false; _it = _it->next; return *this; }
		private:
			HEdge _it;
			bool _beg;
		};
		struct FV_range
		{
			FV_range(const Mesh& m, Face f) : _herep(m.herep(f)) { }
			FV_iterator begin() const { return FV_iterator(_herep, true); }
			FV_iterator end() const { return FV_iterator(_herep, false); }
			HEdge _herep;
		};
		class FF_iterator : public std::iterator<std::forward_iterator_tag, Face>
		{
		public:
			FF_iterator(HEdge he, bool beg) : _it(he), _beg(beg) {
				for (;;) {
					if (_it->sym) break;
					_it = _it->next;
					if (_it == he) { _beg = false; break; }
				}
			}
			bool operator!=(const FF_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Face operator*() const { return _it->sym->face; }
			FF_iterator& operator++()
			{
				HEdge bu = _beg ? nullptr : _it;
				_beg = false;
				for (;;)
				{
					_it = _it->next;
					if (_it->sym) break;
					assert(_it != bu);
				}
				return *this;
			}
		private:
			HEdge _it;
			bool _beg;
		};
		struct FF_range
		{
			FF_range(const Mesh& m, Face f) : _herep(m.herep(f)) { }
			FF_iterator begin() const { return FF_iterator(_herep, true); }
			FF_iterator end() const { return FF_iterator(_herep, false); }
			HEdge _herep;
		};
		class FE_iterator : public std::iterator<std::forward_iterator_tag, Edge>
		{
		public:
			FE_iterator(HEdge he, bool beg) : _it(he), _beg(beg) { }
			bool operator!=(const FE_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Edge operator*() const { return _it->edge; }
			FE_iterator& operator++() { _beg = false; _it = _it->next; return *this; }
		private:
			HEdge _it;
			bool _beg;
		};
		struct FE_range
		{
			FE_range(const Mesh& m, Face f) : _herep(m.herep(f)) { }
			FE_iterator begin() const { return FE_iterator(_herep, true); }
			FE_iterator end() const { return FE_iterator(_herep, false); }
			HEdge _herep;
		};
		class FC_iterator : public std::iterator<std::forward_iterator_tag, Corner> 
		{
		public:
			FC_iterator(HEdge he, bool beg) : _it(he), _beg(beg) { }
			bool operator!=(const FC_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Corner operator*() const { return _it; }
			FC_iterator& operator++() { _beg = false; _it = _it->next; return *this; }
		private:
			HEdge _it;
			bool _beg;
		};
		struct FC_range
		{
			FC_range(const Mesh& m, Face f) : _herep(m.herep(f)) { }
			FC_iterator begin() const { return FC_iterator(_herep, true); }
			FC_iterator end() const { return FC_iterator(_herep, false); }
			HEdge _herep;
		};
		// Edge Iter
		struct EV_range : Vec2<Vertex> // always 2 vertices
		{
			EV_range(const Mesh& m, Edge e)
			{
				HEdge he = m.herep(e);
				(*this)[0] = he->vert;
				(*this)[1] = he->prev->vert;
			}
		};
		struct EF_range : std::vector<Face> // 0 or 1 face
		{
			EF_range(const Mesh& m, Edge e)
			{
				HEdge he = m.herep(e);
				push_back(he->face);
				if (he->sym) push_back(he->sym->face);
			}
		};
	public:                        // should be private but uses Pool
		struct MEdge
		{
			HEdge herep;
			std::unique_ptr<char[]> _string;
			MEdge(HEdge pherep) : herep(pherep) { }
		};
		struct MVertex
		{
			std::vector<HEdge> _arhe;  // hedges he such that he->prev->vert==this
			int id;
			std::unique_ptr<char[]> _string;
			glm::vec3 point;
			MVertex(int pid) : id(pid) { }
		};
		struct MFace
		{
			HEdge herep;
			int id;
			std::unique_ptr<char[]> _string;
			MFace(int pid) : id(pid) { }
		};
		struct MHEdge
		{
			HEdge prev;             // previous HEdge in ring around face
			HEdge next;             // next HEdge in ring around face
			HEdge sym;              // pointer to symmetric HEdge (or 0)
			Vertex vert;            // Vertex to which this HEdge is pointing
			Face face;              // Face on which this HEdge belongs
			Edge edge;              // Edge to which this HEdge belongs
			std::unique_ptr<char[]> _string;
			MHEdge() = default;
		};
	public:                                                       // Discouraged:
		virtual Vertex create_vertex_private(int id);              // die if id is already used
		virtual Face create_face_private(int id, std::vector<Vertex> va); // die if id is already used
	protected:
		static const int sdebug = 0;    // 0=no, 1=min, 2=max
	private:
		std::unordered_map<int, Vertex> _id2vertex; // also acts as set of vertices
		std::unordered_map<int, Face> _id2face;     // also acts as set of faces
		int _vertexnum{ 1 };         // id to assign to next new vertex
		int _facenum{ 1 };           // id to assign to next new face
		int _nedges{ 0 };
		HEdge clw_hedge(HEdge he) const {
			return he->next->sym;
		} // may return nullptr
		HEdge ccw_hedge(HEdge he) const {
			return he->sym ? he->sym->prev : nullptr;
		}
		HEdge herep(Vertex v) const {
			return !v->_arhe.size() ? nullptr : v->_arhe[0]->prev;
		}
		HEdge herep(Face f) const {
			return f->herep;
		}
		HEdge herep(Edge e) const {
			return e->herep;
		}
		bool is_boundary(HEdge he) const {
			return !he->sym;
		}
		HEdge hedge_from_ev1(Edge e, Vertex v) const { // may return nullptr
			if (vertex1(e) == v) return herep(e);
			if (vertex2(e) == v) return herep(e)->sym;
			assert(false);
		}
		HEdge hedge_from_ev2(Edge e, Vertex v) const { // may return nullptr
			if (vertex1(e) == v) return herep(e)->sym;
			if (vertex2(e) == v) return herep(e);
			assert(false);
		}
		HEdge hedge_from_ef(Edge e, Face f) const { // may return nullptr
			if (face1(e) == f) return herep(e);
			if (face2(e) == f) return herep(e)->sym;
			assert(false);
		}
		HEdge get_hedge(Vertex v, Face f) const;      // slow; on f pointing to v
		HEdge query_hedge(Vertex v1, Vertex v2) const;
		void enter_hedge(HEdge he, Vertex v1);
		void remove_hedge(HEdge he, Vertex v1);
		void create_bogus_hedges(std::vector<HEdge>* ar_he);
		void remove_bogus_hedges(std::vector<HEdge>* ar_he);
	};

	using Vertex = Mesh::Vertex;
	using Face = Mesh::Face;
	using Corner = Mesh::Corner;
	using Edge = Mesh::Edge;
	using HEdge = Mesh::HEdge;

	inline void swap(Mesh& l, Mesh& r) noexcept
	{
		using std::swap; swap(l._id2vertex, r._id2vertex); swap(l._id2face, r._id2face);
		swap(l._vertexnum, r._vertexnum); swap(l._facenum, r._facenum); swap(l._nedges, r._nedges);
	}

	inline void Mesh::triangle_vertices(Face f, Vec3<Vertex>& va) const
	{
		HEdge he = herep(f), he0 = he;
		va[0] = he->vert; he = he->next;
		va[1] = he->vert; he = he->next;
		va[2] = he->vert; he = he->next;
		assert(he == he0);           // is_triangle()
	}

} // namespace HuguesHoppe

#endif // MESH_H
