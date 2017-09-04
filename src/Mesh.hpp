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
	private:
		template<typename T> class cvalues_iterator : public std::iterator<std::forward_iterator_tag, const T>
		{
		public:
			cvalues_iterator() = default;
			cvalues_iterator(typename std::unordered_map<int, T>::const_iterator it) : _it(it) { }
		private:
			typename std::unordered_map<int, T>::const_iterator _it;
		};
		template<typename T> class cvalues_range
		{
		public:
			cvalues_range(const std::unordered_map<int, T>& t) : _t(t) { }
		private:
			const std::unordered_map<int, T>& _t;
		};
		using Vertices_range = cvalues_range<Vertex>;
		using Faces_range = cvalues_range<Face>;
		struct Edges_range;
		struct FV_range; struct FE_range; struct FC_range;
		struct EF_range;
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
											   // ccw order
		void get_vertices(Face f, std::vector<Vertex>& va) const;
		void triangle_vertices(Face f, Vec3<Vertex>& va) const;
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
		Face face1(Edge e) const {
			return herep(e)->face;
		}
		Face face2(Edge e) const {
			HEdge he = herep(e); return he->sym ? he->sym->face : nullptr;
		}
		// i==0 or i==1; ret nullptr if i==1 && is_boundary(e)
		Vertex side_vertex1(Edge e) const {
			return opp_vertex(e, face1(e));
		} // is_triangle(face1())
		Vertex side_vertex2(Edge e) const {
			return face2(e) ? opp_vertex(e, face2(e)) : nullptr;
		}
		Vertex opp_vertex(Edge e, Face f) const;   // is_triangle(f)
															   // Other associations
															   // obtain edge from vertices
		Edge query_edge(Vertex v, Vertex w) const;
		Edge edge(Vertex v, Vertex w) const {
			Edge e = query_edge(v, w); assert(e != NULL); return e;
		}

		bool legal_edge_swap(Edge e) const;
		// die if !legal_edge_swap(e)
		// remove f1, f2, (v1, v2)
		// add 2 faces, (vo1, vo2)
		virtual Edge swap_edge(Edge e);
		// introduce one center vertex and triangulate face
		virtual Vertex center_split_face(Face f); // always legal

		// Mesh
		int vertex_id(Vertex v) const {
			return v->id;
		}

									// Iterators; can crash if continued after any change in the Mesh.
									// These mesh iterators do not define an order.
		std::vector<Face> facesVector() const {
			std::vector<Face> faces(_id2face.size());
			get_values<int, Face>(_id2face, faces);
			return faces;
		}
		// Face iterators all go CCW
		FV_range vertices(Face f) const {
			return FV_range(*this, f); }
		FE_range edges(Face f) const {
			return FE_range(*this, f); }
		FC_range corners(Face f) const {
			return FC_range(*this, f); }
		// Edge iterators do not define an order.
		EF_range faces(Edge e) const {
			return EF_range(*this, e); }
	private:
		// Face Iter
		class FV_iterator : public std::iterator<std::forward_iterator_tag, Vertex>
		{
		public:
			FV_iterator(HEdge he, bool beg) : _it(he), _beg(beg) { }
			bool operator!=(const FV_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Vertex operator*() const { return _it->vert; }
			FV_iterator& operator++()
			{
				_beg = false; _it = _it->next; return *this;
			}
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
		class FE_iterator : public std::iterator<std::forward_iterator_tag, Edge>
		{
		public:
			FE_iterator(HEdge he, bool beg) : _it(he), _beg(beg) { }
			bool operator!=(const FE_iterator& rhs) const { assert(!rhs._beg); return _beg || _it != rhs._it; }
			Edge operator*() const { return _it->edge; }
			FE_iterator& operator++()
			{
				_beg = false; _it = _it->next; return *this;
			}
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
			FC_iterator& operator++()
			{
				_beg = false; _it = _it->next; return *this;
			}
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
		HEdge herep(Face f) const {
			return f->herep;
		}
		HEdge herep(Edge e) const {
			return e->herep;
		}
		bool is_boundary(HEdge he) const {
			return !he->sym;
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
} // namespace HuguesHoppe

#endif // MESH_H
