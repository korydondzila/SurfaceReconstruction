// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "includes\includes.hpp"
#include "Mesh.hpp"

namespace HuguesHoppe
{

	//HH_ALLOCATE_POOL(Mesh::MVertex);
	//HH_ALLOCATE_POOL(Mesh::MFace);
	//HH_ALLOCATE_POOL(Mesh::MEdge);
	//HH_ALLOCATE_POOL(Mesh::MHEdge);

	//HH_SAC_INITIALIZATION(Mesh::MVertex);
	//HH_SAC_INITIALIZATION(Mesh::MFace);
	//HH_SAC_INITIALIZATION(Mesh::MEdge);
	//HH_SAC_INITIALIZATION(Mesh::MHEdge);

	// *** MISC

	// Analysis of memory requirement:
	//
	//  MVertex: (7+6*2)*4  *1v/v   == 76 bytes/vertex
	//  MFace:   4*4        *2f/v   == 32 bytes/vertex
	//  MEdge:   3*4        *3e/v   == 36 bytes/vertex
	//  MHEdge:  7*4        *6h/v   == 168 bytes/vertex
	//  id2vertex: 16       *1v/v   == 16 bytes/vertex
	//  id2face:   16       *2f/v   == 32 bytes/vertex
	//
	//  Total:                      == 360 bytes/vertex (+string_info{v,f,e,w})
	//
	//  example: mojave_sq600 (360K vertices): 130 MBytes
	//
	// (Compare with WMesh: 68 bytes/vertex;  AWMesh:  92 bytes/vertex
	//  no attribs:  WMesh: 40 bytes/vertex;  AWMesh:  64 bytes/vertex)

	// *** DRAWING

	// Note: changes to mesh go through enter_hedge() and remove_hedge() !

	/*
	//            v2
	//       /    ^    \
	//   vo1 f1 he|  f2  vo2
	//       \    |    /
	//            v1            if (is_boundary(he)), f2==0, vo2==0
	*/

	//  Vertex v1 -> stack of HEdges (v1, ?)

	// *** Mesh

	Mesh::Mesh()
	{
	}

	void Mesh::clear()
	{
		if (sdebug >= 1) ok();
		while (num_faces())
		{
			destroy_face(_id2face.begin()->second);
		}
		while (num_vertices())
		{
			destroy_vertex(_id2vertex.begin()->second);
		}
		_vertexnum = 1;
		_facenum = 1;
		_nedges = 0;
	}

	void Mesh::copy(const Mesh& m)
	{
		clear();
		_flags = m._flags;
		for (Vertex v : m.vertices())
		{
			Vertex vn = create_vertex_private(m.vertex_id(v));
			vn->_flags = v->_flags;
		}
		std::vector<Vertex> va;
		for (Face f : m.faces())
		{
			m.get_vertices(f, va);
			for_int(i, va.size()) { va[i] = id_vertex(m.vertex_id(va[i])); }
			Face fn = create_face_private(m.face_id(f), va);
			fn->_flags = f->_flags;
		}
		for (Edge e : m.edges())
		{
			if (!e->_flags) continue;
			Edge en = ordered_edge(id_vertex(m.vertex_id(m.vertex1(e))), id_vertex(m.vertex_id(m.vertex2(e))));
			en->_flags = e->_flags;
		}
	}

	// *** Raw manipulation

	Vertex Mesh::create_vertex_private(int id)
	{
		assert(id >= 1);
		Vertex v = new MVertex(id);
		// v->point undefined
		_id2vertex.emplace(id, v);
		_vertexnum = std::max(_vertexnum, id + 1);
		return v;
	}

	void Mesh::destroy_vertex(Vertex v)
	{
		assert(!herep(v));
		assert(_id2vertex.erase(v->id));
		if (_vertexnum - 1 == v->id) --_vertexnum;
		delete v;
	}

	bool Mesh::legal_create_face(std::vector<Vertex> va) const
	{
		assert(va.size() >= 3);
		if (sdebug >= 1) { for (Vertex v : va) valid(v); }
		if (va.size() == 3) // cheap check
		{
			if (va[0] == va[1] || va[1] == va[2] || va[0] == va[2]) return false;
		}
		else
		{
			std::unordered_set<Vertex> setv;
			for (Vertex v : va) { if (!(setv.insert(v).second)) return false; }
		}
		Vertex vo = va[va.size() - 1];
		for_int(i, va.size())
		{
			if (query_hedge(vo, va[i])) return false;
			vo = va[i];
		}
		return true;
	}

	Face Mesh::create_face_private(int id, std::vector<Vertex> va)
	{
		assert(id >= 1);
		assert(va.size() >= 3);
		if (sdebug >= 1) assert(legal_create_face(va));
		Face f = new MFace(id);
		// f->herep defined below
		_id2face.emplace(id, f);
		HEdge hep = nullptr;
		int nv = va.size();
		for_int(i, nv)
		{
			Vertex v2 = va[i + 1 == nv ? 0 : i + 1];
			HEdge he = new MHEdge;
			he->prev = hep;
			// he->next is set below
			he->vert = v2;
			enter_hedge(he, va[i]);
			// he->sym and he->edge were set in enter_hedge
			he->face = f;
			hep = he;
		}
		assert(hep);               // HH_ASSUME(hep);
		HEdge helast = hep;
		for (;;)
		{
			HEdge hepp = hep->prev;
			if (!hepp) break;
			hepp->next = hep;
			hep = hepp;
		}
		hep->prev = helast;
		helast->next = hep;
		f->herep = helast;          // such that f->herep->vert==va[0]
		_facenum = std::max(_facenum, id + 1);
		if (sdebug >= 3) ok();
		return f;
	}

	void Mesh::destroy_face(Face f)
	{
		{
			HEdge he = herep(f); 
			assert(he); 
			HEdge hef = he;
			Vertex v1 = he->prev->vert;
			for (;;)
			{
				HEdge hen = he->next;
				Vertex v1n = he->vert;
				remove_hedge(he, v1);
				delete he; he = hen;
				v1 = v1n;
				if (he == hef) break;
			}
		}
		assert(_id2face.erase(f->id));
		if (_facenum - 1 == f->id) --_facenum;
		delete f;
		if (sdebug >= 3) ok();
	}

	// *** Vertex

	bool Mesh::is_nice(Vertex v) const
	{
		// return num_boundaries(v)<2;  // ignores 2 interior vertices touching
		HEdge her = herep(v);
		if (!her) return true;
		int nhe = 0;
		HEdge he = her;
		for (;;)
		{
			nhe++;
			he = clw_hedge(he);
			if (!he || he == her) break;
		}
		if (he != her) { for (he = her; (he = ccw_hedge(he)) != nullptr; ) nhe++; }
		return nhe == v->_arhe.size();
	}

	int Mesh::degree(Vertex v) const
	{
		int n = 0; for (HEdge he : v->_arhe) { n += is_boundary(he) ? 2 : 1; }
		return n;
	}

	int Mesh::num_boundaries(Vertex v) const
	{
		int n = 0; for (HEdge he : v->_arhe) { if (is_boundary(he)) n++; }
		return n;
	}

	bool Mesh::is_boundary(Vertex v) const
	{
		if (!herep(v))
			assert(true);
		int nb = num_boundaries(v);
		if (nb >= 2)                  // !is_nice(v)
			assert(true);
		return nb>0;
	}

	Edge Mesh::opp_edge(Vertex v, Face f) const
	{
		assert(is_triangle(f));
		return get_hedge(v, f)->prev->edge;
	}

	Vertex Mesh::opp_vertex(Vertex v, Edge e) const
	{
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		if (v == v1) return v2;
		if (v == v2) return v1;
		assert(true);
	}

	Vertex Mesh::most_clw_vertex(Vertex v) const
	{
		HEdge he = most_clw_hedge(v);
		return he ? he->next->vert : nullptr;
	}

	Vertex Mesh::most_ccw_vertex(Vertex v) const
	{
		HEdge he = most_ccw_hedge(v);
		return he ? he->prev->vert : nullptr;
	}

	Vertex Mesh::clw_vertex(Vertex v, Vertex vext) const
	{
		HEdge he = query_hedge(vext, v); if (he) return he->next->vert;
		if (sdebug >= 1) assert(query_hedge(v, vext));
		return nullptr;
	}

	Vertex Mesh::ccw_vertex(Vertex v, Vertex vext) const
	{
		HEdge he = query_hedge(v, vext); if (he) return he->prev->prev->vert;
		if (sdebug >= 1) assert(query_hedge(vext, v));
		return nullptr;
	}

	Face Mesh::most_clw_face(Vertex v) const
	{
		HEdge he = most_clw_hedge(v);
		return he ? he->face : nullptr;
	}

	Face Mesh::most_ccw_face(Vertex v) const
	{
		HEdge he = most_ccw_hedge(v);
		return he ? he->face : nullptr;
	}

	Face Mesh::clw_face(Vertex v, Face f) const
	{
		return opp_face(f, ccw_edge(f, v));
	}

	Face Mesh::ccw_face(Vertex v, Face f) const
	{
		return opp_face(f, clw_edge(f, v));
	}

	Edge Mesh::most_clw_edge(Vertex v) const
	{
		HEdge he = most_clw_hedge(v);
		return he ? he->next->edge : nullptr;
	}

	Edge Mesh::most_ccw_edge(Vertex v) const
	{
		HEdge he = most_ccw_hedge(v);
		return he ? he->edge : nullptr;
	}

	Edge Mesh::clw_edge(Vertex v, Edge e) const
	{
		HEdge he = hedge_from_ev2(e, v);
		return he ? he->next->edge : nullptr;
	}

	Edge Mesh::ccw_edge(Vertex v, Edge e) const
	{
		HEdge he = hedge_from_ev1(e, v);
		return he ? he->prev->edge : nullptr;
	}

	Face Mesh::ccw_face(Vertex v, Edge e) const
	{
		if (vertex1(e) == v) return face1(e);
		if (vertex2(e) == v) return face2(e);
		assert(true);
	}

	Face Mesh::clw_face(Vertex v, Edge e) const
	{
		if (vertex1(e) == v) return face2(e);
		if (vertex2(e) == v) return face1(e);
		assert(true);
	}

	// *** Face

	bool Mesh::is_nice(Face f) const
	{
		// non-nice iff all its neighbors are the same
		Face fp = nullptr; int n = 0;
		for (Face ff : faces(f))
		{
			if (!n++)
			{
				fp = ff;
			}
			else if (ff != fp)
			{
				fp = nullptr; break;
			}
		}
		return n <= 1 || !fp;
	}

	int Mesh::num_vertices(Face f) const
	{
		int n = 0; for (HEdge he : corners(f)) { dummy_use(he); n++; }
		return n;
	}

	bool Mesh::is_boundary(Face f) const
	{
		for (Vertex v : vertices(f)) { if (is_boundary(v)) return true; }
		return false;
	}

	Face Mesh::opp_face(Face f, Edge e) const
	{
		Face f1 = face1(e), f2 = face2(e);
		if (f == f1) return f2;
		if (f == f2) return f1;
		assert(true);
	}

	Face Mesh::opp_face(Vertex v, Face f) const
	{
		return opp_face(f, opp_edge(v, f));
	}

	void Mesh::get_vertices(Face f, std::vector<Vertex>& va) const
	{
		va.clear(); for (Vertex v : vertices(f)) { va.push_back(v); }
	}

	std::vector<Corner> Mesh::get_corners(Face f, std::vector<Corner>&& ca) const
	{
		ca.clear(); for (Corner c : corners(f)) { ca.push_back(c); }
		return std::move(ca);
	}

	// *** Edge

	Vertex Mesh::opp_vertex(Edge e, Face f) const
	{
		HEdge he = hedge_from_ef(e, f);
		if (he->next->next->next != he) assert(true); // cheaper than is_triangle(f)
		return he->next->vert;
	}

	Edge Mesh::opp_boundary(Edge e, Vertex v) const
	{
		HEdge he = herep(e);
		assert(is_boundary(he));
		if (he->vert == v)
		{
			while (HEdge hen = clw_hedge(he)) he = hen;
			return he->next->edge;
		}
		if (he->prev->vert == v)
		{
			he = he->prev;
			while (HEdge hen = ccw_hedge(he)) he = hen;
			return he->edge;
		}
		assert(true);
	}

	Vertex Mesh::vertex_between_edges(Edge e1, Edge e2)
	{
		if (vertex1(e1) == vertex1(e2) || vertex1(e1) == vertex2(e2)) return vertex1(e1);
		if (vertex2(e1) == vertex1(e2) || vertex2(e1) == vertex2(e2)) return vertex2(e1);
		assert(true);
	}

	// *** Other associations

	Edge Mesh::query_edge(Vertex v, Vertex w) const
	{
		HEdge he;
		he = query_hedge(v, w); if (he) return he->edge;
		he = query_hedge(w, v); if (he) return he->edge;
		return nullptr;
	}

	Edge Mesh::ordered_edge(Vertex v1, Vertex v2) const
	{
		HEdge he = query_hedge(v1, v2);
		assert(he);
		Edge e = he->edge;
		assert(e->herep == he);
		return e;
	}

	// *** Counting routines

	/*Vertex Mesh::random_vertex(Random& r) const
	{
		assert(num_vertices());
		return _id2vertex.get_random_value(r);
	}

	Face Mesh::random_face(Random& r) const
	{
		assert(num_faces());
		return _id2face.get_random_value(r);
	}

	Edge Mesh::random_edge(Random& r) const
	{
		Face f = random_face(r);
		int vi = r.get_unsigned(num_vertices(f));
		HEdge hef = nullptr;
		for (HEdge he : corners(f))
		{
			if (!vi--) { hef = he; break; }
		}
		assert(hef);
		return hef->edge;
	}*/

	// *** Mesh operations

	bool Mesh::legal_edge_collapse(Edge e) const
	{
		if (sdebug >= 1) valid(e);
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e); // vo2 may be nullptr
															 // Check that substituting v2 to v1 will not duplicate an edge in any
															 // faces adjacent to v2 (besides f1 and f2).
															 // (case of Face vertices being duplicated cannot happen here
															 //  since only f1 and f2 can have both v1 and v2)
		for (Vertex v : vertices(v2))
		{
			if (v == v1 || v == vo1 || v == vo2) continue;
			if (query_edge(v, v1)) return false;
		}
		return true;
	}

	bool Mesh::nice_edge_collapse(Edge e) const
	{
		if (sdebug >= 1) valid(e);
		if (!legal_edge_collapse(e)) return false;
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);
		assert(is_triangle(f1));
		if (f2) assert(is_triangle(f2));
		if (sdebug >= 1) assert(is_nice(v1) && is_nice(v2));
		// Requirements:
		// * 1 - If v1 and v2 are both boundary, (v1, v2) is a boundary edge
		if (!is_boundary(e) && is_boundary(v1) && is_boundary(v2)) return false;
		// * 2 - For all vertices adjacent to both v1 and v2, exists a face
		Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);
		std::unordered_set<Vertex> set;
		for (Vertex v : vertices(v1))
		{
			if (v != vo1 && v != vo2) set.emplace(v);
		}
		for (Vertex v : vertices(v2))
		{
			if (v != vo1 && v != vo2 && !set.insert(v).second) return false;
		}
		// * 3 - two small base cases: single face and tetrahedron
		if (set.size() == 2 && is_boundary(e))
			return false;           // single face
		if (set.size() == 2 && !is_boundary(v1) && !is_boundary(v2))
			return false;           // tetrahedron
		return true;
	}

	bool Mesh::legal_edge_swap(Edge e) const
	{
		if (sdebug >= 1) valid(e);
		if (is_boundary(e)) return false;
		// illegal if cross edge already exists (as in tetrahedron)
		if (query_edge(side_vertex1(e), side_vertex2(e))) return false;
		return true;
	}

	void Mesh::collapse_edge_vertex(Edge e, Vertex vs)
	{
		assert(legal_edge_collapse(e));
		HEdge he1 = hedge_from_ev1(e, vs);
		HEdge he2 = hedge_from_ev2(e, vs);
		assert(he2);
		Vertex vt = he1 ? he1->vert : he2->prev->vert;
		assert(vt == opp_vertex(vs, e)); // optional
										  // Create bogus hedges if boundaries
		std::vector<HEdge> ar_he;
		if (he1) ar_he.push_back(he1->prev);
		if (he2) ar_he.push_back(he2->next);
		create_bogus_hedges(ar_he);
		if (he1)
		{
			assert(is_triangle(he1->face));
			destroy_face(he1->face);
		}
		if (he2)
		{
			assert(is_triangle(he2->face));
			destroy_face(he2->face);
		}
		// Change remaining faces around vt to have vs instead
		std::vector<Corner> arc; for (HEdge he : corners(vt)) { arc.push_back(he); }
		for (HEdge he : arc)
		{
			// ends up deleting and recreating MEdge structures, which is great.
			remove_hedge(he, he->prev->vert); remove_hedge(he->next, he->vert);
			he->vert = vs;
			enter_hedge(he, he->prev->vert); enter_hedge(he->next, he->vert);
		}
		// Destroy vertex vt
		destroy_vertex(vt);
		remove_bogus_hedges(ar_he);
	}

	void Mesh::collapse_edge(Edge e)
	{
		collapse_edge_vertex(e, vertex1(e));
	}

	Vertex Mesh::split_edge(Edge e, int id)
	{
		if (sdebug >= 1) valid(e);
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);                   // f2 could be nullptr
		Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e); // implies triangles
															 // Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (Face f : faces(e))
		{
			for (HEdge he : corners(f)) { if (he->edge != e) ar_he.push_back(he); }
		} create_bogus_hedges(ar_he); // note: temporarily causes mesh.ok() to fail
									  // Destroy faces
		destroy_face(f1);
		if (f2) destroy_face(f2);
		// Create new vertex
		Vertex vn = id ? create_vertex_private(id) : create_vertex();
		// Create new faces
		create_face(vn, v2, vo1); create_face(vn, vo1, v1);
		if (vo2) { create_face(vn, v1, vo2); create_face(vn, vo2, v2); }
		remove_bogus_hedges(ar_he);
		return vn;
	}

	Edge Mesh::swap_edge(Edge e)
	{
		assert(legal_edge_swap(e));
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);
		Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e); // implies triangles
															 // Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (Face f : faces(e))
		{
			for (HEdge he : corners(f)) { ar_he.push_back(he); }
		} create_bogus_hedges(ar_he);
		// Destroy faces
		destroy_face(f1);
		destroy_face(f2);
		// Create new faces; may die if illegal
		create_face(v1, vo2, vo1);
		create_face(v2, vo1, vo2);
		remove_bogus_hedges(ar_he);
		return edge(vo1, vo2);
	}

	Vertex Mesh::split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i)
	{
		Vertex v2 = v2i ? create_vertex_private(v2i) : create_vertex();
		std::vector<Corner> stackc;       // faces clw of vs1 and ccw of vs2
		Corner c = nullptr;
		if (vs1 && !c)
		{
			c = clw_corner(v1, edge(v1, vs1));
			for (; c; c = clw_corner(c))
			{
				stackc.push_back(c);
				if (vs2 && corner_vertex(ccw_face_corner(c)) == vs2) break;
			}
		}
		if (vs2 && !c)
		{
			c = ccw_corner(v1, edge(v1, vs2));
			for (; c; c = ccw_corner(c))
			{
				stackc.push_back(c);
				if (vs1 && corner_vertex(clw_face_corner(c)) == vs1) assert(true);
			}
		}
		for (Corner cc : stackc)
		{
			// ends up deleting and recreating MEdge structures, which is great.
			HEdge he = cc;
			remove_hedge(he, he->prev->vert); remove_hedge(he->next, he->vert);
			he->vert = v2;
			enter_hedge(he, he->prev->vert); enter_hedge(he->next, he->vert);
		}
		// info on edges around v1 still valid; edges on v2 all new
		return v2;
	}

	bool Mesh::legal_vertex_merge(Vertex vs, Vertex vt)
	{
		for (HEdge he : corners(vt))
		{
			if (query_hedge(he->prev->vert, vs)) return false;
			if (query_hedge(vs, he->next->vert)) return false;
		}
		return true;
	}

	void Mesh::merge_vertices(Vertex vs, Vertex vt)
	{
		assert(legal_vertex_merge(vs, vt));
		// We cannot introduce bogus hedges, because we intend to merge boundary edges together.
		// Change faces around vt to instead use vs.
		std::vector<Corner> arc; for (HEdge he : corners(vt)) { arc.push_back(he); }
		for (HEdge he : arc)
		{
			assert(he->vert == vt);
			remove_hedge(he, he->prev->vert); remove_hedge(he->next, he->vert);
			he->vert = vs;
			enter_hedge(he, he->prev->vert); enter_hedge(he->next, he->vert);
		}
		destroy_vertex(vt);
	}

	Vertex Mesh::center_split_face(Face f)
	{
		if (sdebug >= 1) valid(f);
		std::vector<Vertex> va; get_vertices(f, va);
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (HEdge he : corners(f)) { ar_he.push_back(he); }
		create_bogus_hedges(ar_he);
		// Destroy face
		destroy_face(f);
		// Create new vertex and faces
		Vertex vn = create_vertex();
		for_int(i, va.size())
		{
			create_face(va[i], va[(i + 1) % va.size()], vn);
		}
		remove_bogus_hedges(ar_he);
		return vn;
	}

	Edge Mesh::split_face(Face f, Vertex v1, Vertex v2)
	{
		if (sdebug >= 1) valid(v1), valid(v2);
		assert(!query_edge(v1, v2));
		std::vector<Vertex> va1, va2;
		for (Vertex v = v1; ; ) { va1.push_back(v); if (v == v2) break; v = ccw_vertex(f, v); }
		for (Vertex v = v2; ; ) { va2.push_back(v); if (v == v1) break; v = ccw_vertex(f, v); }
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (HEdge he : corners(f)) { ar_he.push_back(he); }
		create_bogus_hedges(ar_he);
		// Destroy face
		destroy_face(f);
		// Create new faces
		if (sdebug >= 1) assert(legal_create_face(va1) && legal_create_face(va2));
		create_face(va1); create_face(va2);
		remove_bogus_hedges(ar_he);
		return edge(v1, v2);
	}

	std::vector<Vertex> Mesh::gather_edge_coalesce_vertices(Edge e) const
	{
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);
		std::vector<Vertex> va;
		if (1)
		{
			std::vector<Vertex> va1; get_vertices(f1, va1); // slow but thread-safe
			std::vector<Vertex> va2; get_vertices(f2, va2);
			int nv1 = va1.size(), nv2 = va2.size();
			int i1, i2, ic;
			// Find one vertex common to both faces (v1)
			for (i1 = 0; i1<nv1; i1++) if (va1[i1] == v1) break;
			for (i2 = 0; i2<nv2; i2++) if (va2[i2] == v1) break;
			assert(i1<nv1 && i2<nv2);
			// Find most clw vertex on face1 common to both
			while (va1[(i1 - 1 + nv1) % nv1] == va2[(i2 + 1) % nv2])
			{
				i1 = (i1 - 1 + nv1) % nv1; i2 = (i2 + 1) % nv2;
			}
			// Let ic be the number of vertices common to both faces
			for (ic = 1; ; ic++) if (va1[(i1 + ic) % nv1] != va2[(i2 - ic + nv2) % nv2]) break;
			for_intL(i, ic - 1, nv1) { va.push_back(va1[(i1 + i) % nv1]); }
			for_int(i, nv2 - ic + 1) { va.push_back(va2[(i2 + i) % nv2]); }
			dummy_use(v2);
		}
		else
		{
			// this version only works for a single edge between 2 faces
			for (Vertex v = v2; ; )
			{
				va.push_back(v); v = ccw_vertex(f1, v); if (v == v1) break;
			}
			for (Vertex v = v1; ; )
			{
				va.push_back(v); v = ccw_vertex(f2, v); if (v == v2) break;
			}
		}
		return va;
	}

	bool Mesh::legal_coalesce_faces(Edge e)
	{
		if (is_boundary(e)) { return false; }
		std::vector<Vertex> va = gather_edge_coalesce_vertices(e);
		{                           // check for duplicate vertices
			std::unordered_set<Vertex> setv;
			for (Vertex v : va) { if (!setv.insert(v).second) return false; }
		}
		{                           // check that we get a "nice" face
			Face f1 = face1(e), f2 = face2(e);
			Face fp = nullptr; int i = 0;
			for (Face f : faces(e))
			{
				for (Edge ee : edges(f))
				{
					if (is_boundary(ee)) { fp = nullptr; break; }
					Face ff = opp_face(f, ee);
					if (ff == f1 || ff == f2) continue;
					if (!i++)
					{
						fp = ff;
					}
					else if (ff != fp)
					{
						fp = nullptr; break;
					}
				}
			}
			if (fp) { return false; }
		}
		return true;
	}

	Face Mesh::coalesce_faces(Edge e)
	{
		if (sdebug >= 1) assert(legal_coalesce_faces(e));
		Face f1 = face1(e), f2 = face2(e);
		std::vector<Vertex> va = gather_edge_coalesce_vertices(e);
		// See if any vertices can be deleted
		std::unordered_set<Vertex> vbefore;
		for (Vertex v : vertices(f1)) { vbefore.insert(v); }
		for (Vertex v : vertices(f2)) { vbefore.insert(v); }
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (Face f : faces(e))
		{
			for (HEdge he : corners(f)) { ar_he.push_back(he); }
		} create_bogus_hedges(ar_he);
		// Destroy faces
		destroy_face(f1);
		destroy_face(f2);
		// Create new face
		if (sdebug >= 1) assert(legal_create_face(va));
		Face fn = create_face(va);
		remove_bogus_hedges(ar_he);
		// Delete any isolated vertices
		for (Vertex v : va) { vbefore.erase(v); }
		for (Vertex v : vbefore) { destroy_vertex(v); }
		if (sdebug >= 3) ok();
		return fn;
	}

	Vertex Mesh::insert_vertex_on_edge(Edge e)
	{
		// If >1 edge shared between f1 and f2, other shared edges will be
		//  destroyed and recreated -> will lose attributes.
		if (sdebug >= 1) valid(e);
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (Face f : faces(e))
		{
			for (HEdge he : corners(f)) { ar_he.push_back(he); }
		} create_bogus_hedges(ar_he);
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);
		std::vector<Vertex> va1, va2;
		for (Vertex v = v2; ; )
		{
			va1.push_back(v); v = ccw_vertex(f1, v); if (v == v2) break;
		}
		if (f2)
		{
			for (Vertex v = v1; ; )
			{
				va2.push_back(v); v = ccw_vertex(f2, v); if (v == v1) break;
			}
		}
		destroy_face(f1);
		if (f2) destroy_face(f2);
		Vertex vn = create_vertex();
		va1.push_back(vn); f1 = create_face(va1); dummy_use(f1);
		if (f2) { va2.push_back(vn); f2 = create_face(va2); dummy_use(f2); }
		remove_bogus_hedges(ar_he);
		return vn;
	}

	Edge Mesh::remove_vertex_between_edges(Vertex vr)
	{
		std::vector<Face> fa; for (Face f : ccw_faces(vr)) { fa.push_back(f); }
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (Face f : fa)
		{
			for (HEdge he : corners(f)) { ar_he.push_back(he); }
		} create_bogus_hedges(ar_he);
		Vec2<std::vector<Vertex>> va;
		for_int(i, fa.size())
		{
			for (Vertex v = vr; ; )
			{
				v = ccw_vertex(fa[i], v); if (v == vr) break;
				va[i].push_back(v);
			}
		}
		for_int(i, fa.size()) { destroy_face(fa[i]); }
		destroy_vertex(vr);
		for_int(i, fa.size()) { fa[i] = create_face(va[i]); }
		remove_bogus_hedges(ar_he);
		return edge(va[0][0], va[0][va[0].size() - 1]);
	}

	bool Mesh::is_nice() const
	{
		for (Vertex v : vertices()) { if (!is_nice(v)) return false; }
		for (Face f : faces()) { if (!is_nice(f)) return false; }
		return true;
	}

	void Mesh::renumber()
	{
		int id = 1;
		// self-change to mesh is OK since ordered iterator creates copy of verts.
		for (Vertex v : ordered_vertices())
		{
			assert(v->id >= id);
			if (v->id != id)
			{
				_id2vertex.erase(v->id);
				v->id = id;
				_id2vertex.emplace(id, v);
			}
			id++;
		}
		id = 1;
		// self-change to mesh is OK since ordered iterator creates copy of faces.
		for (Face f : ordered_faces())
		{
			assert(f->id >= id);
			if (f->id != id)
			{
				_id2face.erase(f->id);
				f->id = id;
				_id2face.emplace(id, f);
			}
			id++;
		}
	}

	void Mesh::vertex_renumber_id_private(Vertex v, int newid)
	{
		if (v->id == newid) return;
		_id2vertex.erase(v->id);
		v->id = newid;
		_id2vertex.emplace(newid, v);
		_vertexnum = std::max(_vertexnum, newid + 1);
	}

	void Mesh::face_renumber_id_private(Face f, int newid)
	{
		if (f->id == newid) return;
		_id2face.erase(f->id);
		f->id = newid;
		_id2face.emplace(newid, f);
		_facenum = std::max(_facenum, newid + 1);
	}

	void Mesh::ok() const
	{
		// Check consistency of id2x (one way)
		for_map_key_value(_id2vertex, [&](int id, Vertex v)
		{
			valid(v);
			assert(v->id == id);
		});
		for_map_key_value(_id2face, [&](int id, Face f)
		{
			valid(f);
			assert(f->id == id);
		});
		// Look over Vertices
		std::unordered_set<HEdge> sethe;
		for (Vertex v1 : vertices())
		{
			assert(_id2vertex.at(v1->id) == v1);
			std::unordered_set<Vertex> set;
			for (HEdge he : v1->_arhe)
			{
				// Check that HEdges are valid
				valid(he);
				assert(he->prev->vert == v1);
				Vertex v2 = he->vert;
				// Check that v2's are not duplicated
				assert(set.insert(v2).second);
				// Check that sym matches that in _arhe
				assert(query_hedge(v2, v1) == he->sym);
				// Check that HEdge sym is valid
				if (he->sym)
				{
					valid(he->sym);
					assert(he->sym->edge == he->edge);
					assert(he->sym->sym == he);
					assert(he->sym->vert == he->prev->vert);
					assert(he->sym->prev->vert == he->vert);
				}
				// Check that hedges are unique, and one-to-one with ste (1)
				assert(sethe.insert(he).second);
				// Check that Faces reachable from Edges are valid
				valid(he->face);
				// Check that each HEdge appears in its face
				bool found = false;
				for (HEdge hee : corners(he->face))
				{
					if (hee == he) { found = true; break; }
				}
				assert(found);
				// Check that Edge is valid
				valid(he->edge);
			}
		}
		// Look over Faces
		for (Face f : faces())
		{
			assert(_id2face.at(f->id) == f);
			// Check that Face has valid erep
			assert(herep(f)->face == f);
			valid(herep(f)->edge);
			std::unordered_set<Vertex> set;
			for (Vertex v : vertices(f))
			{
				// Check Face contains no duplicate Vertices
				assert(set.insert(v).second);
				// Check Vertices in face are valid
				valid(v);
			}
			assert(set.size() >= 3);
			// Check Edges in Faces are one-to-one with _arhe (2)
			for (HEdge he : corners(f))
			{
				assert(he->face == f);
				assert(sethe.erase(he));
			}
		}
		// Check Edges in Faces are one-to-one with _arhe (3)
		assert(sethe.empty());
		// Check _nedges
		{
			int i = 0;
			for (Edge e : edges()) { dummy_use(e); i++; }
			assert(_nedges == i);
		}
	}

	bool Mesh::valid(Vertex v) const
	{
		assert(v && v == _id2vertex.at(vertex_id(v)));
		return true;
	}

	bool Mesh::valid(Face f) const
	{
		assert(f && f == _id2face.at(face_id(f)));
		return true;
	}

	bool Mesh::valid(Edge e) const
	{
		assert(e);
		valid(vertex1(e));
		valid(vertex2(e));
		valid(face1(e));
		assert(ordered_edge(vertex1(e), vertex2(e)) == e);
		return true;
	}

	bool Mesh::valid(Corner c) const
	{
		HEdge he = c;               // they are equivalent
		assert(he && he->next && he->prev && he->face && he->vert); //HH_ASSUME(he);
		Edge e = he->edge;
		assert(e);
		assert(hedge_from_ev2(e, he->vert) == he);
		return true;
	}

	// *** Iterators

	Mesh::OrderedVertices_range::OrderedVertices_range(const Mesh& mesh)
	{
		_vertices.reserve(mesh.num_vertices());
		for (Vertex v : mesh.vertices()) { _vertices.push_back(v); }
		std::sort(_vertices.begin(), _vertices.end(), [&mesh](Vertex v1, Vertex v2) { return mesh.vertex_id(v1)<mesh.vertex_id(v2); });
	}

	Mesh::OrderedFaces_range::OrderedFaces_range(const Mesh& mesh)
	{
		_faces.reserve(mesh.num_faces());
		for (Face f : mesh.faces()) { _faces.push_back(f); }
		std::sort(_faces.begin(), _faces.end(), [&mesh](Face f1, Face f2) { return mesh.face_id(f1)<mesh.face_id(f2); });
	}

	// *** Mesh protected

	Mesh::HEdge Mesh::most_clw_hedge(Vertex v) const
	{
		assert(is_nice(v));
		HEdge he = herep(v);
		if (!he) return nullptr;
		for (HEdge hef = he; ; )
		{
			HEdge hen = clw_hedge(he);
			if (!hen || hen == hef) break;
			he = hen;
		}
		return he;
	}

	Mesh::HEdge Mesh::most_ccw_hedge(Vertex v) const
	{
		assert(is_nice(v));
		HEdge he = herep(v);
		if (!he) return nullptr;
		for (HEdge hef = he; ; )
		{
			HEdge hen = ccw_hedge(he);
			if (!hen || hen == hef) break;
			he = hen;
		}
		return he;
	}

	Mesh::HEdge Mesh::get_hedge(Vertex v, Face f) const
	{
		for (Corner he : corners(v))
		{
			if (he->face == f) { return he; }
		}
		assert(false);
	}

	Mesh::HEdge Mesh::query_hedge(Vertex v1, Vertex v2) const
	{
		for (HEdge he : v1->_arhe)
		{
			if (he->vert == v2) return he;
		}
		return nullptr;
	}

	// Must have:           vert
	// Defines:             sym, edge
	// Must define elsewhere: next, prev, face, edge
	void Mesh::enter_hedge(HEdge he, Vertex v1)
	{
		Vertex v2 = he->vert;
		if (sdebug >= 1)
		{
			valid(v1); valid(v2);
			bool contains = false;
			auto it = v1->_arhe.begin();
			for (it; it != v1->_arhe.end(); it++)
			{
				if (*it == he)
				{
					contains = true;
					break;
				}
			}
			assert(!query_hedge(v1, v2) && !contains);
		}
		v1->_arhe.push_back(he);
		HEdge hes = he->sym = query_hedge(v2, v1);
		if (hes)
		{
			assert(!hes->sym);
			hes->sym = he;
			Edge e = hes->edge;
			he->edge = e;
			if (he->vert->id>hes->vert->id) e->herep = he;
		}
		else
		{
			_nedges++;
			Edge e = new MEdge(he);
			he->edge = e;
		}
	}

	// Must have:           vert, sym, edge
	// Must do later:       delete he
	// Note: must be careful: could have he==herep(he->face) ! :
	//   destroy_face() : deletes face -> ok.
	//   substitute_face_vertex() : reintroduces edge -> ok.
	void Mesh::remove_hedge(HEdge he, Vertex v1)
	{
		Vertex v2 = he->vert;
		if (sdebug >= 1)
		{
			valid(v1); valid(v2);
			assert(he == query_hedge(v1, v2));
			assert(he->sym == query_hedge(v2, v1));
		}
		Edge e = he->edge;
		HEdge hes = he->sym;
		if (hes)
		{
			hes->sym = nullptr;
			if (e->herep == he) e->herep = hes;
		}
		else
		{
			--_nedges;
			// e->herep = nullptr;     // optional
			delete e;
		}
		// he->edge = nullptr;         // optional
		auto it = v1->_arhe.begin();
		for (it; it != v1->_arhe.end(); it++)
		{
			if (*it == he)
				break;
		}
		v1->_arhe.erase(it); // slow, shucks
	}

	void Mesh::create_bogus_hedges(std::vector<HEdge> ar_he)
	{
		for_int(i, ar_he.size())
		{
			HEdge& he = ar_he[i];
			if (is_boundary(he))
			{
				HEdge heo = he;
				Vertex v1 = heo->vert;
				he = new MHEdge;
				//HH_ASSUME(heo->prev);
				he->vert = heo->prev->vert;
				he->prev = nullptr; // note: temporarily causes mesh.ok() to fail
				he->next = nullptr;
				he->face = reinterpret_cast<Face>(v1); // temporary overload
				enter_hedge(he, v1);
			}
			else
			{
				he = nullptr;
			}
		}
	}

	void Mesh::remove_bogus_hedges(std::vector<HEdge> ar_he)
	{
		for (HEdge he : ar_he)
		{
			if (!he) continue;
			Vertex v1 = reinterpret_cast<Vertex>(he->face); // temporary overload
			remove_hedge(he, v1);
			delete he;
		}
	}

} // namespace HuguesHoppe
