// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "includes\includes.hpp"
#include "Mesh.hpp"

namespace HuguesHoppe
{
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

	Face Mesh::create_face_private(int id, std::vector<Vertex> va)
	{
		assert(id >= 1);
		assert(va.size() >= 3);
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
		return f;
	}

	void Mesh::destroy_face(Face f)
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
		assert(_id2face.erase(f->id));
		if (_facenum - 1 == f->id) --_facenum;
		delete f;
	}

	// *** Face

	void Mesh::get_vertices(Face f, std::vector<Vertex>& va) const
	{
		va.clear(); for (Vertex v : vertices(f)) { va.push_back(v); }
	}

	// *** Edge

	Vertex Mesh::opp_vertex(Edge e, Face f) const
	{
		HEdge he = hedge_from_ef(e, f);
		if (he->next->next->next != he) assert(true); // cheaper than is_triangle(f)
		return he->next->vert;
	}

	// *** Other associations

	Edge Mesh::query_edge(Vertex v, Vertex w) const
	{
		HEdge he;
		he = query_hedge(v, w); if (he) return he->edge;
		he = query_hedge(w, v); if (he) return he->edge;
		return nullptr;
	}

	bool Mesh::legal_edge_swap(Edge e) const
	{
		if (is_boundary(e)) return false;
		if (query_edge(side_vertex1(e), side_vertex2(e))) return false;
		return true;
	}

	// *** Mesh operations

	Edge Mesh::swap_edge(Edge e)
	{
		Vertex v1 = vertex1(e), v2 = vertex2(e);
		Face f1 = face1(e), f2 = face2(e);
		Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e); // implies triangles
															 // Create bogus hedges if boundaries
		std::vector<HEdge>* ar_he = new std::vector<HEdge>(); for (Face f : faces(e))
		{
			for (HEdge he : corners(f)) { ar_he->push_back(he); }
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

	Vertex Mesh::center_split_face(Face f)
	{
		std::vector<Vertex> va; get_vertices(f, va);
		int numVerts = va.size();
		// Create bogus hedges if boundaries
		std::vector<HEdge> ar_he; for (HEdge he : corners(f)) { ar_he.push_back(he); }
		create_bogus_hedges(&ar_he);
		// Destroy face
		destroy_face(f);
		// Create new vertex and faces
		Vertex vn = create_vertex();

		glm::vec3 average = glm::vec3();
		for (Vertex vert : va)
		{
			average += vert->point;
		}

		vn->point = glm::vec3(average.x / numVerts, average.y / numVerts, average.z / numVerts);

		for_int(i, numVerts)
		{
			create_face(va[i], va[(i + 1) % numVerts], vn);
		}
		remove_bogus_hedges(&ar_he);
		return vn;
	}

	// *** Mesh protected

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

	void Mesh::create_bogus_hedges(std::vector<HEdge> *ar_he)
	{
		std::vector<HEdge> temp(ar_he->size());
		for_int(i, ar_he->size())
		{
			HEdge he = (*ar_he)[i];
			if (is_boundary(he))
			{
				HEdge heo = he;
				Vertex v1 = heo->vert;
				he = new MHEdge;
				//HH_ASSUME(heo->prev);
				he->vert = heo->prev->vert;
				he->prev = nullptr; // note: temporarily causes mesh.ok() to fail
				he->next = nullptr;
				he->face = (Face)(v1); // temporary overload
				enter_hedge(he, v1);
			}
			else
			{
				he = nullptr;
			}

			temp[i] = he;
		}
		
		ar_he->clear();
		for (HEdge he : temp)
		{
			ar_he->push_back(he);
		}
	}

	void Mesh::remove_bogus_hedges(std::vector<HEdge>* ar_he)
	{
		for (HEdge he : *ar_he)
		{
			if (!he) continue;
			Vertex v1 = reinterpret_cast<Vertex>(he->face); // temporary overload
			remove_hedge(he, v1);
			delete he;
		}
	}

} // namespace HuguesHoppe
