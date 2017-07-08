// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef SPATIAL_H
#define SPATIAL_H

#include "includes/includes.hpp"

namespace HuguesHoppe
{
	template<typename T> struct PQNode
	{
		PQNode() = default;
		PQNode(const T& e, float pri) : _e(e), _pri(pri) { }
		//PQNode(T&& e, float pri) : _e(std::move(e)), _pri(pri) { }
		//PQNode& operator=(PQNode&& n) noexcept { _e = std::move(n._e); _pri = n._pri; return *this; }
		T _e;
		float _pri;
	};

	struct Node
	{
		Node() = default;
		Node(int pid, const glm::vec3* pp) : id(pid), p(pp) { }
		bool operator==(const Node& other) const { return id == other.id && p == other.p; }
		bool operator!=(const Node& other) const { return !(*this == other);  }
		int id;
		const glm::vec3* p;
	};

	struct cmp
	{
		bool operator()(const PQNode<Node>& left, const PQNode<Node>& right)
		{
			return left._pri > right._pri;
		}
	};

	using SPriority_Queue = std::priority_queue<PQNode<Node>, std::vector<PQNode<Node>>, cmp>;

	using Ind = glm::ivec3;

	// Spatial data structure for efficient queries like "closest_elements" or "find_elements_intersecting_ray".
	class Spatial : noncopyable // abstract class
	{
		static constexpr int k_max_gn = 1023; // 10 bits per coordinate
	public:
		explicit Spatial(int gn, const Vec2<glm::vec3>& boxBound) : 
			_gn(gn), _boxBounds(boxBound) { assert(_gn <= k_max_gn); _gni = 1.f / float(_gn); }
		virtual ~Spatial() { } // not =default because gcc "looser throw specified" in derived
		virtual void clear() = 0;
	protected:
		friend class BSpatialSearch;
		int _gn;                    // grid size
		float _gni;                 // 1/_gn
		Vec2<glm::vec3> _boxBounds;
									//
		int inbounds(int i) const { return i >= 0 && i<_gn; }
		int indices_inbounds(const Ind& ci) const { return inbounds(ci[0]) && inbounds(ci[1]) && inbounds(ci[2]); }
		int float_to_index(int axis, float fd) const;
		float index_to_float(int i) const { return i*_gni; }
		Ind point_to_indices(const glm::vec3& p) const { Ind ci; for_int(c,3) { ci[c] = float_to_index(c, p[c]); } return ci; }
		glm::vec3 indices_to_point(const Ind& ci) const { glm::vec3 p; for_int(c, 3) { p[c] = index_to_float(ci[c]); } return p; }
		int encode(const Ind& ci) const { return (ci[0] << 20) | (ci[1] << 10) | ci[2]; } // k_max_gn implied here
		Ind decode(int en) const;
		// for BSpatialSearch:
		// Add elements from cell ci to priority queue with priority equal to distance from pcenter squared.
		// May use set to avoid duplication.
		virtual void add_cell(const Ind& ci, SPriority_Queue& pq, const glm::vec3& pcenter, std::set<int>& set) const = 0;
		// Refine distance estimate of first entry in pq (optional)
		virtual void pq_refine(SPriority_Queue& pq, const glm::vec3& pcenter) const { dummy_use(pq, pcenter); }
		virtual int pq_id(const Node& pqe) const = 0; // given pq entry, return id
	};

	// Spatial data structure for point elements.
	class BPointSpatial : public Spatial
	{
	public:
		explicit BPointSpatial(int gn, const Vec2<glm::vec3>& boxBound) : Spatial(gn, boxBound) { }
		~BPointSpatial() { clear(); }
		void clear() override;
		void enter(int id, const glm::vec3* pp);  // note: pp not copied, no ownership taken
		void remove(int id, const glm::vec3* pp); // must exist, else die
		void shrink_to_fit();                  // often just fragments memory
	private:
		void add_cell(const Ind& ci, SPriority_Queue& pq, const glm::vec3& pcenter, std::set<int>& set) const override;
		int pq_id(const Node& pqe) const override;
		std::unordered_map<int, std::vector<Node>> _map; // encoded cube index -> Array
	};

	// Search for nearest element(s) from a given query point.
	class BSpatialSearch : noncopyable
	{
	public:
		// pmaxdis is only a request, you may get objects that lie farther
		BSpatialSearch(const Spatial& sp, const glm::vec3& p, float maxdis = 10.f);
		~BSpatialSearch() {}
		bool done();
		int next(float* dis2 = nullptr); // ret id
	private:
		friend Spatial;
		const Spatial& _sp;
		const glm::vec3 _pcenter;
		float _maxdis;
		SPriority_Queue _pq;           // pq of entries by distance
		Vec2<Ind> _ssi;             // search space indices (extents)
		float _disbv2{ 0.f };        // distance to search space boundary
		int _axis;                  // axis to expand next
		int _dir;                   // direction in which to expand next (0, 1)
		std::set<int> _setevis;         // may be used by add_cell()
		int _ncellsv{ 0 };
		int _nelemsv{ 0 };
		//
		void get_closest_next_cell();
		void expand_search_space();
		void consider(const Ind& ci);
	};


	//----------------------------------------------------------------------------

	inline int Spatial::float_to_index(int axis, float fd) const
	{
		float f = fd, min = _boxBounds[0][axis], max = _boxBounds[1][axis];
		if (f <= min + 0.01f) { assert(f >= min - 0.01f); f = min + 0.01f; }
		if (f >= max - 0.01f) { assert(f <= max + 0.01f); f = max - 0.01f; }

		f = (f - min) / (max - min);

		return int(f*_gn);
	}

	inline Ind Spatial::decode(int en) const
	{
		Ind ci;
		// Note: k_max_gn implied here.
		ci[2] = en&((1 << 10) - 1);
		en = en >> 10;
		ci[1] = en&((1 << 10) - 1);
		en = en >> 10;
		ci[0] = en;
		return ci;
	}

	class PointSpatial : public BPointSpatial
	{
	public:
		explicit PointSpatial(int gn, const Vec2<glm::vec3>& boxBound) : BPointSpatial(gn, boxBound) { }
		void enter(int id, const glm::vec3* pp) { BPointSpatial::enter(id, pp); }
		void remove(int id, const glm::vec3* pp) { BPointSpatial::remove(id, pp); }
	};

	class SpatialSearch : public BSpatialSearch
	{
	public:
		SpatialSearch(const Spatial& psp, const glm::vec3& pp, float pmaxdis = 10.f) : BSpatialSearch(psp, pp, pmaxdis) { }
		int next(float* dis2 = nullptr) { return BSpatialSearch::next(dis2); }
	};

	// Iterator for traversing the coordinates of a grid. uL[0]<=[0]<uU[0], ..., uL[D-1]<=[D-1]<uU[D-1].
	template<int D> class CoordL_iterator : public std::iterator<std::forward_iterator_tag, const Ind>
	{
		using type = CoordL_iterator<D>;
	public:
		CoordL_iterator(const Ind& uL, const Ind& uU) : _u(uL), _uL(uL), _uU(uU) { }
		CoordL_iterator(const type& iter) = default;
		bool operator!=(const type& rhs) const
		{
			assert(rhs._uU == _uU);
			assert(rhs._u[0] == _uU[0]);
			return _u[0]<_uU[0];    // quick check against usual end()
		}
		const Ind& operator*() const { assert(_u[0]<_uU[0]); return _u; }
		type& operator++() {
			assert(_u[0]<_uU[0]);
			for (int c = D - 1; c>0; --c)
			{
				_u[c]++;
				if (_u[c]<_uU[c]) return *this;
				_u[c] = _uL[c];
			}
			_u[0]++;
			return *this;
		}
	private:
		Ind _u, _uL, _uU;
	};

	// Range of grid coordinates uL[0]<=[0]<uU[0], ..., uL[D-1]<=[D-1]<uU[D-1].
	template<int D> class CoordL_range
	{
	public:
		CoordL_range(const Ind& uL, const Ind& uU) : _uL(uL), _uU(uU) { }
		CoordL_iterator<D> begin() const { return CoordL_iterator<D>(_uL, _uU); }
		CoordL_iterator<D> end() const { return CoordL_iterator<D>(_uU, _uU); }
	private:
		Ind _uL, _uU;
	};

	  // Construct a grid coordinate range.   e.g.: for (const auto& p : coords(grid.dims())) { grid[p] = func(p); }
	template<int D> CoordL_range<D> coordsL(const Ind& uL, const Ind& uU)
	{
		for_int(c, D) { if (uU[c] <= uL[c]) return CoordL_range<D>(uU, uU); }
		return CoordL_range<D>(uL, uU);
	}

} // namespace HuguesHoppe

#endif // SPATIAL_H
