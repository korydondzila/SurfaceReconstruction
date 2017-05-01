// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef GRAPH_H
#define GRAPH_H

#include "includes\includes.hpp"
#include "UnionFind.hpp"

namespace HuguesHoppe
{

	// A Graph encodes a relation from a set of elements to itself, represented by a set of edge tuples (T, T).
	// The Graph allows quick iteration over outgoing edges.  Duplicate edges are not allowed.
	template<typename T> class Graph : noncopyable
	{
		using type = Graph<T>;
		using atype = std::vector<T>;
		using base = std::unordered_map<T, atype>;
		using bciter = typename base::const_iterator;
	public:
		class vertices_range; class vertex_iterator;
		using edges_range = const atype&;
		using edge_iterator = const typename atype::const_iterator;
		Graph() = default;
		Graph(type&& g) noexcept { swap(*this, g); }                        // =default?
		type& operator=(type&& g) noexcept { clear(); swap(*this, g); return *this; } // =default?
		void clear() { _m.clear(); }
		bool empty() const { return _m.empty(); }
		// enter and remove domain vertices
		void enter(T v) { _m.emplace(v, atype()); } // vertex v must be new
		bool contains(T v) const { return _m.find(v) != _m.end(); }
		bool remove(T v);           // must have 0 out_degree, ret: was_there
									// enter an edge
		void enter(T v1, T v2) { assert(!contains(v1, v2)); _m.at(v1).push_back(v2); }
		// enter undirected edge
		void enter_undirected(T v1, T v2) { enter(v1, v2); enter(v2, v1); } // v1 & v2 present; new edge
		bool contains(T v1, T v2)
		{
			std::vector<T> a = _m.at(v1);
			auto it = find(a, v2);
			if (it != a.end()) return true;
			return false;
		}         // O(n) slow
		bool remove(T v1, T v2)
		{
			atype& a = _m.at(v1); auto it = find(a, v2); if (it != a.end()) { a.erase(it); return true; }
			return false;
		} // O(n) , ret: was_there
		auto find(const atype& ar, T v2) { return std::find(ar.begin(), ar.end(), v2); }
		bool remove_undirected(T v1, T v2); // O(n) , ret: was_there
		int out_degree(T v) const { return _m.at(v).size(); }
		void add(const Graph<T>& g);
		vertices_range vertices() const { return vertices_range(_m); }
		edges_range edges(T v) const { return _m.at(v); }
		friend void swap(Graph& l, Graph& r) noexcept { using std::swap; swap(l._m, r._m); }
	public:
		class vertex_iterator : public std::iterator<std::forward_iterator_tag, const T>
		{
		public:
			vertex_iterator() = default;
			vertex_iterator(bciter it) : _it(it) { }
			bool operator!=(const vertex_iterator& rhs) const { return _it != rhs._it; }
			bool operator==(const vertex_iterator& rhs) const { return _it == rhs._it; }
			const T& operator*() const { return _it->first; }
			vertex_iterator& operator++() { ++_it; return *this; }
		private:
			bciter _it;
		};
		class vertices_range
		{
		public:
			vertices_range(const base& t) : _t(t) { }
			vertex_iterator begin() const { return vertex_iterator(_t.begin()); }
			vertex_iterator end() const { return vertex_iterator(_t.end()); }
		private:
			const base& _t;
		};
	private:
		base _m;
	};


	//----------------------------------------------------------------------------

	template<typename T> bool Graph<T>::remove(T v) {
		if (!contains(v)) return false;
		atype& ar = _m.at(v);
		_m.erase(v);
		assert(!ar.size());
		return true;
	}

	template<typename T> bool Graph<T>::remove_undirected(T v1, T v2) {
		bool r1 = remove(v1, v2);
		bool r2 = remove(v2, v1);
		assert(r1 == r2);
		return r1;
	}

	template<typename T> void Graph<T>::add(const Graph& g) {
		for (const T& v1 : g.vertices())
		{
			for (const T& v2 : g.edges(v1))
			{
				// if (!contains(v1, v2)) enter(v1, v2);
				atype& ar = _m.at(v1);
				if (find(ar, v2) == ar.end()) ar.push_back(v2);
			}
		}
	}

	// *** Kruskal MST

	// Given a graph gnew consisting solely of vertices, computes the minimum spanning tree of undirectedg over
	//  the vertices in gnew under the cost metric fdist.  Returns is_connected.
	// Implementation: Kruskal's algorithm, O(e log(e))
	//  (Prim's algorithm is recommended when e=~n^2, see below)
	template<typename T, typename Func = float(const T&, const T&)>
	bool graph_mst(const Graph<T>& undirectedg, Func fdist, Graph<T>& gnew)
	{
		int nv = 0, nebefore = 0;
		struct tedge { T v1, v2; float w; };
		std::vector<tedge> tedges = std::vector<tedge>();

		for (const T& v1 : gnew.vertices())
		{
			nv++;
			assert(!gnew.out_degree(v1));
			for (const T& v2 : undirectedg.edges(v1))
			{
				if (v1 < v2) continue;
				assert(gnew.contains(v2));
				nebefore++;
				tedges.push_back(tedge{ v1, v2, fdist(v1, v2) });
			}
		}

		std::sort(tedges.begin(), tedges.end(), [](const tedge& a, const tedge& b) { return a.w < b.w; });

		UnionFind<T> uf;
		int neconsidered = 0, neadded = 0;

		for (const tedge& t : tedges)
		{
			neconsidered++;
			T v1 = t.v1, v2 = t.v2;
			if (!uf.unify(v1, v2)) continue;
			gnew.enter_undirected(v1, v2);
			neadded++;
			if (neadded == nv - 1) break;
		}

		printf("graph_mst: %d vertices, %d/%d edges considered, %d output\n", nv, neconsidered, nebefore, neadded);
		return neadded == nv - 1;
	}
} // namespace hh

#endif // GRAPH_H
