// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Spatial.hpp"

namespace HuguesHoppe
{

	// Given 10000 random data points uniformly sampled over the unit cube,
	// find the closest 10 neighbors:
	// spatialtest -gn ? -pn 10000
	// average time
	//      ?       10000           1000
	//      1       1.1745          .14967
	//      2       .25267          .05333
	//      3       .11967          .03883
	//      4       .07450          .03217
	//      5       .05583          .02917
	//      10      .03266          .02517
	//      20      .02733          .02433
	//      30      .02500          .02833
	//      40      .02500          .03550
	//      50      .02550          .04733
	//      70      .03067          .08950
	//      100     .04383          .20933
	//      200     .18033          1.4403
	// optimal #cells/point:
	//              .1-30           .06-40
	// 
	// for 100'000 data points, gn=40, time is .03100 (still very good)
	//                          gn=50, time is .02583

	// *** BPointSpatial
	void BPointSpatial::clear()
	{
		_map.clear();
	}

	void BPointSpatial::enter(int id, const glm::vec3* pp)
	{
		Ind ci = point_to_indices(*pp); assert(indices_inbounds(ci));
		int en = encode(ci);
		_map[en].push_back(Node(id, pp));
	}

	void BPointSpatial::remove(int id, const glm::vec3* pp)
	{
		Ind ci = point_to_indices(*pp); assert(indices_inbounds(ci));
		int en = encode(ci);
		std::vector<Node>& ar = _map.at(en);
		int ind = -1; for_int(i, ar.size()) { if (ar[i].id == id) { assert(ind < 0); ind = i; } } assert(ind >= 0);
		ar.erase(ar.begin() + ind);
		if (!ar.size()) _map.erase(en);
	}

	void BPointSpatial::shrink_to_fit()
	{
		for (auto& cell : _map) { cell.second.shrink_to_fit(); }
	}

	void BPointSpatial::add_cell(const Ind& ci, SPriority_Queue& pq, const glm::vec3& pcenter, std::set<int>&) const
	{
		// SHOW("add_cell", ci);
		int en = encode(ci);
		bool present = _map.find(en) != _map.end();
		const std::vector<Node>& cell = _map.find(en)->second;
		if (!present) return;
		for (const Node& e : cell) {
			// SHOW("enter", *e.p, dist2(pcenter, *e.p));
			PQNode<Node>& node = PQNode<Node>(e, glm::distance2(pcenter, *e.p));
			pq.push(node);
		}
	}

	int BPointSpatial::pq_id(const Node& id) const
	{
		return id.id;
	}

	// *** IPointSpatial
	/*IPointSpatial::IPointSpatial(int gn, CArrayView<Point> arp) : Spatial(gn), _pp(arp.data())
	{
		for_int(i, arp.num()) {
			Ind ci = point_to_indices(arp[i]); assert(indices_inbounds(ci));
			int en = encode(ci);
			_map[en].push(i);       //  first creates empty Array<int> if not present
		}
		if (0) for (auto& cell : _map.values()) { cell.shrink_to_fit(); }
	}

	void IPointSpatial::clear()
	{
		_map.clear();
	}

	void IPointSpatial::add_cell(const Ind& ci, Pqueue<Univ>& pq, const Point& pcenter, std::set<int>&) const
	{
		int en = encode(ci);
		bool present; auto& cell = _map.retrieve(en, present);
		if (!present) return;
		for (int i : cell) {
			pq.enter(Conv<int>::e(i), dist2(pcenter, _pp[i]));
		}
	}

	Univ IPointSpatial::pq_id(Univ id) const
	{
		return id;
	}
	*/

	// *** SpatialSearch

	BSpatialSearch::BSpatialSearch(const Spatial& sp, const glm::vec3& p, float maxdis)
		: _sp(sp), _pcenter(p), _maxdis(maxdis)
	{
		// SHOW("search", p, maxdis);
		Ind ci = _sp.point_to_indices(_pcenter);
		assert(_sp.indices_inbounds(ci));
		for_int(i, 2) for_int(c, 3) { _ssi[i][c] = ci[c]; }
		consider(ci);
		get_closest_next_cell();
	}

	BSpatialSearch::~BSpatialSearch()
	{
		//HH_SSTAT(Sssncellsv, _ncellsv);
		//HH_SSTAT(Sssnelemsv, _nelemsv);
	}

	bool BSpatialSearch::done()
	{
		for (;;) {
			if (!_pq.empty()) return false;
			if (_disbv2 >= square(_maxdis)) return true;
			expand_search_space();
		}
	}

	int BSpatialSearch::next(float* pdis2)
	{
		Node u;
		for (;;) {
			if (_pq.empty()) assert(!done()); // refill _pq
			float dis2 = _pq.top()._pri;
			if (dis2 > _disbv2) {
				expand_search_space();
				continue;
			}
			u = _pq.top()._e;
			_sp.pq_refine(_pq, _pcenter);
			if (_pq.top()._e != u || _pq.top()._pri != dis2) continue;
			if (pdis2) *pdis2 = _pq.top()._pri;
			u = _pq.top()._e;
			_pq.pop();
			break;
		}
		return _sp.pq_id(u);
	}

	void BSpatialSearch::consider(const Ind& ci)
	{
		// SHOW("consider", ci);
		_ncellsv++;
		int n = _pq.size();
		_sp.add_cell(ci, _pq, _pcenter, _setevis);
		_nelemsv += _pq.size() - n;
	}

	void BSpatialSearch::get_closest_next_cell()
	{
		float mindis = 1e10f;
		for_int(c, 3) {
			if (_ssi[0][c]>0) {
				float a = _pcenter[c] - _sp.index_to_float(_ssi[0][c]);
				if (a<mindis) { mindis = a; _axis = c; _dir = 0; }
			}
			if (_ssi[1][c]<_sp._gn - 1) {
				float a = _sp.index_to_float(_ssi[1][c] + 1) - _pcenter[c];
				if (a<mindis) { mindis = a; _axis = c; _dir = 1; }
			}
		}
		// mindis may be big if all of space has been searched
		_disbv2 = square(mindis);
	}

	void BSpatialSearch::expand_search_space()
	{
		assert(_axis >= 0 && _axis<3 && _dir >= 0 && _dir <= 1);
		// SHOW("expand", _axis, _dir, _ssi);
		Vec2<Ind> bi = _ssi;
		_ssi[_dir][_axis] += _dir ? 1 : -1;
		bi[0][_axis] = bi[1][_axis] = _ssi[_dir][_axis];
		// consider the layer whose axis's value is _ssi[_dir][_axis]
		Ind temp = glm::ivec3(bi[1][0] + 1, bi[1][1] + 1, bi[1][2] + 1);
		for (const Ind& cit : coordsL<3>(bi[0], temp)) { consider(cit); }
		get_closest_next_cell();
	}

} // namespace hh
