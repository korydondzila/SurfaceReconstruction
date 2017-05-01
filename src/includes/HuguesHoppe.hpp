#ifndef HUGUES_HOPPE_H
#define HUGUES_HOPPE_H

namespace HuguesHoppe
{
	// Derive from this class to disable copy constructor and copy assignment.
	struct noncopyable
	{
	protected:
		noncopyable(const noncopyable&) = delete;
		noncopyable& operator=(const noncopyable&) = delete;
		noncopyable() = default;
	};

	// Returns a value times itself.
	template<typename T> constexpr T square(const T& e) { return e*e; }

	// Avoid warnings of unused variables
	template<typename... A> void dummy_use(const A&...) { } // C++14: "constexpr void dummy_use(" becomes OK

	constexpr float    BIGFLOAT = 1e30f;
} // namespace HuguesHoppe

#define HH_CAT(a, b) a##b
#define HH_CAT2(a, b) HH_CAT(a, b)
#define HH_ID(x) HH_CAT(_hh_id_, x) // private identifier in a macro definition
#define HH_UNIQUE_ID(x) HH_CAT2(HH_CAT2(HH_CAT2(_hh_id_, __COUNTER__), _), x)

#define for_range_aux(T, i, lb, ub, u) for (T u = ub, i = lb; i<u; i++)
#define for_range(T, i, lb, ub) for_range_aux(T, i, lb, ub, HH_UNIQUE_ID(u))
#define for_int(i, ub)      for_range(int, i, 0, ub)
#define for_intL(i, lb, ub) for_range(int, i, lb, ub)
#define for_size_t(i, ub)   for_range(std::size_t, i, 0, ub)

#endif // !HUGUES_HOPPE_H
