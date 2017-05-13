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

	// Returns v clamped to the range [a, b].
	template<typename T> constexpr T clamp(const T& v, const T& a, const T& b)
	{
		return (assert(!(v < a && v > b)), v < a ? a : v > b ? b : v);
	}

	// Avoid warnings of unused variables
	template<typename... A> void dummy_use(const A&...) { } // C++14: "constexpr void dummy_use(" becomes OK

	constexpr float    BIGFLOAT = 1e30f;

	template<typename K, typename V> void get_keys(const std::unordered_map<K, V>& map, std::vector<K>& keys)
	{
		std::transform(map.begin(), map.end(), keys.begin(), [](auto pair) {return pair.first; });
	}

	template<typename K, typename V> void get_values(const std::unordered_map<K, V>& map, std::vector<V>& values)
	{
		std::transform(map.begin(), map.end(), values.begin(), [](auto pair) {return pair.second; });
	}

	// Iterate over both keys and values simultaneously
	template<typename K, typename V, typename Hash = std::hash<K>, typename Equal = std::equal_to<K>,
		typename Func = void(const K& key, const V& val)>
		inline void for_map_key_value(const std::unordered_map<K, V, Hash, Equal>& map, Func func) {
		for (auto& kv : map) { func(kv.first, kv.second); }
	}

	template<typename T> struct Vec2
	{
		T v1;
		T v2;
	public:
		Vec2() : v1(T()), v2(T()) {}
		Vec2(const T& v1, const T& v2) : v1(v1), v2(v2) {}
		T& operator[](const int& k)
		{
			return k == 0 ? v1 : v2;
		}

		const T& operator[](const int& k) const
		{
			return k == 0 ? v1 : v2;
		}
	};

	template<typename T> struct Vec3
	{
		T v1;
		T v2;
		T v3;
	public:
		Vec3() : v1(T()), v2(T()), v3(T()) {}
		Vec3(const T& v1, const T& v2, const T& v3) : v1(v1), v2(v2), v3(v3) {}
		T& operator[](const int& k)
		{
			return k == 0 ? v1 : k == 1 ? v2 : v3;
		}

		const T& operator[](const int& k) const
		{
			return k == 0 ? v1 : k == 1 ? v2 : v3;
		}
	};

	template<typename T> struct Vec4
	{
		T v1;
		T v2;
		T v3;
		T v4;
	public:
		Vec4() : v1(T()), v2(T()), v3(T()), v4(T()) {}
		Vec4(const T& v1, const T& v2, const T& v3, const T& v4) : v1(v1), v2(v2), v3(v3), v4(v4) {}
		T& operator[](const int& k)
		{
			return k == 0 ? v1 : k == 1 ? v2 : k == 2 ? v3 : v4;
		}

		const T& operator[](const int& k) const
		{
			return k == 0 ? v1 : k == 1 ? v2 : k == 2 ? v3 : v4;
		}
	};
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
