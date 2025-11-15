#ifndef INC_VDB_H_
#define INC_VDB_H_
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <cmath>
#include <limits>
#include <type_traits>
#include <bit>

namespace vdb
{
    using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;
using f64 = double;

using size_t = std::size_t;
using intptr_t = std::intptr_t;
using uintptr_t = std::uintptr_t;
using ptrdiff_t = std::ptrdiff_t;

#if defined(ANDROID)
static constexpr f32 F32_EPSILON = 1.192092896e-07F;
static constexpr f32 F64_EPSILON = 2.2204460492503131e-016;
#else
static constexpr f32 F32_EPSILON = FLT_EPSILON;
static constexpr f32 F64_EPSILON = DBL_EPSILON;
#endif

static constexpr f32 PI = static_cast<f32>(3.14159265358979323846);
static constexpr f32 PI2 = static_cast<f32>(6.28318530717958647692);
static constexpr f32 INV_PI = static_cast<f32>(0.31830988618379067153);
static constexpr f32 INV_PI2 = static_cast<f32>(0.15915494309189533576);
static constexpr f32 PI_2 = static_cast<f32>(1.57079632679489661923);
static constexpr f32 INV_PI_2 = static_cast<f32>(0.63661977236758134308);
static constexpr f32 LOG2 = static_cast<f32>(0.693147180559945309417);
static constexpr f32 INV_LOG2 = static_cast<f32>(1.0 / 0.693147180559945309417);

static constexpr f64 PI_64 = static_cast<f64>(3.14159265358979323846);
static constexpr f64 PI2_64 = static_cast<f64>(6.28318530717958647692);
static constexpr f64 INV_PI_64 = static_cast<f64>(0.31830988618379067153);
static constexpr f64 INV_PI2_64 = static_cast<f64>(0.15915494309189533576);
static constexpr f64 PI_2_64 = static_cast<f64>(1.57079632679489661923);
static constexpr f64 INV_PI_2_64 = static_cast<f64>(0.63661977236758134308);
static constexpr f64 LOG2_64 = static_cast<f64>(0.693147180559945309417);
static constexpr f64 INV_LOG2_64 = static_cast<f64>(1.0 / 0.693147180559945309417);

static constexpr f32 DEG_TO_RAD = static_cast<f32>(1.57079632679489661923 / 90.0);
static constexpr f32 RAD_TO_DEG = static_cast<f32>(90.0 / 1.57079632679489661923);

template<class T>
T lerp(const T& v0, const T& v1, f32 ratio)
{
    return static_cast<T>(v0 + ratio * (v1 - v0));
}

template<class T>
void swap(T& x0, T& x1)
{
    T tmp = x0;
    x0 = x1;
    x1 = tmp;
}

template<class T>
T minimum(const T x0, const T x1)
{
    return x0 < x1 ? x0 : x1;
}

template<class T>
T maximum(const T x0, const T x1)
{
    return x0 < x1 ? x1 : x0;
}

template<class T>
inline T absolute(T val)
{
    return abs(val);
}

template<>
inline u8 absolute<u8>(u8 val)
{
    return val;
}

template<>
inline u16 absolute<u16>(u16 val)
{
    return val;
}

template<>
inline u32 absolute<u32>(u32 val)
{
    return val;
}

template<>
inline f32 absolute<f32>(f32 val)
{
    u32 u = bit_cast<u32, f32>(val);
    u &= 0x7FFFFFFFU;
    return bit_cast<f32, u32>(u);
}

template<>
inline f64 absolute<f64>(f64 val)
{
    return fabs(val);
}

template<class T>
T clamp(const T x, const T minx, const T maxx)
{
    return (x <= maxx) ? ((minx <= x) ? x : minx) : maxx;
}

}
#endif //INC_VDB_H_

