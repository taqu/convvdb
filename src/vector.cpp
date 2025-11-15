#include "vector.h"
#include <algorithm>

namespace vdb
{
//--- Vector2
//---------------------------------------------
const Vector2 Vector2::Zero = {0, 0};
const Vector2 Vector2::One = {1, 1};

f32 Vector2::lengthSqr() const
{
    return x_*x_ + y_*y_;
}

f32 Vector2::length() const
{
    return std::sqrtf(lengthSqr());
}

Vector2& Vector2::operator+=(const Vector2& x)
{
    x_ += x.x_;
    y_ += x.y_;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& x)
{
    x_ -= x.x_;
    y_ -= x.y_;
    return *this;
}

Vector2& Vector2::operator*=(f32 x)
{
    x_ *= x;
    y_ *= x;
    return *this;
}

Vector2& Vector2::operator/=(f32 x)
{
    x_ /= x;
    y_ /= x;
    return *this;
}

Vector2 normalize(const Vector2& x)
{
    f32 l = x.length();
    l = 1.0f / l;
    return {x.x_*l, x.y_*l};
}

Vector2 normalizeChecked(const Vector2& x, f32 epsilon)
{
    f32 l = x.length();
    if(l <= epsilon) {
        return {};
    }
    l = 1.0f / l;
    return {x.x_*l, x.y_*l};
}

Vector2 operator+(const Vector2& x0, const Vector2& x1)
{
    return {x0.x_+x1.x_, x0.y_+x1.y_};
}

Vector2 operator-(const Vector2& x0, const Vector2& x1)
{
    return {x0.x_-x1.x_, x0.y_-x1.y_};
}

Vector2 operator*(const Vector2& x0, const Vector2& x1)
{
    return {x0.x_*x1.x_, x0.y_*x1.y_};
}

Vector2 operator*(f32 x0, const Vector2& x1)
{
    return {x0*x1.x_, x0*x1.y_};
}

Vector2 operator*(const Vector2& x0, f32 x1)
{
    return {x0.x_*x1, x0.y_*x1};
}

Vector2 operator/(const Vector2& x0, const Vector2& x1)
{
    return {x0.x_/x1.x_, x0.y_/x1.y_};
}

Vector2 operator/(const Vector2& x0, f32 x1)
{
    return {x0.x_/x1, x0.y_/x1};
}

f32 dot(const Vector2& x0, const Vector2& x1)
{
    return x0.x_*x1.x_ + x0.y_*x1.y_;
}

f32 distanceSqr(const Vector2& x0, const Vector2& x1)
{
    return x0.x_*x0.x_ + x0.y_*x0.y_;
}

    f32 distance(const Vector2& x0, const Vector2& x1)
{
        return ::sqrtf(x0.x_*x0.x_ + x0.y_*x0.y_);
}

    Vector2 lerp(const Vector2& x0, const Vector2& x1, f32 t)
{
        return {x0.x_ + (x1.x_ - x0.x_) * t, x0.y_ + (x1.y_ - x0.y_) * t};
}

    Vector2 add(const Vector2& x0, const Vector2& x1)
{
        return {x0.x_ + x1.x_, x0.y_ + x1.y_};
}

    Vector2 sub(const Vector2& x0, const Vector2& x1)
{
        return {x0.x_ - x1.x_, x0.y_ - x1.y_};
}

    Vector2 mul(f32 f, const Vector2& x)
{
        return {x.x_ * f, x.y_ * f};
}

    Vector2 mul(const Vector2& x, f32 f)
{
        return {x.x_ * f, x.y_ * f};
}

    Vector2 muladd(f32 f, const Vector2& x0, const Vector2& x1)
{
        return {x0.x_ + x1.x_ * f, x0.y_ + x1.y_ * f};
}

    Vector2 minimum(const Vector2& x0, const Vector2& x1)
{
        return {x0.x_ < x1.x_ ? x0.x_ : x1.x_, x0.y_ < x1.y_ ? x0.y_ : x1.y_};
}

    Vector2 maximum(const Vector2& x0, const Vector2& x1)
{
        return {x0.x_ > x1.x_ ? x0.x_ : x1.x_, x0.y_ > x1.y_ ? x0.y_ : x1.y_};
    }

    f32 minimum(const Vector2& x)
    {
        return x.x_ < x.y_ ? x.x_ : x.y_;
    }

    f32 maximum(const Vector2& x)
    {
        return x.x_ > x.y_ ? x.x_ : x.y_;
    }

Vector2 randomOnDisk(f32 x0, f32 x1)
{
    f32 r0 = 2.0f * x0 - 1.0f;
    f32 r1 = 2.0f * x1 - 1.0f;
    f32 absR0 = std::abs(r0);
    f32 absR1 = std::abs(r1);
    f32 phi;
    f32 r;
    if(absR0 <= F32_EPSILON && absR1 <= F32_EPSILON) {
        r = 0.0f;
        phi = 0.0f;
    } else if(absR1 < absR0) {
        r = r0;
        phi = (PI / 4.0f) * (r1 / r0);
    } else {
        r = r1;
        phi = (PI / 2.0f) - (r0 / r1) * (PI / 4.0f);
    }
    return {r * std::cosf(phi), r * std::sinf(phi)};
}

//--- Vector3
//---------------------------------------------
const Vector3 Vector3::Zero = {0.0f, 0.0f, 0.0f};
const Vector3 Vector3::One = {1.0f, 1.0f, 1.0f};
const Vector3 Vector3::Forward = {0.0f, 0.0f, 1.0f};
const Vector3 Vector3::Right = {1.0f, 1.0f, 1.0f};
const Vector3 Vector3::Up = {0.0f, 1.0f, 0.0f};

Vector3::Vector3()
{
}

        Vector3::Vector3(f32 xyz)
    :x_(xyz)
            , y_(xyz)
            , z_(xyz)
{
}

        Vector3::Vector3(f32 x, f32 y, f32 z)
:x_(x)
            , y_(y)
            , z_(z)
{
}

        Vector3::Vector3(const Vector4& x)
    :x_(x.x_)
            , y_(x.y_)
            , z_(x.z_)
{
}

        void Vector3::set(f32 x, f32 y, f32 z)
{
            x_ = x;
            y_ = y;
            z_ = z;
}

        void Vector3::set(const Vector4& x)
{
            x_ = x.x_;
            y_ = x.y_;
            z_ = x.z_;
}

f32 Vector3::lengthSqr() const
{
    return x_*x_ + y_*y_ + z_*z_;
}

f32 Vector3::length() const
{
    return std::sqrtf(lengthSqr());
}

Vector3& Vector3::operator+=(const Vector3& x)
{
    x_ += x.x_;
    y_ += x.y_;
    z_ += x.z_;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& x)
{
    x_ -= x.x_;
    y_ -= x.y_;
    z_ -= x.z_;
    return *this;
}

Vector3& Vector3::operator*=(const Vector3& x)
{
    x_ *= x.x_;
    y_ *= x.y_;
    z_ *= x.z_;
    return *this;
}

Vector3& Vector3::operator/=(const Vector3& x)
{
    x_ /= x.x_;
    y_ /= x.y_;
    z_ /= x.z_;
    return *this;
}

Vector3& Vector3::operator*=(f32 x)
{
    x_ *= x;
    y_ *= x;
    z_ *= x;
    return *this;
}

Vector3& Vector3::operator/=(f32 x)
{
    x_ /= x;
    y_ /= x;
    z_ /= x;
    return *this;
}

Vector3 operator+(const Vector3& x0, const Vector3& x1)
{
    return {x0.x_+x1.x_, x0.y_+x1.y_, x0.z_+x1.z_};
}

Vector3 operator-(const Vector3& x0, const Vector3& x1)
{
    return {x0.x_-x1.x_, x0.y_-x1.y_, x0.z_-x1.z_};
}

Vector3 operator*(const Vector3& x0, const Vector3& x1)
{
    return {x0.x_*x1.x_, x0.y_*x1.y_, x0.z_*x1.z_};
}

Vector3 operator*(f32 x0, const Vector3& x1)
{
    return {x0*x1.x_, x0*x1.y_, x0*x1.z_};
}

Vector3 operator*(const Vector3& x0, f32 x1)
{
    return {x0.x_*x1, x0.y_*x1, x0.z_*x1};
}

Vector3 operator/(const Vector3& x0, const Vector3& x1)
{
    return {x0.x_/x1.x_, x0.y_/x1.y_, x0.z_/x1.z_};
}

Vector3 operator/(const Vector3& x0, f32 x1)
{
    return {x0.x_/x1, x0.y_/x1, x0.z_/x1};
}

Vector3 normalize(const Vector3& x)
{
    f32 l = x.length();
    l = 1.0f / l;
    return {x.x_*l, x.y_*l, x.z_*l};
}

    Vector3 normalize(const Vector3& x, f32 lengthSqr)
{
    f32 l = 1.0f / std::sqrtf(lengthSqr);
    return {x.x_*l, x.y_*l, x.z_*l};
}

    Vector3 normalizeChecked(const Vector3& x, const Vector3& fallback=Vector3::Zero, f32 epsilon)
{
        f32 l = x.length();
    if(l <= epsilon) {
        return fallback;
    }
    l = 1.0f / l;
    return {x.x_*l, x.y_*l, x.z_*l};
}

    Vector3 absolute(const Vector3& x)
{
        return {std::fabsf(x.x_), std::fabsf(x.y_), std::fabsf(x.z_)};
}

Vector3 hadamardMul(const Vector3& x0, const Vector3& x1)
{
    return {x0.x_ * x1.x_, x0.y_ * x1.y_, x0.z_ * x1.z_};
}

f32 dot(const Vector3& x0, const Vector3& x1)
{
    return x0.x_ * x1.x_ + x0.y_ * x1.y_ + x0.z_ * x1.z_;
}

f32 distanceSqr(const Vector3& x0, const Vector3& x1)
{
    f32 d0 = x0.x_ - x1.x_;
    f32 d1 = x0.y_ - x1.y_;
    f32 d2 = x0.z_ - x1.z_;
    return d0 * d0 + d1 * d1 + d2 * d2;
}

Vector3 cross(const Vector3& x0, const Vector3& x1)
{
    f32 x = x0.y_ * x1.z_ - x0.z_ * x1.y_;
    f32 y = x0.z_ * x1.x_ - x0.x_ * x1.z_;
    f32 z = x0.x_ * x1.y_ - x0.y_ * x1.x_;
    return {x, y, z};
}

Vector3 minimum(const Vector3& x0, const Vector3& x1)
{
    f32 x = minimum(x0.x_, x1.x_);
    f32 y = minimum(x0.y_, x1.y_);
    f32 z = minimum(x0.z_, x1.z_);
    return {x, y, z};
}

Vector3 maximum(const Vector3& x0, const Vector3& x1)
{
    f32 x = maximum(x0.x_, x1.x_);
    f32 y = maximum(x0.y_, x1.y_);
    f32 z = maximum(x0.z_, x1.z_);
    return {x, y, z};
}

Vector3 lerp(const Vector3& x0, const Vector3& x1, f32 t)
{
    assert(0.0f <= t && t <= 1.0f);
    f32 t0 = 1.0f - t;
    f32 t1 = t;
    return x0 * t0 + x1 * t1;
}

void orthonormalBasis(Vector3& b0, Vector3& b1, const Vector3& n)
{
    if(n.z_ < -0.999999f) {
        b0 = {0, -1, 0};
        b1 = {-1, 0, 0};
        return;
    }
    f32 a = 1.0f / (1.0f + n.z_);
    f32 b = -n.x_ * n.y_ * a;
    b0 = {1.0f - n.x_ * n.x_ * a, b, -n.x_};
    b1 = {b, 1.0f - n.y_ * n.y_ * a, -n.y_};
}

Vector3 randomInSphere(f32 x0, f32 x1, f32 x2)
{
    f32 theta = 2.0f * x0 - 1.0f;
    f32 r = ::sqrtf(1.0f - theta * theta);
    f32 phi = (PI * 2.0f) * x1;
    f32 sn = ::sinf(phi);
    f32 cs = ::cosf(phi);
    r *= x2;
    return {r * cs, r * sn, x2 * theta};
}

Vector3 randomOnSphere(f32 x0, f32 x1)
{
    f32 theta = 2.0f * x0 - 1.0f;
    f32 r = ::sqrtf(1.0f - theta * theta);
    f32 phi = (PI * 2.0f) * x1;
    f32 sn = ::sinf(phi);
    f32 cs = ::cosf(phi);
    return {r * cs, r * sn, theta};
}

Vector3 randomOnHemiSphere(f32 x0, f32 x1)
{
    f32 theta = x0;
    f32 r = ::sqrtf(1.0f - theta * theta);
    f32 phi = (PI * 2.0f) * x1;
    f32 sn = ::sinf(phi);
    f32 cs = ::cosf(phi);
    return {r * cs, r * sn, theta};
}

Vector3 randomOnHemiSphereAround(f32 x0, f32 x1, const Vector3& n)
{
    Vector3 x = randomOnHemiSphere(x0, x1);
    f32 t = dot(x, n);
    if(t < 0.0f) {
        return -x;
    }
    return x;
}

Vector3 randomOnCosineHemiSphere(f32 x0, f32 x1)
{
    Vector2 p = randomOnDisk(x0, x1);
    f32 z = std::max(0.0f, ::sqrtf(std::max(F32_EPSILON, 1.0f - p.x_ * p.x_ - p.y_ * p.y_)));
    return {p.x_, p.y_, z};
}

Vector3 randomOnCosineHemiSphereAround(f32 x0, f32 x1, const Vector3& n)
{
    Vector3 x = randomOnCosineHemiSphere(x0, x1);
    Vector3 b0, b1;
    orthonormalBasis(b0, b1, n);
    f32 rx = x.x_ * b0.x_ + x.y_ * n.x_ + x.z_ * b1.x_;
    f32 ry = x.x_ * b0.y_ + x.y_ * n.y_ + x.z_ * b1.y_;
    f32 rz = x.x_ * b0.z_ + x.y_ * n.z_ + x.z_ * b1.z_;
    return {rx, ry, rz};
}

Vector3 randomCone(f32 x0, f32 x1, f32 cosCutoff)
{
    f32 cosTheta = (1.0f - x0) + x0 * cosCutoff;
    f32 sinTheta = ::sqrtf(std::max(F32_EPSILON, (1.0f - cosTheta * cosTheta)));
    f32 phi = 2.0f * PI * x1;
    f32 sinPhi = ::sinf(phi);
    f32 cosPhi = ::cosf(phi);
    return {cosPhi * sinTheta, sinPhi * sinTheta, cosTheta};
}

Vector3 reflect(const Vector3& x, const Vector3& n)
{
    return x - 2.0f * dot(x, n) * n;
}

bool refract(Vector3& refracted, const Vector3& x, const Vector3& n, f32 niOverNt)
{
    f32 dt = dot(x, n);
    f32 discriminant = 1.0f - niOverNt * niOverNt * (1.0f - dt * dt);
    if(0.0f < discriminant) {
        refracted = (niOverNt * (x - dt * n)) - ::sqrtf(discriminant) * n;
        return true;
    }
    return false;
}

//--- Vector4
//---------------------------------------------
const Vector4 Vector4::Zero = {0, 0, 0, 0};
const Vector4 Vector4::One = {1, 1, 1, 1};
const Vector4 Vector4::Identity = {0, 0, 0, 1};
const Vector4 Vector4::Forward = {0, 0, 1, 0};
const Vector4 Vector4::Backward = {0, 0, -1, 0};
const Vector4 Vector4::Up = {0, 1, 0, 0};
const Vector4 Vector4::Down = {0, -1, 0, 0};
const Vector4 Vector4::Right = {1, 0, 0, 0};
const Vector4 Vector4::Left = {-1, 0, 0, 0};

Vector4::Vector4()
{
}

        Vector4::Vector4(f32 xyzw)
    : x_(xyzw)
            , y_(xyzw)
            , z_(xyzw)
            , w_(xyzw)
{
}

        Vector4::Vector4(f32 x, f32 y, f32 z)
    : x_(x)
            , y_(y)
            , z_(z)
            , w_(0)
{
}

        Vector4::Vector4(f32 x, f32 y, f32 z, f32 w)
    : x_(x)
            , y_(y)
            , z_(z)
            , w_(w)
{
}

        Vector4::Vector4(const Vector3& x)
    : x_(x.x_)
            , y_(x.y_)
            , z_(x.z_)
            , w_(0)
{
}

        Vector4::Vector4(const Vector3& x, f32 w)
    : x_(x.x_)
            , y_(x.y_)
            , z_(x.z_)
            , w_(w)
{
}

        void Vector4::zero()
{
    x_ = 0;
    y_ = 0;
    z_ = 0;
    w_ = 0;
}

        void Vector4::one()
{
    x_ = 1;
    y_ = 1;
    z_ = 1;
    w_ = 1;
}
        void Vector4::identity()
{
x_ = 0;
y_ = 0;
z_ = 0;
w_ = 1;
}


Vector4& Vector4::operator+=(const Vector4& x)
{
    x_ += x.x_;
    y_ += x.y_;
    z_ += x.z_;
    w_ += x.w_;
    return *this;
}

Vector4& Vector4::operator-=(const Vector4& x)
{
    x_ -= x.x_;
    y_ -= x.y_;
    z_ -= x.z_;
    w_ -= x.w_;
    return *this;
}

Vector4& Vector4::operator*=(const Vector4& x)
{
    x_ *= x.x_;
    y_ *= x.y_;
    z_ *= x.z_;
    w_ *= x.w_;
    return *this;
}

Vector4& Vector4::operator/=(const Vector4& x)
{
    x_ /= x.x_;
    y_ /= x.y_;
    z_ /= x.z_;
    w_ /= x.w_;
    return *this;
}

Vector4& Vector4::operator*=(f32 x)
{
    x_ *= x;
    y_ *= x;
    z_ *= x;
    w_ *= x;
    return *this;
}

Vector4& Vector4::operator/=(f32 x)
{
    x_ /= x;
    y_ /= x;
    z_ /= x;
    w_ /= x;
    return *this;
}

Vector4 operator+(const Vector4& x0, const Vector4& x1)
{
    return {x0.x_+x1.x_, x0.y_+x1.y_, x0.z_+x1.z_, x0.w_+x1.w_};
}

Vector4 operator-(const Vector4& x0, const Vector4& x1)
{
    return {x0.x_-x1.x_, x0.y_-x1.y_, x0.z_-x1.z_, x0.w_-x1.w_};
}

Vector4 operator*(const Vector4& x0, const Vector4& x1)
{
    return {x0.x_*x1.x_, x0.y_*x1.y_, x0.z_*x1.z_, x0.w_*x1.w_};
}

Vector4 operator*(f32 x0, const Vector4& x1)
{
    return {x0*x1.x_, x0*x1.y_, x0*x1.z_, x0*x1.w_};
}

Vector4 operator*(const Vector4& x0, f32 x1)
{
    return {x0.x_*x1, x0.y_*x1, x0.z_*x1, x0.w_*x1};
}

Vector4 operator/(const Vector4& x0, const Vector4& x1)
{
    return {x0.x_/x1.x_, x0.y_/x1.y_, x0.z_/x1.z_, x0.w_/x1.w_};
}

Vector4 operator/(const Vector4& x0, f32 x1)
{
    return {x0.x_/x1, x0.y_/x1, x0.z_/x1, x0.w_/x1};
}

Vector4 hadamardMul(const Vector4& x0, const Vector4& x1)
{
    return {x0.x_ * x1.x_, x0.y_ * x1.y_, x0.z_ * x1.z_, x0.w_ * x1.w_};
}

f32 dot(const Vector4& x0, const Vector4& x1)
{
    return x0.x_*x1.x_ + x0.y_*x1.y_ + x0.z_*x1.z_ + x0.w_*x1.w_;
}

Vector4 normalize(const Vector4& x)
{
    f32 l = x.length();
    l = 1.0f / l;
    return {x.x_*l, x.y_*l, x.z_*l, x.w_*l};
}

Vector4 normalize(const Vector4& x, f32 lengthSqr)
{
    f32 l = 1.0f / std::sqrtf(lengthSqr);
    return {x.x_*l, x.y_*l, x.z_*l, x.w_*l};
}

Vector4 normalizeChecked(const Vector4& x, const Vector4& fallback, f32 epsilon)
{
    f32 l = x.length();
    if(l <= epsilon) {
        return fallback;
    }
    l = 1.0f / l;
    return {x.x_*l, x.y_*l, x.z_*l, x.w_*l};
}

Vector4 absolute(const Vector4& x);

    Vector4 cross3(const Vector4& x0, const Vector4& x1);
    f32 distanceSqr(const Vector4& x0, const Vector4& x1);
    f32 distance(const Vector4& x0, const Vector4& x1);

    f32 distanceSqr3(const Vector3& x0, const Vector4& x1);
    inline f32 distanceSqr3(const Vector4& x0, const Vector3& x1)
    {
        return distanceSqr3(x1, x0);
    }
    f32 distanceSqr3(const Vector4& x0, const Vector4& x1);
    f32 distance3(const Vector3& x0, const Vector4& x1);
    inline f32 distance3(const Vector4& x0, const Vector3& x1)
    {
        return distance3(x1, x0);
    }
    f32 distance3(const Vector4& x0, const Vector4& x1);

    Vector4 mul(const Matrix34& m, const Vector4& x);
    Vector4 mul(const Vector4& x, const Matrix34& m);

    Vector4 mul(const Matrix44& m, const Vector4& x);
    Vector4 mul(const Vector4& x, const Matrix44& m);

    Vector4 mulPoint(const Matrix44& m, const Vector4& x);
    Vector4 mulVector(const Matrix44& m, const Vector4& x);

    Vector4 rotate(const Vector4& x, const Quaternion& rotation);
    Vector4 rotate(const Quaternion& rotation, const Vector4& x);

    Vector4 operator+(const Vector4& x0, const Vector4& x1);
    Vector4 operator-(const Vector4& x0, const Vector4& x1);
    Vector4 operator*(f32 f, const Vector4& x);

    inline Vector4 operator*(const Vector4& x, f32 f)
    {
        return f*x;
    }

    Vector4 operator*(const Vector4& x0, const Vector4& x1);

    Vector4 operator/(const Vector4& x, f32 f);
    Vector4 operator/(const Vector4& x0, const Vector4& x1);

    inline Vector4 add(const Vector4& x0, const Vector4& x1)
    {
        return x0+x1;
    }

    inline Vector4 sub(const Vector4& x0, const Vector4& x1)
    {
        return x0-x1;
    }

    inline Vector4 mul(f32 f, const Vector4& x)
    {
        return f*x;
    }

    inline Vector4 mul(const Vector4& x, f32 f)
    {
        return x*f;
    }

    inline Vector4 div(const Vector4& x0, f32 f)
    {
        return x0/f;
    }

    Vector4 add(const Vector4& x, f32 f);
    Vector4 sub(const Vector4& x, f32 f);

    Vector4 minimum(const Vector4& x0, const Vector4& x1);
    Vector4 maximum(const Vector4& x0, const Vector4& x1);

    f32 minimum(const Vector4& x);
    f32 maximum(const Vector4& x);

    /**
    @brief x0*x1 + v2
    */
    Vector4 muladd(const Vector4& x0, const Vector4& x1, const Vector4& v2);

    /**
    @brief a*x0 + x1
    */
    Vector4 muladd(f32 a, const Vector4& x0, const Vector4& x1);

    /**
    @brief x0 * (1-t) + x1 * t
    */
    Vector4 lerp(const Vector4& x0, const Vector4& x1, f32 t);

} // namespace ray
