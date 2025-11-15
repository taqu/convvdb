#ifndef INC_VDB_VECTOR_H_
#define INC_VDB_VECTOR_H_
#include "vdb.h"

namespace vdb
{
    class Vector4;
    class Quaternion;
    class Matrix34;
    class Matrix44;

    //--- Vector2
    //--------------------------------------------
    class Vector2
    {
    public:
        static const Vector2 Zero;
        static const Vector2 One;

        inline Vector2();
        explicit inline Vector2(f32 xy);

        inline Vector2(f32 x, f32 y);

        inline void zero();
        inline void one();
        inline void set(f32 x, f32 y);

        inline f32 operator[](s32 index) const;
        inline f32& operator[](s32 index);
        inline Vector2 operator-() const;

        Vector2& operator+=(const Vector2& x);
        Vector2& operator-=(const Vector2& x);

        Vector2& operator*=(f32 f);
        Vector2& operator/=(f32 f);

        bool isEqual(const Vector2& x) const;
        bool isEqual(const Vector2& x, f32 epsilon) const;

        inline bool operator==(const Vector2& x) const;
        inline bool operator!=(const Vector2& x) const;

        inline f32 length() const;
        f32 lengthSqr() const;

        bool isNan() const;

        f32 x_, y_;
    };

    static_assert(std::is_trivially_copyable<Vector2>::value, "Vector2 should be trivially copyable");

    inline Vector2::Vector2()
    {
    }

    inline Vector2::Vector2(f32 xy)
        :x_(xy)
        ,y_(xy)
    {}

    inline Vector2::Vector2(f32 x, f32 y)
        :x_(x)
        ,y_(y)
    {}

    inline void Vector2::zero()
    {
        x_ = y_ = 0.0f;
    }

    inline void Vector2::one()
    {
        x_ = y_ = 1.0f;
    }

    inline void Vector2::set(f32 x, f32 y)
    {
        x_ = x;
        y_ = y;
    }

    inline f32 Vector2::operator[](s32 index) const
    {
        assert(0<=index && index < 2);
        return (&x_)[index];
    }

    inline f32& Vector2::operator[](s32 index)
    {
        assert(0<=index && index < 2);
        return (&x_)[index];
    }

    inline Vector2 Vector2::operator-() const
    {
        return {-x_, -y_};
    }

    inline bool Vector2::operator==(const Vector2& x) const
    {
        return isEqual(x);
    }

    inline bool Vector2::operator!=(const Vector2& x) const
    {
        return !isEqual(x);
    }

    inline f32 Vector2::length() const
    {
        return ::sqrtf(lengthSqr());
    }

    //--- Vector2's friend functions
    //--------------------------------------------------
    Vector2 normalize(const Vector2& x);

    Vector2 normalizeChecked(const Vector2& x, f32 epsilon=F32_EPSILON);

    Vector2 operator+(const Vector2& x0, const Vector2& x1);

    Vector2 operator-(const Vector2& x0, const Vector2& x1);

    Vector2 operator*(f32 f, const Vector2& x);

    Vector2 operator*(const Vector2& x, f32 f);

    Vector2 operator/(const Vector2& x, f32 f);

    f32 dot(const Vector2& x0, const Vector2& x1);

    f32 distanceSqr(const Vector2& x0, const Vector2& x1);

    f32 distance(const Vector2& x0, const Vector2& x1);

    Vector2 lerp(const Vector2& x0, const Vector2& x1, f32 t);

    Vector2 add(const Vector2& x0, const Vector2& x1);

    Vector2 sub(const Vector2& x0, const Vector2& x1);

    Vector2 mul(f32 f, const Vector2& x);

    Vector2 mul(const Vector2& x, f32 f);

    Vector2 muladd(f32 f, const Vector2& x0, const Vector2& x1);

    Vector2 minimum(const Vector2& x0, const Vector2& x1);

    Vector2 maximum(const Vector2& x0, const Vector2& x1);

    f32 minimum(const Vector2& x);

    f32 maximum(const Vector2& x);

    Vector2 randomOnDisk(f32 x0, f32 x1);

    //--------------------------------------------
    //---
    //--- Vector3
    //---
    //--------------------------------------------
    class Vector3
    {
    public:
        static const Vector3 Zero;
        static const Vector3 One;
        static const Vector3 Forward;
        static const Vector3 Backward;
        static const Vector3 Up;
        static const Vector3 Down;
        static const Vector3 Right;
        static const Vector3 Left;

        Vector3();
        explicit Vector3(f32 xyz);
        Vector3(f32 x, f32 y, f32 z);
        explicit Vector3(const Vector4& x);

        void zero();
        void one();

        void set(f32 x, f32 y, f32 z);

        void set(const Vector4& x);

        inline f32 operator[](s32 index) const;
        inline f32& operator[](s32 index);
        inline Vector3 operator-() const;

        Vector3& operator+=(const Vector3& x);
        Vector3& operator-=(const Vector3& x);

        Vector3& operator*=(f32 f);
        Vector3& operator/=(f32 f);

        Vector3& operator*=(const Vector3& x);
        Vector3& operator/=(const Vector3& x);

        bool isEqual(const Vector3& x) const;
        bool isEqual(const Vector3& x, f32 epsilon) const;

        inline bool operator==(const Vector3& x) const;
        inline bool operator!=(const Vector3& x) const;

        inline f32 length() const;
        f32 lengthSqr() const;

        void swap(Vector3& rhs);

        bool isNan() const;

        f32 x_;
        f32 y_;
        f32 z_;
    };

    static_assert(std::is_trivially_copyable<Vector3>::value, "Vector3 should be trivially copyable");

    inline f32 Vector3::operator[](s32 index) const
    {
        assert(0<=index && index < 3);
        return (&x_)[index];
    }

    inline f32& Vector3::operator[](s32 index)
    {
        assert(0<=index && index < 3);
        return (&x_)[index];
    }

    inline Vector3 Vector3::operator-() const
    {
        return {-x_, -y_, -z_};
    }

    inline bool Vector3::operator==(const Vector3& x) const
    {
        return isEqual(x);
    }

    inline bool Vector3::operator!=(const Vector3& x) const
    {
        return !isEqual(x);
    }

    inline f32 Vector3::length() const
    {
        return ::sqrtf(lengthSqr());
    }

    //--- Vector3's friend functions
    //--------------------------------------------------
    Vector3 operator+(const Vector3& x0, const Vector3& x1);
    Vector3 operator-(const Vector3& x0, const Vector3& x1);
    Vector3 operator*(f32 f, const Vector3& x);
    Vector3 operator*(const Vector3& x, f32 f);
    Vector3 operator*(const Vector3& x0, const Vector3& x1);
    Vector3 operator/(const Vector3& x, f32 f);
    Vector3 operator/(const Vector3& x0, const Vector3& x1);

    Vector3 normalize(const Vector3& x);
    Vector3 normalize(const Vector3& x, f32 lengthSqr);
    Vector3 normalizeChecked(const Vector3& x, const Vector3& fallback=Vector3::Zero, f32 epsilon=F32_EPSILON);

    Vector3 absolute(const Vector3& x);

    Vector3 hadamardMul(const Vector3& x0, const Vector3& x1);

    f32 dot(const Vector3& x0, const Vector3& x1);

    f32 distanceSqr(const Vector3& x0, const Vector3& x1);
    inline f32 distance(const Vector3& x0, const Vector3& x1)
    {
        return ::sqrtf(distanceSqr(x0, x1));
    }

    Vector3 cross(const Vector3& x0, const Vector3& x1);

    /**
    @brief Linear interpolation x = (1-t)*x0 + t*x1
    @param x0 ...
    @param x1 ...
    */
    Vector3 lerp(const Vector3& x0, const Vector3& x1, f32 t);

    void orthonormalBasis(Vector3& b0, Vector3& b1, const Vector3& n);
    Vector3 randomInSphere(f32 x0, f32 x1, f32 x2);
    Vector3 randomOnSphere(f32 x0, f32 x1);
    Vector3 randomOnHemiSphere(f32 x0, f32 x1);
    Vector3 randomOnHemiSphereAround(f32 x0, f32 x1, const Vector3& n);
    Vector3 randomOnCosineHemiSphere(f32 x0, f32 x1);
    Vector3 randomOnCosineHemiSphereAround(f32 x0, f32 x1, const Vector3& n);
    Vector3 randomCone(f32 x0, f32 x1, f32 cosCutoff);
    Vector3 reflect(const Vector3& x, const Vector3& n);
    bool refract(Vector3& refracted, const Vector3& x, const Vector3& n, f32 niOverNt);

    inline Vector3 mul(f32 f, const Vector3& x)
    {
        return f*x;
    }

    inline Vector3 mul(const Vector3& x, f32 f)
    {
        return x*f;
    }

    Vector3 mul(const Matrix34& m, const Vector3& x);
    Vector3 mul(const Vector3& x, const Matrix34& m);

    Vector3 mul33(const Matrix34& m, const Vector3& x);
    Vector3 mul33(const Vector3& x, const Matrix34& m);

    Vector3 mul33(const Matrix44& m, const Vector3& x);
    Vector3 mul33(const Vector3& x, const Matrix44& m);

    Vector3 rotate(const Vector3& x, const Quaternion& rotation);
    Vector3 rotate(const Quaternion& rotation, const Vector3& x);


    inline Vector3 add(const Vector3& x0, const Vector3& x1)
    {
        return x0+x1;
    }

    inline Vector3 sub(const Vector3& x0, const Vector3& x1)
    {
        return x0-x1;
    }

    Vector3 mul(const Vector3& x0, const Vector3& x1);
    Vector3 div(const Vector3& x0, const Vector3& x1);


    Vector3 minimum(const Vector3& x0, const Vector3& x1);

    Vector3 maximum(const Vector3& x0, const Vector3& x1);

    f32 minimum(const Vector3& x);

    f32 maximum(const Vector3& x);

    // x0*x1 + v2
    Vector3 muladd(const Vector3& x0, const Vector3& x1, const Vector3& v2);

    // a*x1 + v2
    Vector3 muladd(f32 a, const Vector3& x0, const Vector3& x1);


    //--- Vector4
    //--------------------------------------------
    class Vector4
    {
    public:
        static const Vector4 Zero;
        static const Vector4 One;
        static const Vector4 Identity;
        static const Vector4 Forward;
        static const Vector4 Backward;
        static const Vector4 Up;
        static const Vector4 Down;
        static const Vector4 Right;
        static const Vector4 Left;

        Vector4();
        explicit Vector4(f32 xyzw);
        Vector4(f32 x, f32 y, f32 z);
        Vector4(f32 x, f32 y, f32 z, f32 w);
        explicit Vector4(const Vector3& x);
        Vector4(const Vector3& x, f32 w);

        void zero();
        void one();
        void identity();

        f32 length() const;
        f32 lengthSqr() const;

        inline f32 operator[](s32 index) const;
        inline f32& operator[](s32 index);

        Vector4 operator-() const;

        Vector4& operator+=(const Vector4& x);
        Vector4& operator-=(const Vector4& x);

        Vector4& operator*=(f32 f);
        Vector4& operator/=(f32 f);

        Vector4& operator*=(const Vector4& x);
        Vector4& operator/=(const Vector4& x);

        bool isEqual(const Vector4& x) const;
        bool isEqual(const Vector4& x, f32 epsilon) const;

        inline bool operator==(const Vector4& x) const;
        inline bool operator!=(const Vector4& x) const;

        void swap(Vector4& rhs);

        bool isNan() const;
        bool isZero() const;

        f32 x_, y_, z_, w_;
    };

    static_assert(std::is_trivially_copyable<Vector4>::value, "Vector4 should be trivially copyable");

    inline f32 Vector4::operator[](s32 index) const
    {
        assert(0<=index && index < 4);
        return (&x_)[index];
    }

    inline f32& Vector4::operator[](s32 index)
    {
        assert(0<=index && index < 4);
        return (&x_)[index];
    }

    inline bool Vector4::operator==(const Vector4& x) const
    {
        return isEqual(x);
    }

    inline bool Vector4::operator!=(const Vector4& x) const
    {
        return !isEqual(x);
    }

    //--- Vector4's friend functions
    //--------------------------------------------------
    Vector4 hadamardMul(const Vector4& x0, const Vector4& x1);
    f32 dot(const Vector4& x0, const Vector4& x1);

    Vector4 normalize(const Vector4& x);
    Vector4 normalize(const Vector4& x, f32 lengthSqr);
    Vector4 normalizeChecked(const Vector4& x, const Vector4& fallback = Vector4::Zero, f32 epsilon=F32_EPSILON);
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
}
#endif //INC_VDB_VECTOR_H_

