#include "mathLibrary.hpp"
#include <iostream>

// Add a vector2D with a another vector2D
inline Vector2D Vector2D::operator+(Vector2D const b)
{
    return Vector2D{x + b.x, y + b.y};
}

// Substract a Vector2DD with a another Vector2DD
inline Vector2D Vector2D::operator-(Vector2D const b)
{
    return Vector2D{x - b.x, y - b.y};
}

// Multiplposition.y a Vector2D bposition.y a number
inline Vector2D Vector2D::operator*(float multiplier)
{
    Vector2D result = {0, 0};
    result.x = x * multiplier;
    result.y = y * multiplier;
    return result;
}

// Divide a Vector2D bposition.y a number
inline Vector2D Vector2D::operator/(float divider)
{
    x = x / divider;
    y = y / divider;
    return *this;
}

// Substract a Vector2DD with a another Vector2DD
inline Vector2D Vector2D::operator-()
{
    return Vector2D{-x, -y};
}

// Compare two Vector2D value to see if there are equal
inline bool Vector2D::operator==(Vector2D const v2)
{
    if (x != v2.x)
        return false;
    if (y != v2.y)
        return false;

    return true;
}

inline bool Vector2D::operator==(Vector2D const v2) const
{
    if (x != v2.x)
        return false;
    if (y != v2.y)
        return false;

    return true;
}

// Compare two Vector2D value to see if there are different
inline bool Vector2D::operator!=(Vector2D const v2)
{
    if (x != v2.x)
        return true;
    if (y != v2.y)
        return true;

    return false;
}

inline Vector2D Vector2D::operator+=(Vector2D const v2)
{
    this->x += v2.x;
    this->y += v2.y;

    return *this;
}

inline Vector2D Vector2D::operator-=(Vector2D const v2)
{
    this->x -= v2.x;
    this->y -= v2.y;

    return *this;
}

inline const Vector2D Vector2D::operator*(const float multiplier) const
{
    const Vector2D newVec = {x * multiplier, y * multiplier};
    return newVec;
}

// Get the direction of vector with two point
inline Vector2D Vector2D::GetDirection2D(Vector2D PosB)
{
    return PosB - *this;
}

// Dot Product of two Vector2D
inline float Vector2D::DotProduct2D(Vector2D v2)
{
    return x * v2.x + y * v2.y;
}

inline float Vector2D::GetDeterminant2D(Vector2D b)
{
    return x * b.y - y * b.x;
}

inline float Vector2D::magnitude() const
{
    return sqrt(x * x + y * y);
}
inline void Vector2D::normalize()
{
    float dist = this->magnitude();
    x = x / dist;
    y = y / dist;
}

inline Vector2D Vector2D::normal()
{
    return Vector2D{y, -x};
}

inline float DotProduct2D(Vector2D a, Vector2D b)
{
    return a.x * b.x + a.y * b.y;
}

inline Vector2D GetDirection2D(Vector2D posA, Vector2D posB)
{
    return posB - posA;
}

inline float GetDeterminant2D(Vector2D a, Vector2D b)
{
    return a.x * b.y - a.y * b.x;
} 

inline float DotProduct(Vector3D a, Vector3D b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// ------------- Vector 3D -----------------


inline bool Vector3D::operator==(const Vector3D v2)
{
    if (v2.x != x) return false;
    if (v2.y != y) return false;
    if (v2.z != z) return false;

    return true;
}

inline Vector3D Vector3D::operator+(const Vector3D v2)
{
    return Vector3D{x + v2.x, y + v2.y, z + v2.z};
}

inline Vector3D Vector3D::operator-(const Vector3D v2)
{
    return Vector3D{ x - v2.x, y - v2.y, z - v2.z};
}

inline Vector3D Vector3D::operator*(const float multiplier)
{
    return Vector3D{x * multiplier, y * multiplier, z * multiplier};
}
inline Vector3D Vector3D::operator*(const float multiplier) const
{
    return Vector3D{ x * multiplier, y * multiplier, z * multiplier };
}



inline Vector3D Vector3D::operator+=(const  Vector3D v2)
{
    x += v2.x;
    y += v2.y;
    z += v2.z;
    return Vector3D{ x + v2.x, y + v2.y, z + v2.z };
}

inline float Vector3D::GetMagnitude() const
{
    return sqrt(x * x + y * y + z * z);
}

inline void Vector3D::Normalize()
{
    float length = GetMagnitude();
    x = x / length;
    y = y / length;
    z = z / length;
}

// ------------- Vector 4D -----------------

inline Vector4D Vector4D::operator+(const Vector4D v2)
{
    return Vector4D{x + v2.x, y + v2.y,z + v2.z, w + v2.w};
}

inline Vector4D Vector4D::operator-(const Vector4D v2)
{
    return Vector4D{x - v2.x, y - v2.y, z - v2.z, w - v2.w};
}

inline Vector4D Vector4D::operator*(const float multiplier)
{
    return Vector4D{x * multiplier,y * multiplier, z * multiplier, w * multiplier};
}

inline Vector4D &Vector4D::operator=(const Vector3D v2)
{
    x = v2.x;
    y = v2.y;
    z = v2.z;
    w = 1.0f;
    return *this;
}

inline void Vector4D::Homogenize()
{
    if (w != 0)
    {
        x = x / w;
        y = y / w;
        z = z / w;
        w = w / w;
    }
    else
    {
        std::cout << "Divide by zero" << __FILE__ << __LINE__ << std::endl;
    }
}

inline float Vector4D::GetMagnitude() const
{
    return sqrt(x * x + y * y + z * z + w * w);
}

inline void Vector4D::Normalize()
{
    float length = GetMagnitude();
    x = x / length;
    y = y / length;
    z = z / length;
    w = w / length;
}

// -------------- Matrice  4x4 -------------

inline float Matrix4D::operator[](const unsigned int index)
{
        return coord[index%4][index/4];
}


// --------------- Quaternion --------------------

inline Quaternion Quaternion::operator+(const Quaternion m2)
{
    Quaternion q = Quaternion();
    q.scalar = this->scalar + m2.scalar;
    q.imaginaries = this->imaginaries + m2.imaginaries;
    return q;
}

inline void Quaternion::operator+=(const Quaternion m2)
{
    this->scalar += m2.scalar;
    this->imaginaries = this->imaginaries + m2.imaginaries;
}

inline Quaternion Quaternion::operator-(const Quaternion m2)
{
    Quaternion q = Quaternion();
    q.scalar = this->scalar - m2.scalar;
    q.imaginaries = this->imaginaries - m2.imaginaries;
    return q;
}

inline void Quaternion::operator-=(const Quaternion m2)
{
    this->scalar -= m2.scalar;
    this->imaginaries = this->imaginaries - m2.imaginaries;
}

inline Quaternion Quaternion::operator*(const Quaternion m2)
{
    Quaternion q = Quaternion();
    q.scalar = this->scalar * m2.scalar - DotProduct(this->imaginaries, m2.imaginaries);
    q.imaginaries = this->imaginaries * m2.scalar + m2.imaginaries * this->scalar + CrossProduct(this->imaginaries, m2.imaginaries);
    return q;
}

inline void Quaternion::operator*=(const Quaternion m2)
{
    this->scalar = this->scalar * m2.scalar - DotProduct(this->imaginaries, m2.imaginaries);
    this->imaginaries = this->imaginaries * m2.scalar + m2.imaginaries * this->scalar + CrossProduct(this->imaginaries, m2.imaginaries);
}

inline Quaternion Quaternion::operator*(float k)
{
    Quaternion q = Quaternion();
    q.scalar = this->scalar * k;
    q.imaginaries = this->imaginaries * k;
    return q;
}

inline void Quaternion::operator*=(float k)
{
    this->scalar *= k;
    this->imaginaries = this->imaginaries * k;
}

inline Vector3D Quaternion::operator*(Vector3D v2)
{
    Quaternion q = Quaternion(0, v2);
    return (q * *this * q.GetInverseQuat()).imaginaries;
}

inline float Quaternion::GetNorm()
{
    float a = 0;
    for (size_t i = 0; i < 4; i++)
    {
        a += pow(this->coord[i], 2.f);
    }
    return sqrt(a);
}

inline void Quaternion::Normalize()
{
    *this *= 1 / this->GetNorm();
}

inline Quaternion Quaternion::GetNormalizedQuat()
{
    Quaternion q = *this;
    q.Normalize();
    return q;
}

inline void Quaternion::Conjugate()
{
    this->imaginaries = this->imaginaries * -1;
}

inline Quaternion Quaternion::GetConjugatedQuat()
{
    Quaternion q = *this;
    q.Conjugate();
    return q;
}

inline void Quaternion::Inverse()
{
    *this = this->GetConjugatedQuat() * (1 / pow(this->GetNorm(), 2.f)); 
}

inline Quaternion Quaternion::GetInverseQuat()
{
    Quaternion q = *this;
    q.Inverse();
    return q;
}

// --------------- Math Function -----------------
template <typename T>
T Lerp(T start, T end, float t)
{
    return ((end - start) * t) + start;
}
