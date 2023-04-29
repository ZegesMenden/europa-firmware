#pragma once

#ifndef ARDUINO
#include <math.h>
#else 
#include <Arduino.h>
#endif

#define EARTH_RADIUS 6371000.0

inline double DBP(double la1, double lo1, double la2, double lo2)
{
    double dlat = (la2 - la1);
    double dlon = (lo2 - lo1);
    double half_dlon = dlon / 2.0;
    double half_dlat = dlat / 2.0;
    double a = sin(half_dlat) * sin(half_dlat) + cos(la1) * cos(la2) * sin(half_dlon) * sin(half_dlon);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

inline double CBP(double la1, double lo1, double la2, double lo2)
{
    double d_lo = (lo2-lo1);
    double y = sin(d_lo) * cos(la2);
    double x = cos(la1) * sin(la2) - sin(la1) * cos(la2) * cos(d_lo);

    return atan2(y,x);
}

inline float DBPf(float la1, float lo1, float la2, float lo2)
{
    float dlat = (la2 - la1);
    float dlon = (lo2 - lo1);
    float half_dlon = dlon / 2.0;
    float half_dlat = dlat / 2.0;
    float sin_dlat = sinf(half_dlat);
    float sin_dlon = sinf(half_dlon);
    float a = sin_dlat*sin_dlat + cosf(la1) * cosf(la2) * sin_dlon*sin_dlon;
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return EARTH_RADIUS * c;
}

inline float CBPf(float la1, float lo1, float la2, float lo2)
{
    float d_lo = (lo2-lo1);
    float cos_la2 = cosf(la2);
    float y = sinf(d_lo) * cos_la2;
    float x = cosf(la1) * sinf(la2) - sinf(la1) * cos_la2 * cosf(d_lo);

    return atan2f(y,x);
}

template <class T = float>
class vec3
{
public:

    T x, y, z;

    /**
     * @brief Construct a new vec3 object
     */
    vec3() { x = y = z = 0.0; }

    /**
     * @brief Construct a new vec3 object
     *
     * @param x x component
     * @param y y component
     * @param z z component
     */
    vec3(T x, T y, T z) { this->x = x, this->y = y, this->z = z; }

    template<class _t>
    operator vec3<_t>() const { return vec3<_t>( (_t)this->x, (_t)this->y, (_t)this->z ); }

    /**
     * @brief Adds the right handed vector to the left handed vector
     * @return sum of the two vectors
     */
    vec3& operator+=(const vec3& v) {
        this->x += v.x;
        this->y += v.y;
        this->z += v.z;

        return(*this);
    };

    /**
     * @brief Adds two vectors
     * @return sum of the two vectors
     */
    vec3& operator+(const vec3& v) const { return vec3(*this) += v; }

    /**
     * @brief Subtracts the right handed vector from the left handed vector
     * @return difference of the two vectors
     */
    vec3& operator-=(const vec3& v) {
        this->x -= v.x;
        this->y -= v.y;
        this->z -= v.z;

        return(*this);
    };

    /**
     * @brief Adds the right handed vector to the left handed vector
     * @return sum of the two vectors
     */
    const vec3 operator-(const vec3& v) const { return vec3(*this) -= v; }

    const vec3& operator- () const { return vec3(-this->x, -this->y, -this->z); }

    /**
     * @brief Multiplies the left handed vector by the right handed vector
     * @return product of the two vectors
     */
    vec3& operator*=(const vec3& v) {
        this->x *= v.x;
        this->y *= v.y;
        this->z *= v.z;

        return(*this);
    };

    /**
     * @brief Multiplies two vectors
     * @return product of the two vectors
     */
    const vec3& operator*(const vec3& v) const { return vec3(*this) *= v; }

    /**
     *  @brief Multiplies the left handed vector by a scalar
     * @return product of the vector and scalar
     */
    vec3& operator*=(T scale) {
        this->x *= scale;
        this->y *= scale;
        this->z *= scale;

        return(*this);
    };

    /**
     * @brief Multiplies a vector by a scalar
     * @return product of the vector and scalar
     */
    const vec3 operator*(T scale) const { return vec3(*this) *= scale; }

    /**
     * @brief Divides the left handed vector by the right handed vector
     * @return quotient of the two vectors
     */
    vec3& operator/=(const vec3& v) {
        this->x /= v.x;
        this->y /= v.y;
        this->z /= v.z;

        return(*this);
    };

    /**
     * @brief Divides two vectors
     * @return quotient of the two vectors
     */
    const vec3 operator/(const vec3& v) const { return vec3(this->x/v.x, this->y/v.y, this->z/v.z); }

    /**
     * @brief Divides the left handed vector by a scalar
     * @return quotient of the vector and scalar
     */
    vec3& operator/=(T scale) {
        this->x /= scale;
        this->y /= scale;
        this->z /= scale;

        return(*this);
    };

    /**
     * @brief Divides a vector by a scalar
     * @return quotient of the vector and scalar
     */
    const vec3 operator/(T scale) const { return vec3(*this) /= scale; }

    /**
     * @brief Normalizes the vector
     * @return normalized vector
     */
    vec3& norm() {
        T normal = len();

        if ( normal != 0.0 )
        {
            this->x /= normal;
            this->y /= normal;
            this->z /= normal;
        }

        return(*this);
    };

    /**
     * @brief Returns the length of the vector
     * @return length of the vector
     */
    inline T len() { return(sqrtf(x * x + y * y + z * z)); };

    /**
     * @brief Returns the cross product of the left and right hand vectors
     * @return cross product of the two vectors
     */
    vec3 cross(const vec3& v) {
        vec3<T> vNew = vec3<T>( this->y * v.z - this->z * v.y, 
                                this->z * v.x - this->x * v.z, 
                                this->x * v.y - this->y * v.x);
        return (vNew);
    }

    /**
     * @brief Returns the dot product of the left and right hand vectors
     * @return dot product of the two vectors
     */
    T dot(const vec3& v) {
        return (this->x * v.x + this->y * v.y + this->z * v.z);
    }

};

template <class T = float>
class quat
{

public:
    T w, x, y, z;

    /**
     * @brief Constructs a new quat object
     */
    quat()
    {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    /**
     * @brief Constructs a new quat object
     *
     * @param w w component
     * @param x x component
     * @param y y component
     * @param z z component
     */
    quat(float sw, float sx, float sy, float sz)
    {
        w = sw;
        x = sx;
        y = sy;
        z = sz;
    } // initializer with varibles

    /**
     * @brief Sets the quaternion equal to another quaternion
     *
     * @param other quat to copy from
     */
    quat& operator=(const quat& other) // assignment operator
    {
        w = other.w;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    /**
     * @brief Adds the right hand quaternion to the left hand quaternion
     *
     * @param other quat to add
     * @return quat sum
     */
    quat& operator+=(const quat& other) {

        w += other.w;
        x += other.x;
        y += other.y;
        z += other.z;

        return (*this);
    };

    /**
     * @brief Adds two quaternions
     *
     * @param other quat to add
     * @return quat sum
     */
    const quat operator+(const quat& other) const { return quat(*this) += other; } // addition operators

    /**
     * @brief Subtracts the right hand quaternion from the left hand quaternion
     *
     * @param other quat to subtract
     * @return quat difference
     */
    quat& operator-=(const quat& other) {
        w -= other.w;
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return (*this);
    };

    /**
     * @brief Subtracts two quaternions
     *
     * @param other quat to subtract
     * @return quat difference
     */
    const quat operator-(const quat& other) const { return quat(*this) -= other; } // subtraction operators

    /**
     * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    inline quat& operator*=(const quat& other) {
        quat<T> qNew;

        qNew.x = this->x * other.w + this->y * other.z - this->z * other.y + this->w * other.x;
        qNew.y = -this->x * other.z + this->y * other.w + this->z * other.x + this->w * other.y;
        qNew.z = this->x * other.y - this->y * other.x + this->z * other.w + this->w * other.z;
        qNew.w = -this->x * other.x - this->y * other.y - this->z * other.z + this->w * other.w;

        return (*this = qNew);
    }; // multiplication operators

    /**
     * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    const inline quat operator*(const quat& other) const { return quat(*this) *= other; }

    /**
     * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    inline quat& operator*=(const vec3<T>& other) {
        quat<T> qNew;

        qNew.w = -this->x * other.x - this->y * other.y - this->z * other.z;
        qNew.x =  this->w * other.x + this->y * other.z - this->z * other.y;
        qNew.y =  this->w * other.y - this->x * other.z + this->z * other.x;
        qNew.z =  this->w * other.z + this->x * other.y - this->y * other.x;

        return (*this = qNew);
    }; // multiplication operators

    /**
     * @brief Computes the multiplication of a quaternion by a vector
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    const inline quat operator*(const vec3<T>& other) const { return quat(*this) *= other; }

    /**
     * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    inline quat& operator*=(float scalar) {
        this->w *= scalar;
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        
        return (*this);
    }; // multiplication operators

    /**
     * @brief Computes the hamiltonian product of the left hand quaternion by the right hand quaternion
     * @param other quat to multiply by
     * @return hamiltonian product
     */
    const inline quat operator*(float scalar) const { return quat(*this) *= scalar; }

    // quat & operator*=(T s);
    // const quat operator*(T s) const { return quat(*this) *= s; } // scalar multiplication

    /**
     * @brief Calculates the length of the quaternion
     * @return length of the quaternion
     */
    inline T len() const { return (sqrtf(w * w + x * x + y * y + z * z)); } // norm of quat

    /**
     * @brief Calculates the length of the quaternion
     * @return length of the quaternion
     */
    inline T flen() const { return (fast_sqrt(w * w + x * x + y * y + z * z)); } // fast norm of quat

    /**
     * @brief Computes the conjugate of the quaternion
     * @return conjugate of the quaternion
     */
    quat conjugate() {
        return quat<T>(this->w, -this->x, -this->y, -this->z);
    }

    /**
     * @brief Normalizes the quaternion
     * @return normalized quaternion
     */
    quat normalize()
    {
        T n = len();
        return (quat(w / n, x / n, y / n, z / n));
    } // normalize quat

    /**
     * @brief Computes the fractional of the quaternon
     * @return fractional of the quaternion
     */
    quat fractional(T alpha) {

        this->w = 1 - alpha + alpha * w;
        this->x *= alpha;
        this->y *= alpha;
        this->z *= alpha;

        return this->normalize();
    };

    /**
     * @brief Computes the quaternion from a set of axis angles and magnitude
     * @param t magnitude
     * @param angles normalized axis angles
     * @return quaternion
     */
    quat from_axis_angle(T t, const vec3<T>& angles) {
        T sn = sinf(t / 2.0f);

        w = cosf(t / 2.0f);
        x = angles.x * sn;
        y = angles.y * sn;
        z = angles.z * sn;

        return(*this);
    }; // from axis angles? lol

    /**
     * @brief rotates another quaternion by this quaternion
     * @param q quaternion to rotate
     * @return rotated quaternion as a quaternion
     */
    quat rotate(const quat& q) const {
        quat<T> qNew = (*this * q) * quat<T>(this->w, -this->x, -this->y, -this->z);
        return qNew;
    };// input quat ouput quat

    /**
     * @brief rotates a vector by this quaternion
     * @param v vector to rotate
     * @return rotated vector as a quaternion
     */
    quat rotate(const vec3<T>& v) const {
        quat<T> qNew = this.rotate(quat<T>(1.0, v.x, v.y, v.z));
        return quat<T>(0.0f, qNew.x, qNew.y, qNew.z);
    } // input vec3 ouput quat

    /**
     * @brief rotates a vector by this quaternion
     * @param v vector to rotate
     * @return rotated vector
     */
    vec3<T> rotate_vec(const quat& q) const {
        quat<T> qNew = this.rotate(q);
        return vec3<T>(qNew.x, qNew.y, qNew.z);
    }; // input quat ouput vec3

    /**
     * @brief rotates a vector by this quaternion
     * @param v vector to rotate
     * @return rotated vector
     */
    vec3<T> rotate_vec(const vec3<T>& v) const {
        quat<T> qNew = this->rotate(quat<T>(0.0, v.x, v.y, v.z));
        return vec3<T>(qNew.x, qNew.y, qNew.z);
    };// input vec3 ouput vec3

    /**
     * @brief rotates a vector by this quaternion, explanation at https://www.johndcook.com/blog/2021/06/16/faster-quaternion-rotations/
     * @param v vector to rotate
     * @return rotated vector
     * also currently broken
     */
    const vec3<T> rotate_fast(const vec3<T>& v) {
        vec3<T> qVec = vec3<T>(this->x, this->y, this->z);
        vec3<T> t = qVec.cross(v) * 2.0f;
        vec3<T> ret = (t * this->w) + v + qVec.cross(t);
        return ret;
    };

    /**
     * @brief rotates a vector by this quaternion, explanation at https://www.johndcook.com/blog/2021/06/16/faster-quaternion-rotations/
     * @param v vector to rotate
     * @return rotated vector
     * also currently broken
     */
    const vec3<T> inv_rotate_fast(const vec3<T>& v) {

        vec3<T> qVec = vec3<T>(-this->x, -this->y, -this->z);
        vec3<T> t = qVec.cross(v) * 2.0f;
        vec3<T> ret = (t * this->w) + v + qVec.cross(t);
        
        return ret;

    };

    /**
     * @brief Calculates the rotation between this quaternion and a vector
     * @param v vector to compare rotation to
     * @return the rotation between the two vectors
     */
    quat rotation_between_vectors(const vec3<T>& v) {

        quat<T> q = *this * quat<T>(0, v.x, v.y, v.z);
        q.w = 1 - q.w;
        
        return(q.normalize());
    
    };   // rotation between two vectors

    /**
     * @brief Converts this quaternion to a set of euler angles
     * @return vec3 containing the euler angles
     */
    const vec3<T> euler_angles() const {

        T r = atan2f(2.0f * (this->w * this->x + this->y * this->z), 1.0f - 2.0f * (this->x * this->x + this->y * this->y));
        T p = 2.0f * (this->w * this->y - this->z * this->x);
        T y = atan2f(2.0f * (this->w * this->z + this->x * this->y), 1.0f - 2.0f * (this->y * this->y + this->z * this->z));

        // T r = atan2f(this->w * this->x + this->y * this->z, 0.5f - this->x * this->x - this->y * this->y);
        // T p = asinf(-2.0f * (this->x * this->z - this->w * this->y));
        // T y = atan2f(this->w * this->z + this->x * this->y, 0.5f - this->y * this->y - this->z * this->z);

        vec3<T> ret = vec3<T>(r, p, y);
        return ret;
    };

    float euler_angles_x() {
        return atan2f(2.0f * (this->w * this->x + this->y * this->z), 1.0f - 2.0f * (this->x * this->x + this->y * this->y));
    }

    float euler_angles_y() {
        return 2.0f * (this->w * this->y - this->z * this->x);
    }

    float euler_angles_z() {
        return atan2f(2.0f * (this->w * this->z + this->x * this->y), 1.0f - 2.0f * (this->y * this->y + this->z * this->z));
    }

    /**
     * @brief Converts a set of euler angles to a quaternion
     * @param roll angle around the roll axis
     * @param pitch angle around the pitch axis
     * @param yaw angle around the yaw axis
     * @return the calculatedquaternion
     */
    const quat<T> from_eulers(T roll, T pitch, T yaw) {

        T cr = cosf(roll / 2.0f);
        T cp = cosf(pitch / 2.0f);
        T cy = cosf(yaw / 2.0f);
        T sr = sinf(roll / 2.0f);
        T sp = sinf(pitch / 2.0f);
        T sy = sinf(yaw / 2.0f);

        this->w = cr * cp * cy + sr * sp * sy;
        this->x = sr * cp * cy - cr * sp * sy;
        this->y = cr * sp * cy + sr * cp * sy;
        this->z = cr * cp * sy - sr * sp * cy;

        return (*this);

    }

    /**
     * @brief Converts a set of euler angles to a quaternion
     * @param roll angle around the roll axis
     * @param pitch angle around the pitch axis
     * @param yaw angle around the yaw axis
     * @return the calculatedquaternion
     */
    const quat<T> from_eulers(vec3<T>& eulers) { return this->from_eulers(eulers.x, eulers.y, eulers.z); }

    /**
     * @brief Converts a set of euler angles to a quaternion
     * @param roll angle around the roll axis
     * @param pitch angle around the pitch axis
     * @param yaw angle around the yaw axis
     * @return the calculatedquaternion
     */
    const quat<T> from_eulers_fast(T roll, T pitch, T yaw) {

        float cos_cutoff = 0.5; // cutoff for cosine angle approximation (0.5 gives < 0.15 degrees of error)
        float sin_cutoff = 0.5; // cutoff for sine angle approximation (0.5 gives < 0.15 degrees of error)

        T cr, cp, cy, sr, sp, sy;

        abs(roll) < cos_cutoff ? cr = physics_redef_fast_cos(roll / 2.0f) : cr = cosf(roll / 2.0f);
        abs(pitch) < cos_cutoff ? cp = physics_redef_fast_cos(pitch / 2.0f) : cp = cosf(pitch / 2.0f);
        abs(yaw) < cos_cutoff ? cy = physics_redef_fast_cos(yaw / 2.0f) : cy = cosf(yaw / 2.0f);

        abs(roll) < sin_cutoff ? sr = roll / 2.0f : sr = sinf(roll / 2.0f);
        abs(pitch) < sin_cutoff ? sp = pitch / 2.0f : sp = sinf(pitch / 2.0f);
        abs(yaw) < sin_cutoff ? sy = yaw / 2.0f : sy = sinf(yaw / 2.0f);

        this->w = cr * cp * cy + sr * sp * sy;
        this->x = sr * cp * cy - cr * sp * sy;
        this->y = cr * sp * cy + sr * cp * sy;
        this->z = cr * cp * sy - sr * sp * cy;

        return (*this);

    }

    /**
     * @brief Converts a set of euler angles to a quaternion
     * @param roll angle around the roll axis
     * @param pitch angle around the pitch axis
     * @param yaw angle around the yaw axis
     * @return the calculatedquaternion
     */
    const quat<T> from_eulers_fast(vec3<T>& eulers) { return this->from_eulers_fast(eulers.x, eulers.y, eulers.z); }

};

using quaternion = quat<float>;
using vector3 = vec3<float>;

template <class T = float>
vec3<T> quat2vec(const quat<T>& q) { return vec3<T>(q.x, q.y, q.z); };

template <class T = float>
quat<T> vec2quat(const vec3<T>& v) { return quat<T>(1.0, v.x, v.y, v.z); };
