#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>
#include <cassert>
#include <iostream>

template<size_t N>
struct vector {
    float data[N] = {};

    float &operator[](const size_t i) {
        assert(i < N);
        return data[i];
    }

    const float &operator[](const size_t i) const {
        assert(i < N);
        return data[i];
    }
};

template<size_t N>
vector<N> operator*(const vector<N> &l, const float r) {
    vector<N> ans;
    for (size_t i = N; i--; ans[i] = l[i] * r);
    return ans;
}

template<size_t N>
float operator*(const vector<N> &l, const vector<N> &r) {
    float ans = 0;
    for (size_t i = N; i--; ans += l[i] * r[i]);
    return ans;
}

template<size_t N>
vector<N> operator+(vector<N> l, const vector<N> &r) {
    for (size_t i = N; i--; l[i] += r[i]);
    return l;
}

template<size_t N>
vector<N> operator-(vector<N> l, const vector<N> &r) {
    for (size_t i = N; i--; l[i] -= r[i]);
    return l;
}

template<size_t N>
vector<N> operator-(const vector<N> &l) {
    return l * (-1.f);
}

template<>
struct vector<3> {
    float x = 0;
    float y = 0;
    float z = 0;

    float &operator[](const size_t i) {
        assert(i < 3);
        return i == 0 ? x : (1 == i ? y : z);
    }

    const float &operator[](const size_t i) const {
        assert(i < 3);
        return i == 0 ? x : (1 == i ? y : z);
    }

    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    vector<3> &normalize(float l = 1) {
        *this = (*this) * (l / norm());
        return *this;
    }
};

typedef vector<3> vector3;
typedef vector<4> vector4;

vector3 cross(vector3 v1, vector3 v2) {
    return {v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x};
}

template<size_t N>
std::ostream &operator<<(std::ostream &out, const vector<N> &v) {
    for (size_t i = 0; i < N; i++) {
        out << v[i] << " ";
    }
    return out;
}

#endif //__GEOMETRY_H__
