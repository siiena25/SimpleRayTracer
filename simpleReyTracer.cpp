#include <limits>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include "geometry.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION

#include "stb/stb_image.h"

struct Material {
    float refractive_index;
    vector4 albedo;
    vector3 diffuse_color;
    float specular_exponent;

    Material(const float &r, const vector4 &a, const vector3 &color, const float &spec) : refractive_index(r),
                                                                                          albedo(a),
                                                                                          diffuse_color(color),
                                                                                          specular_exponent(spec) {}

    Material() : refractive_index(1), albedo{1, 0, 0, 0}, diffuse_color(), specular_exponent() {}
};

vector3 reflect(const vector3 &I, const vector3 &N) {
    return I - N * 2.f * (I * N);
}

vector3 refract(const vector3 &I, const vector3 &N, const float &refractive_index) {
    float cosQ1 = -std::max(-1.f, std::min(1.f, I * N));
    float n1 = 1;
    float n2 = refractive_index;
    vector3 n = N;
    if (cosQ1 < 0) {
        cosQ1 = -cosQ1;
        std::swap(n1, n2);
        n = -N;
    }
    float n_ratio = n1 / n2;
    float sinQ1 = sqrtf(1 - cosQ1 * cosQ1);
    float sinQ2 = n_ratio * sinQ1;
    float cosQ2_sqr = 1 - sinQ2 * sinQ2;
    return cosQ2_sqr < 0 ? vector3{0, 0, 0} : I * n_ratio + n * (n_ratio * cosQ1 - sqrtf(cosQ2_sqr));
}

struct Light {
    Light(const vector3 &p, const float &i) : position(p), intensity(i) {}

    vector3 position;
    float intensity;
};

struct Sphere {
    vector3 center;
    float radius;
    Material material;

    Sphere(const vector3 &c, const float &r, const Material &m) : center(c), radius(r), material(m) {
    }

    bool rayIntersectionTest(const vector3 &orig, const vector3 &dir, float &t0) const {
        vector3 L = center - orig;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius) return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

bool sceneIntersectionTest(
        const vector3 &orig,
        const vector3 &dir,
        const std::vector<Sphere> &spheres, vector3 &hit,
        vector3 &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (const auto &sphere: spheres) {
        float dist_i;
        if (sphere.rayIntersectionTest(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir * dist_i;
            N = (hit - sphere.center).normalize();
            material = sphere.material;
        }
    }
    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y) > 1e-3) {
        float d = -(orig.y + 4) / dir.y; // the checkerboard plane has equation y = -4
        vector3 pt = orig + dir * d;
        if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = vector3{0, 1, 0};
            material.diffuse_color =
                    (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? vector3{1, 1, 1} : vector3{1, .7, .3};
            material.diffuse_color = material.diffuse_color * .3;
        }
    }
    return std::min(spheres_dist, checkerboard_dist) < 1000;
}

vector3 cast_ray(
        const vector3 &orig,
        const vector3 &dir,
        const std::vector<Sphere> &spheres,
        const std::vector<Light> &lights,
        size_t depth = 0) {
    vector3 point, N;
    Material material;

    if (depth > 4 || !sceneIntersectionTest(orig, dir, spheres, point, N, material)) {
        return vector3{0.6, 0.6, 0.8};
    }

    vector3 reflect_dir = reflect(dir, N).normalize();
    vector3 reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    vector3 reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);

    vector3 refract_dir = refract(dir, N, material.refractive_index).normalize();
    vector3 refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    vector3 refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0;
    float specular_light_intensity = 0;
    for (auto light: lights) {
        vector3 light_direction = (light.position - point).normalize();
        float light_distance = (light.position - point).norm();
        vector3 shadow_orig = light_direction * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
        vector3 shadow_pt, shadow_N;
        Material tmp_material;
        if (sceneIntersectionTest(shadow_orig, light_direction, spheres, shadow_pt, shadow_N, tmp_material)
            && (shadow_pt - shadow_orig).norm() < light_distance) {
            continue;
        }

        diffuse_light_intensity += light.intensity * std::max(0.f, light_direction * N);
        specular_light_intensity +=
                powf(std::max(0.f, reflect(light_direction, N) * dir), material.specular_exponent) * light.intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] +
           vector3{1., 1., 1.} * specular_light_intensity * material.albedo[1] +
           reflect_color * material.albedo[2] +
           refract_color * material.albedo[3];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights) {
    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 2.;
    std::vector<vector3> framebuffer(width * height);

    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            framebuffer[i + j * width] = vector3{float(j) / float(height), float(i) / float(width), 0};
            float x = float(2 * (float(i) + 0.5) / (float) width - 1) * tan(fov / 2.) * float(width) / (float) height;
            float y = float(-(2 * (float(j) + 0.5) / (float) height - 1)) * tan(fov / 2.);
            vector3 dir = vector3{x, y, -1}.normalize();
            framebuffer[i + j * width] = cast_ray(vector3{0, 0, 0}, dir, spheres, lights);
        }
    }

    std::ofstream file;
    file.open("./out.ppm");
    file << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i) {
        vector3 &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max > 1) {
            c = c * (float(1. / max));
        }
        for (size_t j = 0; j < 3; j++) {
            file << (char) (255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    file.close();
}

int main() {
    Material violet(1.0, vector4{0.6, 0.3, 0.1, 0.0}, vector3{0.3, 0.2, 0.4}, 50.);
    Material blue(1.0, vector4{0.9, 0.1, 0.0, 0.0}, vector3{0.3, 0.1, 0.7}, 10.);
    Material mirror(1.0, vector4{0.0, 10.0, 0.8, 0.0}, vector3{1.0, 1.0, 1.0}, 1425.);
    Material glass(1.5, vector4{0.0, 0.5, 0.1, 0.8}, vector3{0.6, 0.7, 0.8}, 125.);

    std::vector<Sphere> spheres;
    spheres.emplace_back(vector3{-3, 0, -16}, 2, violet);
    spheres.emplace_back(vector3{-1.0, -1.5, -12}, 2, glass);
    spheres.emplace_back(vector3{1.5, -0.5, -18}, 3, blue);
    spheres.emplace_back(vector3{7, 5, -18}, 4, mirror);

    std::vector<Light> lights;
    lights.emplace_back(vector3{-20, 20, 20}, 1.5);

    render(spheres, lights);
    return 0;
}