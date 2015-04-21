// --------------------------------------------------------------------------
// Copyright(C) 2009-2015
// Tamy Boubekeur
//
// All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
// for more details.
// --------------------------------------------------------------------------

#ifndef MESH_H
#define MESH_H

#include "tiny_obj_loader.h"
#include "Vec3.h"
#include <vector>
#include <iostream>
#include <algorithm>

#define ptv(i) shapes[s].mesh.positions[i]
#define ntv(i) shapes[s].mesh.normals[i]

class Vertex {
public:
    inline Vertex () {}
    inline Vertex (const Vec3f & p, const Vec3f & n) : p (p), n (n) {}
    inline virtual ~Vertex () {}
    inline bool operator == (const Vertex& that) {
        return (p[0] == that.p[0] && p[1] == that.p[1] && p[2] == that.p[2]);
    }
    Vec3f p;
    Vec3f n;
};

class Triangle {
public:
    inline Triangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        material_id = t.material_id;
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }
    inline virtual ~Triangle () {}
    inline Triangle & operator= (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        material_id = t.material_id;
        return (*this);
    }
    inline void set_triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    unsigned int v[3];
    int material_id;
};

class Mesh {
private:
    std::vector<tinyobj::shape_t> shapes;

    void regenerate_from_obj() {
        unsigned int vertex_ind(0), i;
        Triangle tri;
        for (size_t s = 0; s < shapes.size(); s++) {
            T.reserve(shapes[s].mesh.indices.size() / 3);
            V.reserve(shapes[s].mesh.indices.size() / 3);
            for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
                for (size_t v = 0; v < 3; v++) {
                    i = 3*shapes[s].mesh.indices[3*f+v];
                    V.push_back(Vertex(Vec3f(ptv(i), ptv(i + 1), ptv(i + 2)), Vec3f(ntv(i), ntv(i + 1), ntv(i + 2))));
                    tri.v[v] = vertex_ind++;
                }
                if (!materials.empty ())
                    tri.material_id = shapes[s].mesh.material_ids[f];
                T.push_back(tri);
            }
        }

        /*centerAndScaleToUnit();

        recomputeNormals();*/
    }

    void recomputeNormals () {
        for (unsigned int i = 0; i < V.size (); i++)
            V[i].n = Vec3f (0.0, 0.0, 0.0);
        for (unsigned int i = 0; i < T.size (); i++) {
            Vec3f e01 = V[T[i].v[1]].p -  V[T[i].v[0]].p;
            Vec3f e02 = V[T[i].v[2]].p -  V[T[i].v[0]].p;
            Vec3f n = cross (e01, e02);
            n.normalize ();
            for (unsigned int j = 0; j < 3; j++)
                V[T[i].v[j]].n += n;
        }
        for (unsigned int i = 0; i < V.size (); i++)
            V[i].n.normalize ();
    }

    void centerAndScaleToUnit () {
        Vec3f c;
        for  (unsigned int i = 0; i < V.size (); i++)
            c += V[i].p;
        c /= V.size ();
        float maxD = dist (V[0].p, c);
        for (unsigned int i = 0; i < V.size (); i++){
            float m = dist (V[i].p, c);
            if (m > maxD)
                maxD = m;
        }
        for  (unsigned int i = 0; i < V.size (); i++)
            V[i].p = (V[i].p - c) / maxD;
    }

public:
    std::vector<Vertex> V;
    std::vector<Triangle> T;
    std::vector<tinyobj::material_t> materials;

    Mesh() {}

    Mesh(const std::vector<tinyobj::shape_t>& shapes,const std::vector<tinyobj::material_t>& materials)
        : shapes(shapes), materials(materials)
    {
        regenerate_from_obj();
    }

    inline void set_mesh(std::vector<tinyobj::shape_t>& _shapes, const std::vector<tinyobj::material_t>& _materials) {
        /*V.clear();
        T.clear();*/
        shapes = _shapes;
        materials = _materials;
        regenerate_from_obj();
    }

    void show_properties() {
        unsigned int shapes_size(0);
        for (size_t s = 0; s < shapes.size(); s++) shapes_size += shapes[s].mesh.indices.size();
        std::cout << "Number of triangles in shape model = " << shapes_size / 3 << std::endl;
        std::cout << "Number of vertices in mesh model = " << V.size() << std::endl;
        std::cout << "Number of triangles in mesh model = " << T.size() << std::endl;
    }

    tinyobj::material_t& material(unsigned int tri) {
        if (!materials.empty ()) {
            return materials[T[tri].material_id];
        }
    }
};

#endif // MESH_H

