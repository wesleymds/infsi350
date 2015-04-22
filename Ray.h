#ifndef RAY_H
#define RAY_H

#include <iostream>
#include "Vec3.h"
#include "Mesh.h"
#include "tiny_obj_loader.h"
#include "KDNode.h"

#define ivtri(i, tri) (mesh.V[tri.v[i]].p)

using namespace std;

struct intersectData
{
    intersectData(unsigned int a_noffs,float a_t_far)
    {
        far_node_offset = a_noffs;
        t_far = a_t_far;
    }
    unsigned int far_node_offset;
    float t_far;
};


class Ray {
private:
    static const float epsilon;
    const Vec3f origin;
    Vec3f direction;

    Mesh& mesh;
public:
    Ray(const Vec3f& origin, const Vec3f& direction, Mesh& mesh)
        : origin(origin), direction(direction), mesh(mesh)
    {}

    Ray(Mesh& mesh, const Vec3f& origin)
        : origin(origin), mesh(mesh)
    {}

    void setDirection(const Vec3f& _direction) {
        direction = _direction;
    }

    int rayTriangleIntersection(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Vertex& out) {
        Vec3f e0, e1, n, q, s, r;
        float a, b0, b1, b2, t;

        e0 = p1 - p0;
        e1 = p2 - p0;

        n = cross(e0, e1);
        n.normalize();
        q = cross(direction, e1);

        a = dot(e0, q);
        if ( dot(n, direction) >= 0 || abs(a) < epsilon) return 0;

        s = (origin - p0)/a;
        r = cross(s, e0);
        b0 = dot(s, q);
        b1 = dot(r, direction);
        b2 = 1 - b0 - b1;
        if (b0 < 0 || b1 < 0 || b2 < 0) return 0;

        t = dot(e1, r);
        if (t >= 0) {
            out.p = origin + t * direction;
            out.n = n;
            return 1;
        }
        return 0;
    }

    int raySceneIntersection(const Vec3f& eye, Vertex& out) {
        Vec3f p0;
        Vec3f p1;
        Vec3f p2;
        int e(1000000), d;
        Vertex intersect;
        bool isIntersect(false);
        for(auto tri: mesh.T) {
            p0 = ivtri(0, tri);
            p1 = ivtri(1, tri);
            p2 = ivtri(2, tri);
            if(rayTriangleIntersection(p0, p1, p2, intersect) == 1) {
                isIntersect = true;
                d = (eye - intersect.p).length();
                if(d < e) {
                    e = d;
                    out = intersect;
                    out.material_id = tri.material_id;
					out.shapeName = tri.shapeName; // test
					//cout << tri.shapeName << " " << tri.v[0] << endl;
                }
            }
        }
        return isIntersect ? 1 : 0;
    }

    /*int rayBoxIntersection(const Box& box)
    {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        Vec3f& coins = box.coins;
        tmin = (coins[r.sign[0]].x - r.orig.x) * r.invdir.x;
        tmax = (coins[1-r.sign[0]].x - r.orig.x) * r.invdir.x;
        tymin = (coins[r.sign[1]].y - r.orig.y) * r.invdir.y;
        tymax = (coins[1-r.sign[1]].y - r.orig.y) * r.invdir.y;
        if ((tmin > tymax) || (tymin > tmax))
            return 0;
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        tzmin = (bounds[r.sign[2]].z - r.orig.z) * r.invdir.z;
        tzmax = (bounds[1-r.sign[2]].z - r.orig.z) * r.invdir.z;
        if ((tmin > tzmax) || (tzmin > tmax))
            return 0;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        if (tmin > r.tmin) r.tmin = tmin;
        if (tmax < r.tmax) r.tmax = tmax;
        return 1;
    }

    int rayKDIntersection(KDNode& kdn) {
        std::stack<intersectData> nstack;
        float t_near,t_far;

        //TODO
        // intersect ray with box
        bool isIntersected = false;

        if(t_near <= 0.0f) t_near = 0.0f;

        if(t_near >= t_far || !isIntersected) return 0;

        /*


        const int sign[3] = { (ray.dir[0] >= 0)? 1 : 0,
                                       (ray.dir[1] >= 0)? 1 : 0,
                                       (ray.dir[2] >= 0)? 1 : 0 };

        unsigned int nodeOffset=0;
        KdTreeNode node = *(kd_tree->GetRoot());

        float t_split = 0;

        while(true)
        {
            //err_stream << nodeOffset << std::endl;
            while(!node.Leaf())
            {
                // get axis number and t according to split_pos plane
                int axis = node.GetAxis();
                t_split = (node.GetSplitPos() - ray.pos[axis])*rdir_inv[axis];

                // get far and near nodes.
                // assume { left = near, far = right }; if(ray.dir[axis] < 0) swap left and right nodes.
                // this trick with left_or_right[axis] allows us to remove one 'if'
                unsigned int nearNodeOffset = node.GetLeftOffset() + (1-left_or_right[axis]);
                unsigned int farNodeOffset  = node.GetLeftOffset() + left_or_right[axis];

                // now kd-tree traversal algorithm specific
                if(t_split <= t_near)
                {
                    node = kd_tree->GetNodeByOffset(farNodeOffset);
                }
                else if(t_split >= t_far)
                {
                    node = kd_tree->GetNodeByOffset(nearNodeOffset);
                }
                else
                {
                    nstack.push(Traversal_Data(farNodeOffset,t_far));
                    node = kd_tree->GetNodeByOffset(nearNodeOffset);
                    t_far = t_split;
                }

            }

            float t_hit = IntersectAllPrimitivesInLeaf(ray,node,pHit);
            if (t_hit <= t_far)
                return 0xFFFFFFFF; // early ray termination

            if (nstack.empty())
                return 0;	// noting else to traverse any more...

            t_near = t_far;

            //( t_near, t_far ) = stack.pop();
            nodeOffset = nstack.top().far_node_offset;
            t_far      = nstack.top().t_far;
            nstack.pop();

            node = kd_tree->GetNodeByOffset(nodeOffset);
        }

        return 0;
    }*/
};

#endif // RAY_H
