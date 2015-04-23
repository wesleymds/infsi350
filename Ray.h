#ifndef RAY_H
#define RAY_H

#include <iostream>
#include "Vec3.h"
#include "Mesh.h"
#include "tiny_obj_loader.h"
#include "KDNode.h"
#include <stack>

#define ivtri(i, tri) (mesh.V[tri.v[i]].p)

using namespace std;

class Ray {
private:
    static const float epsilon;
    const Vec3f origin;
    Vec3f direction;
    Mesh& mesh;

    Vec3f idirection;
    int sign[3];
public:
    float tnear, tfar;

    void initDirection() {
        for(auto i = 0; i < 3; ++i) idirection[i] = 1.f/direction[i];
        sign[0] = (idirection[0] < 0);
        sign[1] = (idirection[1] < 0);
        sign[2] = (idirection[2] < 0);
    }

    Ray(const Vec3f& origin, const Vec3f& direction, Mesh& mesh)
        : origin(origin), direction(direction), mesh(mesh)
    {
        initDirection();
    }

    Ray(Mesh& mesh, const Vec3f& origin)
        : origin(origin), mesh(mesh)
    {}

    void setDirection(const Vec3f& _direction) {
        direction = _direction;
        initDirection();
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

    int rayTriangleIntersection(unsigned int i, Vertex& out) {
        Triangle tri = mesh.T[i];
        Vec3f p0 = ivtri(0, tri);
        Vec3f p1 = ivtri(1, tri);
        Vec3f p2 = ivtri(2, tri);
        return rayTriangleIntersection(p0, p1, p2, out);
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

    int rayBoxIntersection(const Box& box)
    {
        float tnear, tfar, tymin, tymax, tzmin, tzmax;
        Vec3f coins[2] = box.coins;

        tnear = (coins[sign[0]][0] - origin[0]) * idirection[0];
        tfar = (coins[1-sign[0]][0] - origin[0]) * idirection[0];
        this->tnear = tnear;
        this->tfar = tfar;
        tymin = (coins[sign[1]][1] - origin[1]) * idirection[1];
        tymax = (coins[1-sign[1]][1] - origin[1]) * idirection[1];

        // Box is missed
        if ((tnear > tymax) || (tymin > tfar)) return 0;

        // Update tnear tfar
        if (tymin > tnear) tnear = tymin;
        if (tymax < tfar) tfar = tymax;

        tzmin = (coins[sign[2]][2] - origin[2]) * idirection[2];
        tzmax = (coins[1-sign[2]][2] - origin[2]) * idirection[2];

        // Box is missed
        if ((tnear > tzmax) || (tzmin > tfar)) return 0;

        // Update tnear tfar
        if (tzmin > tnear) tnear = tzmin;
        if (tzmax < tfar) tfar = tzmax;

        if (tnear > this->tnear) this->tnear = tnear;
        if (tfar < this->tfar) this->tfar = tfar;

        return 1;
    }

    int rayKDIntersection(KDNode* root, const Vec3f& eye, Vertex& out) {
        /*struct intersectData
        {
            intersectData(unsigned int a_noffs, float tfar)
            {
                far_node_offset = a_noffs;
                tfar = _tfar;
            }
            unsigned int far_node_offset;
            float tfar;
        };*/

        std::stack<float> nstack;

        float tnear, tfar;

        // intersect ray with box
        bool isIntersected = rayBoxIntersection(root->data);

        tnear = this->tnear;
        tfar = this->tfar;

        // box is behind the origin
        if(tfar < 0.f) return 0;

        // we are in box and first intersection is origin
        if(tnear <= 0.f) tnear = 0.f;

        //cout << isIntersected << endl;
        if(tnear >= tfar || !isIntersected) return 0;
        //unsigned int nodeOffset=0;

        float tsplit = 0;

        while(true)
        {
            while(!root->isLeaf())
            {
                // get axis number and t according to split_pos plane
                int axis = root->data.max_axe;
                tsplit = (root->data.mediane[axis] - origin[axis])*idirection[axis];

                /*// get far and near nodes.
                // assume { left = near, far = right }; if(ray.dir[axis] < 0) swap left and right nodes.
                // this trick with left_or_right[axis] allows us to remove one 'if'
                unsigned int nearNodeOffset = node.GetLeftOffset() + (1-sign[axis]);
                unsigned int farNodeOffset  = node.GetLeftOffset() + sign[axis];*/

                // now kd-tree traversal algorithm specific
                if(tsplit <= tnear)
                {
                    root = root->rightChild;
                    //root = kd_tree->GetNodeByOffset(farNodeOffset);
                }
                else if(tsplit >= tfar)
                {
                    root = root->leftChild;
                    //node = kd_tree->GetNodeByOffset(nearNodeOffset);
                }
                else
                {
                    nstack.push(tfar);
                    root = root->leftChild;
                    tfar = tsplit;
                    /*nstack.push(Traversal_Data(farNodeOffset,t_far));
                    node = kd_tree->GetNodeByOffset(nearNodeOffset);
                    t_far = t_split;*/
                }

            }

            Vertex intersect;
            isIntersected = false;
            float d, e(100000.f);
            for(auto tri: root->primitives) {
                if (rayTriangleIntersection(tri, intersect)) {
                    d = (eye - intersect.p).length();
                    if(d < e) {
                        e = d;
                        out = intersect;
                        out.material_id = mesh.T[tri].material_id;
                        isIntersected = true;
                    }
                }
            }

            if (isIntersected && out.p[root->data.max_axe] <= tfar) return 1;

            if (nstack.empty())
                return 0;	// noting else to traverse any more...

            tnear = tfar;

            //( t_near, t_far ) = stack.pop();
            /*nodeOffset = nstack.top().far_node_offset;
            t_far      = nstack.top().t_far;
            nstack.pop();*/
            tfar = nstack.top();
            nstack.pop();
        }

        return 0;
    }
};

#endif // RAY_H
