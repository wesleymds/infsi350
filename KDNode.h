#ifndef KDNODE_INCLUDED
#define KDNODE_INCLUDED

#include "Mesh.h"

//KDNode tree

// Data is the structure that describes plane
struct Data {
    Vec3f mediane; // mediane
    Vec3f min;// value of the minimal point in the bounding box
    Vec3f max;// value of the maximal point in the bounding box
    unsigned int max_axe; // max axe of the current bounding boxe B
};

class KDNode {
public:
    //variables :
    Data data;
    KDNode *leftChild;
    KDNode *rightChild;

    KDNode() {}
    ~KDNode () {}
    
    //compute KDtree for the list of triangles from Mesh
    static KDNode* buildKDTree (const Mesh& mesh, const std::vector<int>& list, float percentage);

};

#endif // KDNODE_INCLUDED
