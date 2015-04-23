#ifndef KDNODE_INCLUDED
#define KDNODE_INCLUDED

#include "Mesh.h"

//KDNode tree

// Data is the structure that describes plane
struct Box {
    Vec3f mediane; // mediane
    Vec3f coins[2];// value of the minimal point and the maximal point in the bounding box
    unsigned int max_axe; // max axe of the current bounding boxe B
};

class KDNode {
public:
    //variables :
    Box data;
    std::vector<int> primitives;
    KDNode *leftChild;
    KDNode *rightChild;

    KDNode() {}
    ~KDNode () {}
    
    //compute KDtree for the list of triangles from Mesh
    KDNode* buildKDTree (const Mesh& mesh, const std::vector<int>& list, float percentage);

    bool isLeaf();
};

#endif // KDNODE_INCLUDED
