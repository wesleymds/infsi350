#ifndef KDNODE_INCLUDED
#define KDNODE_INCLUDED

#include "Mesh.h"

//KDNode tree

class KDNode
{
    public:

    KDNode() {};
    ~KDNode () {};
    
	//compute KDtree for the set of vertices from Mesh
    static KDNode* buildKDTree_test (const Mesh& mesh, const std::vector<int>& list);
   
   //variables :   
   KDNode *leftChild;
   KDNode *rightChild;
   // Data is the structure that describes plane
   struct Data {
   Vec3f mediane; // mediane
   unsigned int max_axe; // max axe of the current bounding boxe B
   };
   
   Data data;
};

#endif // KDNODE_INCLUDED
