#ifndef KDNODE_INCLUDED
#define KDNODE_INCLUDED

#include "Mesh.h"

//KDNode tree

class KDNode
{
    public:

    KDNode() {};
    ~KDNode () {};
    
	//compute KDtree for the list of triangles from Mesh
    static KDNode* buildKDTree_test (const Mesh& mesh, const std::vector<int>& list, float percentage);
    
   //variables :   
   KDNode *leftChild;
   KDNode *rightChild;
   // Data is the structure that describes plane
   struct Data {
   Vec3f mediane; // mediane
   Vec3f min;// value of the minimal point in the bounding box
   Vec3f max;// gitvalue of the maximal point in the bounding box
   unsigned int max_axe; // max axe of the current bounding boxe B

   };
   
   Data data;
};

#endif // KDNODE_INCLUDED
