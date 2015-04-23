#include "KDNode.h"

using namespace std;

int i_max_axis;

bool coordinate_sort (Vec3f v1, Vec3f v2) {
    return (v1[i_max_axis]<v2[i_max_axis]);
}

bool KDNode::isLeaf() {
    return leftChild == nullptr || rightChild == nullptr;
}

//list of triangles is input
KDNode* KDNode::buildKDTree (const Mesh& mesh, const vector<int>& list, float percentage) {
    //cout<<"Input percentage "<< percentage << endl;
    //cout<<"Size of all elements "<< list.size() << endl;

    if (percentage > 0.55f || list.size() <= 5) return nullptr;

    KDNode* n = new KDNode;

    // find the extension of each axe of the bounding boxe
    Vec3f max(mesh.V[mesh.T[list[1]].v[1]].p);
    Vec3f min(mesh.V[mesh.T[list[1]].v[1]].p);

    // k is the number of vectex in triangle
    for(unsigned int k=0;k<3;k++) {
        // i is the number of coordinate of vertex
        for (unsigned int i=0;i<3;i++) {
            // j is the number of triangle
            for (unsigned int j=0;j<list.size();j++) {
                if (max[i] < mesh.V[mesh.T[list[j]].v[k]].p[i]) max[i] = mesh.V[mesh.T[list[j]].v[k]].p[i];
                if (min[i] > mesh.V[mesh.T[list[j]].v[k]].p[i]) min[i] = mesh.V[mesh.T[list[j]].v[k]].p[i];
            }
        }
    }

    // axis-aligned bounding box is the cartesian product of max and min
    n->data.coins[0] = min*1.1f;
    n->data.coins[1] = max*1.1f;

    // find the max extension
    Vec3f max_ax;
    for (unsigned int i=0;i<3;i++) max_ax[i] = max[i] - min[i];

    // find the axe that corresponds to the maximum extension
    i_max_axis = max_ax[0] < max_ax[1] ? 1 : 0;
    i_max_axis = max_ax[2] < max_ax[i_max_axis] ? i_max_axis : 2;
    n->data.max_axe = i_max_axis;

    // find median sample:
    vector<Vec3f> sort_mesh(3*list.size());
    for (unsigned int j = 0; j<list.size();j++)
        //cout<<"Triangle "<< j <<" "<<endl;
        for (unsigned int k=0; k<3; k++)
            sort_mesh[3*j+k] = mesh.V[mesh.T[list[j]].v[k]].p;

    // sort
    sort(sort_mesh.begin(), sort_mesh.end(), coordinate_sort);

    // find mediane (according to max axe) in the current list
    n->data.mediane = sort_mesh[sort_mesh.size()/2];

    n->primitives = list;

    //split vectices in two lists: left and rigth according to the median and max axe

    //cout<<"Mediane coordinate "<< i_max_axis << " Value of mediane axe " << data.mediane[i_max_axis] << endl;

    vector<int> listPu;
    vector<int> listPl;
    vector<int> s(list.size(), 0);

    for (unsigned int j=0; j<list.size(); j++)
        for (unsigned int k=0; k<3; k++)
            s[j] += (mesh.V[mesh.T[list[j]].v[k]].p[i_max_axis] < n->data.mediane[i_max_axis]);

    for (unsigned int j=0;j<list.size();j++){
        if (s[j]==3) listPu.push_back(list[j]);
        if (s[j]<3 && 0<s[j]) {
            listPu.push_back(list[j]);
            listPl.push_back(list[j]);
        }
        if (s[j]==0) listPl.push_back(list[j]);
    }

    // compute intersection of two point lists in percentage

    //sort element in the array
    sort (listPu.begin(),listPu.end());
    sort (listPl.begin(),listPl.end());

    // find the maximal size of the list
    unsigned int max_size = listPu.size()>listPl.size() ? listPu.size() : listPl.size();

    vector<int> v(max_size);
    vector<int>::iterator it;
    it=set_intersection (listPu.begin(),listPu.end(), listPl.begin(),listPl.end(), v.begin());
    v.resize(it-v.begin());
    percentage = v.size()/(float)max_size;

    //cout<<"Output percentage "<<percentage<<endl;
    //cout<<endl;

    // apply recursion if the list is not empty
    /*KDNode* left(nullptr);
    KDNode* right(nullptr);*/
    n->leftChild = (listPu.size() != list.size()) ? buildKDTree(mesh,listPu,percentage) : nullptr;
    n->rightChild = (listPl.size() != list.size()) ? buildKDTree(mesh,listPl,percentage) : nullptr;
    /*if (listPu.size() != list.size()) {
        left = new KDNode;
        left->buildKDTree(mesh, listPu, percentage);
    }
    leftChild = left;
    if (listPl.size() != list.size()) {
        right = new KDNode;
        right->buildKDTree(mesh, listPl, percentage);
    }
    rightChild = right;*/

    //cout << "p.size()" << this->primitives.size() << " " << this << " " << rightChild << " " << endl;
    return n;
}

//list of triangles is input
KDNode* fastBuildKDNode (const tinyobj::shape_t& shape, vector<int> list, float percentage) {
    if (percentage > 0.55f || list.empty()) return nullptr;

    unsigned int index;
    KDNode* n = new KDNode;

    float max[3], min[3];
    copy(shape.mesh.positions.begin(), shape.mesh.positions.begin() + 3, min);
    copy(shape.mesh.positions.begin(), shape.mesh.positions.begin() + 3, max);

    for (size_t l=0; l<list.size(); l++) {
        for (size_t v = 0; v < 3; v++) {
            cout << list[l] << endl;
            index = 3*shape.mesh.indices[3*list[l]+v];
            for(size_t j = 0; j < 3; j++) {
                if (max[index+j] < shape.mesh.positions[index+j]) max[index+j] = shape.mesh.positions[index+j];
                if (min[index+j] > shape.mesh.positions[index+j]) min[index+j] = shape.mesh.positions[index+j];
            }
        }
    }

    // axis-aligned bounding box is the cartesian product of max and min
    n->data.coins[0] = Vec3f(min[0], min[1], min[2])*1.1f;
    n->data.coins[1] = Vec3f(max[0], max[1], max[2])*1.1f;

    // find the max extension
    float max_ax[3];
    for (unsigned int i=0; i<3; ++i) max_ax[i] = max[i] - min[i];

    // find the axe that corresponds to the maximum extension
    i_max_axis = max_ax[0] < max_ax[1] ? 1 : 0;
    i_max_axis = max_ax[2] < max_ax[i_max_axis] ? i_max_axis : 2;
    n->data.max_axe = i_max_axis;

    // find median sample:
    vector<Vec3f> sort_mesh(3*list.size());
    for (auto f: list)
        for (size_t v = 0; v < 3; v++) {
            index = 3*shape.mesh.indices[3*f+v];
            sort_mesh[3*f+v] = Vec3f(shape.mesh.positions[index],
                                     shape.mesh.positions[index + 1],
                    shape.mesh.positions[index + 2]);
        }

    // sort
    sort(sort_mesh.begin(), sort_mesh.end(), coordinate_sort);

    // find mediane (according to max axe) in the current list
    n->data.mediane = sort_mesh[sort_mesh.size()/2];

    n->primitives = list;

    //split vectices in two lists: left and rigth according to the median and max axe

    vector<int> listPu;
    vector<int> listPl;
    vector<int> check(list.size(), 0);

    for (unsigned int j=0; j<list.size(); j++) {
        index = 3*shape.mesh.indices[3*list[j]+i_max_axis];
        check[j] += (shape.mesh.positions[index] < n->data.mediane[i_max_axis]);
    }

    for (unsigned int j=0;j<list.size();j++){
        if (check[j]==3) listPu.push_back(list[j]);
        if (check[j]<3 && 0<check[j]) {
            listPu.push_back(list[j]);
            listPl.push_back(list[j]);
        }
        if (check[j]==0) listPl.push_back(list[j]);
    }

    // compute intersection of two point lists in percentage

    //sort element in the array
    sort (listPu.begin(),listPu.end());
    sort (listPl.begin(),listPl.end());

    // find the maximal size of the list
    unsigned int max_size = listPu.size()>listPl.size() ? listPu.size() : listPl.size();

    vector<int> v(max_size);
    vector<int>::iterator it;
    it=set_intersection (listPu.begin(),listPu.end(), listPl.begin(),listPl.end(), v.begin());
    v.resize(it-v.begin());
    percentage = v.size()/(float)max_size;

    // apply recursion if the list is not empty
    n->leftChild = (listPu.size() != list.size()) ? fastBuildKDNode(shape,listPu,percentage) : nullptr;
    n->rightChild = (listPl.size() != list.size()) ? fastBuildKDNode(shape,listPl,percentage) : nullptr;

    return n;
}

void KDNode::fastBuildKDTree (const std::vector<tinyobj::shape_t>& shapes, vector<KDNode*>& tree) {
    KDNode* node;
    for(size_t s=0; s < shapes.size(); ++s) {
        vector<int> list;
        unsigned int number_tri = shapes[s].mesh.indices.size() / 3;
        list.reserve(number_tri);
        for (unsigned int i=0; i < number_tri; i++) list.push_back(i);
        node = fastBuildKDNode(shapes[s], list, 0.f);
        tree.push_back(node);
    }
}


