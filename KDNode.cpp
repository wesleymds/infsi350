#include "KDNode.h"
#include <vector>
#include <algorithm>
using namespace std;

int i_max_axis;

bool coordinate_sort (Vec3f v1, Vec3f v2) {
    return (v1[i_max_axis]<v2[i_max_axis]);
}

//list of triangles is input
KDNode* KDNode::buildKDTree (const Mesh& mesh, const vector<int>& list, float percentage) {
    cout<<"Input percentage "<< percentage << endl;
    cout<<"Size of all elements "<< list.size() << endl;

    if (percentage > 0.55f || list.size() <= 5) return nullptr;

    KDNode* n = new KDNode();

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
    n->data.coins[0] = min;
    n->data.coins[1] = max;

    // find the max extension
    Vec3f max_ax;
    for (unsigned int i=0;i<3;i++) max_ax[i] = max[i] - min[i];

    // find the axe that corresponds to the maximum extension
    i_max_axis = max_ax[0] < max_ax[1] ? 1 : 0;
    i_max_axis = max_ax[2] < max_ax[i_max_axis] ? i_max_axis : 2;

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

    //split vectices in two lists: left and rigth according to the median and max axe

    cout<<"Mediane coordinate "<< i_max_axis << " Value of mediane axe " << n->data.mediane[i_max_axis] << endl;

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

    cout<<"Output percentage "<<percentage<<endl;
    cout<<endl;

    // apply recursion if the list is not empty
    n->leftChild = listPu.size() != list.size() ? buildKDTree(mesh,listPu,percentage) : nullptr;
    n->rightChild = listPl.size() != list.size() ? buildKDTree(mesh,listPl,percentage) : nullptr;

    return n;
}



/*
    cout<<"sort mesh after sorting "<<endl;
    for (unsigned int j = 0; j<list.size();j++){
        cout<<"Triangle "<< j <<" "<<endl;
         for (unsigned int k=0; k<3; k++){
             cout<<"Vertex "<< k << endl;
            for (unsigned int i=0;i<3;i++) {
                cout<<sort_mesh[3*j+k][i]<<" ";
            }
            cout<<endl;
        }
        cout<<endl;
    }
 */

/*
for (unsigned int j=0;j<list.size();j++){
    cout<<endl;
    cout<<"Triangle "<< j <<endl;
    cout<<"Coordinates imaxaxis "<<endl;
    for (unsigned int k=0;k<3;k++)
        cout<<" "<<mesh.V[mesh.T[list[j]].v[k]].p[i_max_axis]<<" ";
}
*/

/*
//cout<<" listPu "<<endl;
for(unsigned int i=0;i<listPu.size();i++){
    //cout<< listPu[i]<<" ";
}
//cout<<endl;
//cout<<" listPl "<<endl;
for(unsigned int i=0;i<listPl.size();i++){
    // cout<< listPl[i]<<" ";
}*/
