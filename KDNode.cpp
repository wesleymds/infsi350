#include "KDNode.h"
#include <vector>
#include <algorithm>
using namespace std;
int i_max_axis;

bool coordinate_sort (Vec3f v1, Vec3f v2) 
{ return (v1[i_max_axis]<v2[i_max_axis]); };


KDNode* KDNode :: buildKDTree_test (const Mesh& mesh, const std::vector<int>& list){
	
	KDNode* n = new KDNode();
	//find the extension of each axe of the bounding boxe		
	if (list.size()!=1) {		
		Vec3f max(mesh.V[list[1]].p);
		Vec3f min(mesh.V[list[1]].p);		
			for (unsigned int i=0;i<3;i++) {
				for (unsigned int j=0;j<list.size();j++) {
					if (max[i]<mesh.V[list[j]].p[i]) max[i]=mesh.V[list[j]].p[i];
					if (min[i]>mesh.V[list[j]].p[i]) min[i]=mesh.V[list[j]].p[i];
				}
			}
	// find the max extension
		Vec3f max_ax;
		for (unsigned int i=0;i<3;i++) {max_ax[i]=max[i]-min[i];}		
	// find the axe that correspond to the maximum extension
		int a(0),b(0);
		if (max_ax[0]<max_ax[1]) a=1;
		else a = 0;
		if (max_ax[1]<max_ax[2]) b = 2;
		else b = 1;
		if (max_ax[a]<max_ax[b]) i_max_axis=b;
		else i_max_axis=a;
		n->data.max_axe =  i_max_axis;		
		
	// find median sample:
	
	std::vector<Vec3f> sort_mesh(list.size());	
		for (unsigned int i = 0; i<list.size();i++) {
			for (unsigned int j=0;j<3;j++) {
				sort_mesh[i][j]=mesh.V[list[i]].p[j];
			}
		}
				
	// sort by the max axe	
	std::sort(sort_mesh.begin(),sort_mesh.end(),coordinate_sort);
		
	// find mediane (according to max axe) in the current list		
	Vec3f med;
	med = sort_mesh[int(sort_mesh.size()/2)];	
	n->data.mediane = med;	
	//split vectices in two lists: left and rigth according to the median and max axe
	std::vector<int> listPu(0);
	std::vector<int> listPl(0);	
	for (unsigned int i=0;i<list.size();i++){
		if (mesh.V[list[i]].p[i_max_axis]<n->data.mediane[i_max_axis])
			listPu.push_back(list[i]); 
		else listPl.push_back(list[i]);
	}
	 cout<<" listPu "<<endl;
	 for(unsigned int i=0;i<listPu.size();i++){
	 cout<< listPu[i]; 
	 } 
	 cout<<endl;
	 cout<<" listPl "<<endl;
	 for(unsigned int i=0;i<listPl.size();i++){
	 cout<< listPl[i]; 
	 } 
	 cout<<endl;	
	// apply recursion if the list is not empty  
	if (!listPu.empty())
	n->leftChild = KDNode :: buildKDTree_test(mesh,listPu);
	if (!listPl.empty())
	n->rightChild = KDNode :: buildKDTree_test (mesh,listPl);
		
 }
	return n;
};
