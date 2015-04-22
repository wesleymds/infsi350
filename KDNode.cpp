#include "KDNode.h"
#include <vector>
#include <algorithm>
using namespace std;
int i_max_axis;

bool coordinate_sort (Vec3f v1, Vec3f v2) 
{ return (v1[i_max_axis]<v2[i_max_axis]); };



//list of triangles is input

KDNode* KDNode :: buildKDTree_test (const Mesh& mesh, const std::vector<int>& list,float percentage){
	
	cout<<"Input percentage"<<percentage<<endl;
	KDNode* n = new KDNode();
	
	// find the extension of each axe of the bounding boxe		
	 if (percentage <= 0.55f && list.size()>5) {	
		Vec3f max(mesh.V[mesh.T[list[1]].v[1]].p);
		Vec3f min(mesh.V[mesh.T[list[1]].v[1]].p);	
		// k is the number of vectex in triangle
		for(unsigned int k=0;k<3;k++) {	
			// i is the number of coordinate of vertex
			for (unsigned int i=0;i<3;i++) {
				// j is the number of triangle
				for (unsigned int j=0;j<list.size();j++) {
					if (max[i]<mesh.V[mesh.T[list[j]].v[k]].p[i]) max[i]=mesh.V[mesh.T[list[j]].v[k]].p[i];
					if (min[i]>mesh.V[mesh.T[list[j]].v[k]].p[i]) min[i]=mesh.V[mesh.T[list[j]].v[k]].p[i];
				}
			}
		}
	// axis-aligned bounding box is the cartesian product of max and min
	//n.data.max = max;
	//n.data.min = min;
	// find the max extension
		Vec3f max_ax;
		for (unsigned int i=0;i<3;i++) {max_ax[i]=max[i]-min[i];}		
	// find the axe that corresponds to the maximum extension
		int a(0),b(0);
		if (max_ax[0]<max_ax[1]) a=1;
		else a = 0;
		if (max_ax[1]<max_ax[2]) b = 2;
		else b = 1;
		if (max_ax[a]<max_ax[b]) i_max_axis=b;
		else i_max_axis=a;
		n->data.max_axe =  i_max_axis;		
		
	// find median sample:
	
	//cout<<"sort mesh before sorting "<<endl;	
	std::vector<Vec3f> sort_mesh(3*list.size());
	for (unsigned int j = 0; j<list.size();j++){
		//cout<<"Triangle "<< j <<" "<<endl;
		 for (unsigned int k=0; k<3; k++){
			// cout<<"Vertex "<< k << endl;
			for (unsigned int i=0;i<3;i++) {
				sort_mesh[3*j+k][i]=mesh.V[mesh.T[list[j]].v[k]].p[i];
				//cout<<sort_mesh[j+k][i]<<" ";
			}
			//cout<<endl;
		}
		//cout<<endl;
	}
	
	std::sort(sort_mesh.begin(),sort_mesh.end(),coordinate_sort);
	
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
	
	// find mediane (according to max axe) in the current list		
	Vec3f med;
	med = sort_mesh[int(sort_mesh.size()/2)];	
	
	n->data.mediane = med;	
	
	//split vectices in two lists: left and rigth according to the median and max axe
	
	
		cout<<"Mediane coordinate "<< i_max_axis << " Value of mediane axe "  <<med[i_max_axis]<<endl;
		/*
		for (unsigned int j=0;j<list.size();j++){
			cout<<endl;
			cout<<"Triangle "<< j <<endl;
			cout<<"Coordinates imaxaxis "<<endl;
			for (unsigned int k=0;k<3;k++)
				cout<<" "<<mesh.V[mesh.T[list[j]].v[k]].p[i_max_axis]<<" ";
		}
		*/
	
	std::vector<int> listPu(0);
	std::vector<int> listPl(0);	
	std::vector<int> s(list.size(),0);
	
	for (unsigned int j=0;j<list.size();j++){
		for (unsigned int k=0;k<3;k++){
			s[j] = s[j]+(mesh.V[mesh.T[list[j]].v[k]].p[i_max_axis]<n->data.mediane[i_max_axis]);
		}
		//cout<<"s "<< j <<" "<< s[j] <<endl;;
	}	
		
	for (unsigned int j=0;j<list.size();j++){
		if (s[j]==3) listPu.push_back(list[j]); 
		if (s[j]<3 && 0<s[j]) {listPu.push_back(list[j]); listPl.push_back(list[j]);}
		if (s[j]==0) listPl.push_back(list[j]);
	}		
	
	 cout<<"Size of all elements "<<list.size()<<endl;	
	 //cout<<" listPu "<<endl;
	 for(unsigned int i=0;i<listPu.size();i++){
	 //cout<< listPu[i]<<" "; 
	 } 
	 //cout<<endl;
	 //cout<<" listPl "<<endl;
	 for(unsigned int i=0;i<listPl.size();i++){
	// cout<< listPl[i]<<" "; 
	 } 
	 // compute intersection of two point lists in percentage
	 
	 //sort element in the array
	 std::sort (listPu.begin(),listPu.end());  
	 std::sort (listPl.begin(),listPl.end());   
	 
	// find the maximal size of the list 
	int max_size(0);
	if (listPu.size()>listPl.size()) max_size=listPu.size();
	else max_size=listPl.size();
  
	std::vector<int> v(max_size);
	std::vector<int>::iterator it;
	
	it=std::set_intersection (listPu.begin(),listPu.end(), listPl.begin(),listPl.end(), v.begin());
	v.resize(it-v.begin());
	percentage = (1.0*v.size())/(1.0*max_size); 	
	cout<<"Output percentage "<<percentage<<endl;	 
	cout<<endl;	
	// apply recursion if the list is not empty  
	
	if (!listPu.empty() && listPu.size()!=list.size()){
		n->leftChild = KDNode :: buildKDTree_test(mesh,listPu,percentage);}
		
	if (!listPl.empty() && listPl.size()!=list.size()){
		n->rightChild = KDNode :: buildKDTree_test (mesh,listPl,percentage);}	
 }
 
	return n;
};
