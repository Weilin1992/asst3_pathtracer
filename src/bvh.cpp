#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 { namespace StaticScene {

				BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
													 size_t max_leaf_size) {


					// TODO:
					// Construct a BVH from the given vector of primitives and maximum leaf
					// size configuration. The starter code build a BVH aggregate with a
					// single leaf node (which is also the root) that encloses all the
					// primitives.


					this->primitives = _primitives;
					std::vector<Primitive *> prims;
					// copy user's primitives data to a local version, which can be modified;
					for(size_t i = 0; i < _primitives.size(); i++){
						prims.push_back(_primitives[i]);
					}

					BBox bb;
					for (size_t i = 0; i < _primitives.size(); ++i) {
						bb.expand(_primitives[i]->get_bbox());
					}

					this->root = new BVHNode(bb, 0, _primitives.size());
					this->buildBVH(prims,max_leaf_size);
					this->primitives = prims;

				}

				BVHAccel::~BVHAccel() {

					// TODO:
					// Implement a proper destructor for your BVH accelerator aggregate

				}

				BBox BVHAccel::get_bbox() const {
					return root->bb;
				}

				bool BVHAccel::intersect(const Ray &ray) const {

					// TODO:
					// Implement ray - bvh aggregate intersection test. A ray intersects
					// with a BVH aggregate if and only if it intersects a primitive in
					// the BVH that is not an aggregate.
					Intersection* isect = new Intersection();
					find_closest_hit(ray, root, isect);
					bool result = !(isect->primitive == NULL);
					delete isect;
					return result;
				}

				bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

					// TODO:
					// Implement ray - bvh aggregate intersection test. A ray intersects
					// with a BVH aggregate if and only if it intersects a primitive in
					// the BVH that is not an aggregate. When an intersection does happen.
					// You should store the non-aggregate primitive in the intersection data
					// and not the BVH aggregate itself.

					find_closest_hit(ray, root, i);
					bool result = !(i->primitive == NULL);
					return result;
				}


				void BVHAccel::find_closest_hit(const Ray &ray, BVHNode* node, Intersection *i) const{

					double t0 = 0;
					double t1 = INF_D;
					if(! node->bb.intersect(ray, t0, t1) || t0 > i->t){
						return ;
					}

					if(node->isLeaf()){
						// get primitive list of node;
						std::vector<Primitive*> prims;
						for(size_t p = node->start; p < (node->start + node->range); p++){
							prims.push_back(primitives[p]);
						}
						// check intersection of ray and primitives
						for(size_t p = 0; p < prims.size(); p++){
							Intersection* isect = new Intersection();
							bool flag = prims[p]->intersect(ray, isect);
							if(flag && isect->t < i->t){

								// update the closest hit data;
								i->n = isect->n;
								i->bsdf = isect->bsdf;
								i->primitive = isect->primitive;
								i->t = isect->t;
							}
							delete isect;
						}
					}else{
						find_closest_hit(ray, node->l, i);
						find_closest_hit(ray, node->r, i);
					}

				}

				bool BVHAccel::splitNode(BVHNode *parent, std::vector<Primitive *> &prims) {

					Vector2D cost_split = Vector2D(INF_D,0);
					int axis = 0;
					for(int i = 0; i < 3; i++){
						Vector2D c = planeCost(prims,i,parent);
						cout<<"cost of " << i <<" axis: "<<c.x<<endl;
						if(cost_split.x > c.x){
							cost_split = c;
							axis = i;
						}
					}


					std::vector<Primitive* >::iterator bound;

					bound = std::partition(prims.begin() + parent->start,
																 prims.begin() + parent->start + parent->range,
																 [axis,cost_split](Primitive * prim){
																		 bool r = (cost_split.y > prim->get_bbox().centroid()[axis]);
																		 return r;
																 });

					size_t range_left = 0;
					BBox bboxl;

					for(vector<Primitive*>::iterator it = prims.begin() + parent->start;it != bound; it++){
						Primitive * p = *it;
						bboxl.expand(p->get_bbox());
						range_left++;
					}
					if(range_left == parent->range){
						return false;
					}

					size_t range_right = 0;
					BBox bboxr;
					for(auto it  = bound; it != prims.begin() + parent->start + parent->range;it++){
						Primitive *p = *it;
						bboxr.expand(p->get_bbox());
						range_right++;
					}
					if(range_right == parent->range){
						return false;
					}

					parent->l = new BVHNode(bboxl,parent->start,range_left);
					parent->r = new BVHNode(bboxr,parent->start + range_left,range_right);
					cout<<"left range:"<<range_left<<" "<<"right range"<<range_right<<endl;
					return true;
				}

				void BVHAccel::buildBVH(std::vector<Primitive *> &prims,size_t max_leaf_size) {
					std::stack<BVHNode*> s;
					s.push(root);
					int numofNodes = 0;
					while(!s.empty()){
						BVHNode *node = s.top();
						s.pop();
						if(node->range > max_leaf_size) {
							numofNodes++;
							cout<<numofNodes<<"th nodes to split"<<endl;
							if(splitNode(node,prims)){
								s.push(node->l);
								s.push(node->r);
							}
						}
					}
				}

				BVHNode *BVHAccel::get_root() const {
					return root;
				}

				Vector2D BVHAccel::planeCost(std::vector<Primitive*> &prims, int axis, BVHNode *parent) {

					Vector2D result = Vector2D(INF_D,0);

					auto begin = prims.begin() + parent->start;
					auto end = prims.begin() + parent->start + parent->range;

					std::sort(begin,end,[axis](Primitive* a,Primitive* b){
							bool r = a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
							return r;
					});

					vector<double> bottom(parent->range);
					vector<double> top(parent->range);

					BBox b1,b2;
					for(auto it = begin; it != end; it++){
						Primitive *p = *it;
						BBox test = p->get_bbox();
						b1.expand(p->get_bbox());
						bottom[it - begin] = b1.surface_area() * (it - begin + 1);
						int i = it - begin;
						p = *(end - i - 1);
						b2.expand(p->get_bbox());
						top[parent->range - i - 1] = b2.surface_area()*(it - begin  + 1);

					}

					for(int i = 0; i < parent->range; i++){
						double c = bottom[i] + top[i];
						if(c < result.x){
							result.x = c;
							Primitive* p = *(begin + i);
							result.y = p->get_bbox().centroid()[axis];
						}
					}

					return result;
				}

		}  // namespace StaticScene
}  // namespace CMU462
