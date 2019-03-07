#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
#include <algorithm>

using namespace std;

namespace CMU462 {
namespace StaticScene {


BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  this->primitives = _primitives;

  // TODO (PathTracer):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  //numrecursion = 0;
  BBox broot;
  for (size_t i = 0; i < primitives.size(); ++i) {
    broot.expand(primitives[i]->get_bbox());
  }

  root = new BVHNode(broot, 0, primitives.size());
  //root = new BVHNode(broot, 0, 0);
  buildRecursive(root, broot, max_leaf_size);

  //cout << "FINISHED" << endl;
}


BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate
  destroyRecursive(root);
}

void BVHAccel::destroyRecursive(BVHNode* node)
{
  if(node->l->isLeaf())
    delete node->l;
  else
    destroyRecursive(node->l);
  if(node->r->isLeaf())
    delete node->r;
  else
    destroyRecursive(node->r);
}

void BVHAccel::buildRecursive(BVHNode* parent, BBox &broot, size_t max_leaf_size)
{
  //cout << "Begin" << endl;
  //numrecursion++;
  //if(numrecursion > 100)
  //  return;
  if(parent->range <= max_leaf_size)
  {
    //cout << parent->range << endl;
    //cout << "Ending" << endl;
    return;
     
  }
  
  Vector3D mintotal = broot.min;
  Vector3D maxtotal = broot.max;
  Vector3D exttotal = broot.extent;
  //cout << parent->range << endl;

  BBox bb[48]; //3*16
  int counts[48] = {0};
  int numBuckets = 16;
  //
  double costs[45] = {0.};
  //
  double arealt[45] = {0.};
  double areart[45] = {0.};
  double numlt[45] = {0.};
  double numrt[45] = {0.};

  for(int i = 0; i < 3; i++) //for each axis
  {
    //split buckets based evenly along axis
    double maxD = maxtotal[i];
    double minD = mintotal[i];
    double extent = exttotal[i] / ((double) numBuckets*(1. - 10.*EPS_D)) + EPS_D;

    for (int p = parent->start; p < parent->start + parent->range; ++p) //for each primitive in current node
    {
      // compute bucket
      //cout << p << endl;

      BBox prim = primitives[p]->get_bbox();
      Vector3D cent = prim.centroid();
      int bucket = min(max(0., floor ( (cent[i] - minD) / extent ) ), 15.0 ) + i * numBuckets ;

      //cout << (cent[i] - minD) << endl;
      //cout << extent << endl;
      //cout << bucket << endl;
      //expand
      bb[bucket].expand(prim);
      //update count
      counts[bucket]++;
    }
    //First pass collect Surface Area and Number of primitives left to right
    
    // LEFT
    //Left traversal
    numlt[15*i] = counts[16*i];
    arealt[15*i] = bb[16*i].surface_area();
    BBox templt = bb[16*i];
    for(int j = 1; j < 15; j++)
    {
      // Get number and surface area
      numlt[ j + i*15 ] = numlt[ j + i*15 - 1 ] + counts[ j + i*16 ];
      templt.expand(bb[ j + i*numBuckets ]);
      arealt[ j + i*15 ] = templt.surface_area();
    }
    
    // RIGHT
    //Right traversal
    numrt[15*i+14] = counts[16*i+15];
    BBox temprt = bb[16*i + 15];
    areart[15*i + 14] = bb[16*i + 15].surface_area();
    for(int j = 13; j >= 0; j--)
    {
      //Get count of primitives and surface area
      numrt[ j + i*15] += numrt[ j + i*15 + 1] + counts[ j + 1 + i*16 ];
      temprt.expand(bb[ j + 1 + i*numBuckets ]);
      areart[ j + i*15 ] = temprt.surface_area();
    }
  }

  //calculate all costs
  int indexMin = -1;
  double costMin = INF_D;
  for(int j = 0; j < 45; j++)
  {
    if(numlt[j] != 0 && numrt[j] != 0)
      costs[j] = numlt[j]*arealt[j] + numrt[j]*areart[j];
    else
      costs[j] = INF_D;
    if (costs[j] < costMin)
      {
        costMin = costs[j];
        indexMin = j;
      }
  }

  int axis = indexMin / 15;
  double minD = mintotal[axis];
  double extent = exttotal[axis] + EPS_D;
  double splitThresh = ((double) (indexMin % 15) + 1.)/((double) numBuckets);
  std::vector<Primitive *> sorted_primitives;
  int numLeft = 0;
  int numRight = 0;

  int ntotal = parent->range;
  sorted_primitives.resize(ntotal);
  BBox leftCh = BBox();
  BBox rightCh = BBox();
  for (int p = parent->start; p < parent->start + parent->range; ++p) //for each primitive in current node
  {
    // compute bucket
    BBox prim = primitives[p]->get_bbox();
    Vector3D cent = prim.centroid();
    //cout << (cent[axis] - minD) / extent << endl;
    if ( (cent[axis] - minD) / extent  <= splitThresh) 
    {
      sorted_primitives[numLeft] = primitives[p];
      leftCh.expand(prim);
      numLeft++;
    }
    else{
      sorted_primitives[ntotal - numRight - 1] = primitives[p];
      rightCh.expand(prim);
      numRight++;
    }
  }
  //cout << "checkpoint 1" << endl;
  //cout << numRight << endl;
  //cout << numLeft << endl;
  
  for (int p = parent->start, a = 0; p < parent->start + parent->range; ++p, ++a)
  {
    //cout << "p: " << p << endl;
    //cout << "a: " << a << endl;
    primitives[p] = sorted_primitives[a];
  }

  //cout << "checkpoint 2" << endl;
  //left
  
  parent->l = new BVHNode(leftCh, parent->start, numLeft);
  buildRecursive(parent->l, leftCh, max_leaf_size);
  //right
  //if(numRight > max_leaf_size)
  
  parent->r = new BVHNode(rightCh, parent->start + numLeft , numRight);
  buildRecursive(parent->r, rightCh, max_leaf_size);
  
    

  //cout << "checkpoint 3" << endl;
  //return;
}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. 
  
  double t0 = INF_D, t1 = -INF_D;

  if (root->bb.intersect(ray,t0,t1))
    return recursive_intersect(ray,root);
  else 
   return false;
  /*
 
  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray)) hit = true;
  }

  return hit;
  */
  
}

bool BVHAccel::recursive_intersect(const Ray& r, BVHNode* node) const{
  bool hit = false;
  bool hit2 = false;
  if(node->isLeaf())
  {
    for (size_t p = node->start; p < node->start + node->range; ++p) 
      if (primitives[p]->intersect(r)) hit = true;
    return hit;
  }
  
  double t0,t1, t2,t3;
  bool hitLeft = false, hitRight = false;

  hitLeft = node->l->bb.intersect(r,t0,t1);
  hitRight = node->r->bb.intersect(r,t2,t3);

  //if (t0 < t2)
  if(hitLeft)
    hit = recursive_intersect(r,node->l);
  if(hitRight)
    hit2 = recursive_intersect(r,node->r);

  return (hit || hit2);
}

bool BVHAccel::recursive_intersect(const Ray& r, Intersection* i, BVHNode* node) const{
  bool hit = false;
  bool hit2 = false;
  if(node->isLeaf())
  {
    for (size_t p = node->start; p < node->start + node->range; ++p) 
      if (primitives[p]->intersect(r,i)) hit = true;
    return hit;
  }
  
  double t0,t1, t2,t3;
  t0 = INF_D;
  t2 = -INF_D;
  bool hitLeft = false, hitRight = false;
  
  //if (node->l != NULL)
    hitLeft = node->l->bb.intersect(r,t0,t1);
  //if (node->r != NULL)
    hitRight = node->r->bb.intersect(r,t2,t3);
  
  if(hitLeft)
    hit = recursive_intersect(r,i,node->l);
  if(hitRight)
    hit2 = recursive_intersect(r,i,node->r);

  return (hit || hit2);
}


bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  
  double t0 = INF_D, t1 = -INF_D;
//double t0 = INF_D, t1 = -INF_D;

 if (root->bb.intersect(ray,t0,t1))
    return recursive_intersect(ray,isect, root);
  else 
    return false;

   // Old starter code
   /*
  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray, isect)) hit = true;
  }

  return hit;
  */
  
}

}  // namespace StaticScene
}  // namespace CMU462
