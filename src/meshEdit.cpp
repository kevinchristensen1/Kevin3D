#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"
#include <iomanip>
#include <cmath>
#include "CMU462/quaternion.h"
#include <iostream>
//#include <quaternion.h>

namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

  // This function is only valid for triangles!
  if(e0->halfedge()->face()->degree() > 3 || e0->halfedge()->twin()->face()->degree() > 3)
    return e0->halfedge()->vertex();

  //when could this happen, idk, but check for it anyways
  if(e0->halfedge()->isBoundary() && e0->halfedge()->twin()->isBoundary())
    return e0->halfedge()->vertex();

  // TODO: check lengths to see if it is aggressively obtuse
  //Since this function is only valid for triangles, we know there 
  // will be one new vertex, and six new halfedges.
  // Unless one face is a boundary, in which case we

  //TODO what if only one boundary?
  //bool f0isBoundary = e0->halfedge()->isBoundary();
  //bool f1isBoundary = e0->halfedge()->twin()->isBoundary();

  HalfedgeIter h0 = e0->halfedge();
  FaceIter f0 = h0->face();
  //f0->halfedge() = h0;
  FaceIter f1 = h0->twin()->face();
  //f1->halfedge() = h0->twin();

  FaceIter f2 = newFace();
  FaceIter f3 = newFace();
  VertexIter v = newVertex();
  EdgeIter e1 = newEdge();
  EdgeIter e2 = newEdge();
  EdgeIter e3 = newEdge();

  std::vector<HalfedgeIter> hnew;
  std::vector<HalfedgeIter> hnew2;

  for(int j = 0; j < 3; j++)
    hnew.push_back(newHalfedge());

  v->position = e0->centroid();

  HalfedgeIter h = h0;
  HalfedgeIter h2 = h->next();
  HalfedgeIter h3 = h2->next();

  
  //edges
  hnew[0]->edge() = e1;
  e1->halfedge() = hnew[0];
  hnew[1]->edge() = e1;
  hnew[2]->edge() = e2;
  e2->halfedge() = hnew[2];

  hnew[0]->vertex() = v;
  hnew[1]->vertex() = h3->vertex();
  hnew[2]->vertex() = v;
  v->halfedge() = hnew[2];
  //faces
  hnew[0]->face() = f0;
  f0->halfedge() = h0;

  hnew[1]->face() = f2;
  f2->halfedge() = hnew[1];
  h2->face() = f2;
  hnew[2]->face() = f2;
  h3->face() = f0;

  //twins
  hnew[0]->twin() = hnew[1];
  hnew[1]->twin() = hnew[0];

  //next
  h0->next() = hnew[0];
  hnew[0]->next() = h3;
  hnew[1]->next() = hnew[2];
  hnew[2]->next() = h2;
  h2->next() = hnew[1];

  hnew[0]->twin() = hnew[1];
  hnew[1]->twin() = hnew[0];
  

  h  = h0->twin();
  h2 = h->next();
  h3 = h2->next();

  for(int j = 0; j < 3; j++)
    hnew2.push_back(newHalfedge());
  
  //e2->halfedge() = h;
  //edges
  h->edge() = e2;
  hnew2[0]->edge() = e3;
  e3->halfedge() = hnew2[0];
  hnew2[1]->edge() = e3;
  //e2->halfedge() = h;
  hnew2[2]->edge() = e0;
  //e2->halfedge() = hnew2[2];

  hnew2[0]->vertex() = v;
  hnew2[1]->vertex() = h3->vertex();
  hnew2[2]->vertex() = v;
  //v->halfedge() = hnew2[0]; //not needed
  //faces
  hnew2[0]->face() = f1;
  f1->halfedge() = h;
  hnew2[1]->face() = f3;
  f3->halfedge() = hnew2[1];
  hnew2[2]->face() = f3;
  h2->face() = f3;
  h3->face() = f1;
  
  //next
  h->next() = hnew2[0];
  hnew2[0]->next() = h3;
  hnew2[1]->next() = hnew2[2];
  hnew2[2]->next() = h2;
  h2->next() = hnew2[1];
  
  
  //twin
  hnew2[0]->twin() = hnew2[1];
  hnew2[1]->twin() = hnew2[0];

  h0->twin() = hnew2[2];
  hnew2[2]->twin() = h0;

  hnew[2]->twin() = h;
  h->twin() = hnew[2];
  e0->isNew = false;
  e2->isNew = false;
  e1->isNew = true;
  e3->isNew = true;
  //showError("splitEdge() not implemented.");
  return v;
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.
  
  HalfedgeIter h0 = e->halfedge();
  VertexIter v = h0->vertex();
  //if(edges.size() <= 6 || faces.size() <= 4)
  //  return v;
  v->position = e->centroid(); //update position and reuse, saves iterating around one vertex!

  HalfedgeIter ht = h0->twin();
  VertexIter vt = ht->vertex();
  HalfedgeIter h = ht;
  
  //iterate around twins vertex
  do{
    h = h->twin()->next();
    h->vertex() = v;
  }
  while( h != ht );
  
  if(h0->face()->degree() == 3)
  {
    FaceIter f = h0->face();
    HalfedgeIter h2 = h0->next();
    HalfedgeIter h3 = h2->next();

    VertexIter v1 = h3->vertex();
    v1->halfedge() = h2->twin();

    EdgeIter e1 = h3->edge();
    EdgeIter e2 = h2->edge();
    h2->twin()->edge() = e1;
    e1->halfedge() = h3->twin();
    h2->twin()->edge() = e1;

    v->halfedge() = h3->twin();
    
    h2->twin()->twin() = h3->twin();
    h3->twin()->twin() = h2->twin();

    deleteEdge(e2);
    deleteHalfedge(h2);
    deleteHalfedge(h3);
    deleteFace(f);
  }
  else{
    
    HalfedgeIter h2 = h0->next();
    HalfedgeIter h3 = h2->next();
    while(h3->next() != h0)
      h3 = h3->next();
    //now h3->next() == h0
    v->halfedge() = h2; //guarenteed to not be deleted
    h0->face()->halfedge() = h2;
    h3->next() = h2;
  }

  if(ht->face()->degree() == 3)
  {
    FaceIter f = ht->face();
    HalfedgeIter h2 = ht->next();
    HalfedgeIter h3 = h2->next();

    VertexIter v1 = h3->vertex();
    v1->halfedge() = h2->twin();

    EdgeIter e1 = h3->edge();
    EdgeIter e2 = h2->edge();
    h2->twin()->edge() = e1;
    e1->halfedge() = h3->twin();
    h2->twin()->edge() = e1;

    h2->twin()->twin() = h3->twin();
    h3->twin()->twin() = h2->twin();
    deleteEdge(e2);
    deleteHalfedge(h2);
    deleteHalfedge(h3);
    deleteFace(f);
  }
  else{
    HalfedgeIter h2 = ht->next();
    HalfedgeIter h3 = h2->next();
    while(h3->next() != ht)
      h3 = h3->next();

    ht->face()->halfedge() = h2;
    //now h3->next() == h0
    h3->next() = h2;
  }
  //vt->halfedge() = v->halfedge();

  //got it, implemented above
  //don't forget at end
  //v->halfedge() = hsomething

  //delete twin's vertex, two halfedges, and edge
  deleteVertex(vt);
  deleteEdge(e);
  deleteHalfedge(h0);
  deleteHalfedge(ht);
  //showError("collapseEdge() not implemented.");
  return v;
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
  showError("collapseFace() not implemented.");
  return VertexIter();
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // TODO: (meshEdit)
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.
  showError("eraseVertex() not implemented.");
  return FaceIter();
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should erase the given edge and return an iterator to the
  // merged face.

  showError("eraseVertex() not implemented.");
  return FaceIter();
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

  //showError("flipEdge() not implemented.");
  //if e0->face or e0->twin->face are boundary return e0
  if(e0->isBoundary())
    return e0;

  //Cool we are provided a wrapper for exactly what I just did
  //if(h0->face()->isBoundary() || h0->twin()->face()->isBoundary())
  //  return e0;

  HalfedgeIter h0 = e0->halfedge();

  FaceIter f0 = h0->face();
  FaceIter f1 = h0->twin()->face();
  
  double eps = 0.000005;

  //if(h0->face()->degree() == 3)
  //{
    
  Vector3D pos0 = h0->vertex()->position;
  Vector3D pos1 = h0->next()->next()->vertex()->position;
  Vector3D vec0 = pos1-pos0;
  Vector3D pos2 = h0->twin()->next()->next()->vertex()->position;
  Vector3D vec1 = pos2-pos0;
  Vector3D pos3 = h0->next()->vertex()->position;

  Vector3D vec2 = pos3 - pos0;
  double t1 = acos(dot(vec0,vec2)/(vec0.norm()*vec2.norm()));
  double t2 = acos(dot(vec1,vec2)/(vec1.norm()*vec2.norm()));
  //std::cout << setprecision(5) << sameLine << std::endl;
  
  if(t1+t2 > PI - 0.1)
    return e0;

  vec0 = pos1 -pos3;
  vec1 = pos2 - pos3;
  vec2 = pos0 - pos3;

  t1 = acos(dot(vec0,vec2)/(vec0.norm()*vec2.norm()));
  t2 = acos(dot(vec1,vec2)/(vec1.norm()*vec2.norm()));
  if(t1+t2 > PI - 0.1)
    return e0;
  

  
    
      /*
    pos0 = h0->next()->vertex()->position;
    pos1 = h0->next()->next()->vertex()->position;
    pos2 = h2->vertex()->position;
    vec0 = pos1-pos0;
    vec1 = pos2-pos1;
    sameLine = abs(cross(vec0,vec1).norm());
    //std::cout << setprecision(5) << sameLine << std::endl;
    if(sameLine <= eps)
      return e0;
      */
  //}
  //std::cout << "f0: " << elementAddress(h0->face()) << "h0: " << elementAddress(h0->twin()->twin()) <<std::endl;
  if(h0->face()->degree() > 3 || h0->twin()->face()->degree() > 3)
  {
    double normDiff = std::abs( (f0->normal() - f1->normal()).norm());
    if( normDiff > eps)
      return e0;
  }

  HalfedgeIter h = h0;

  while( h->next() != h0 )
    h=h->next();
    
  HalfedgeIter h2 = h0->twin();
  while( h2->next() != h0->twin() )
    h2=h2->next();
  
  //VertexIter v = h0->vertex();
  //First handle h0
  HalfedgeIter ht = h0->twin();
  VertexIter v = h0->vertex();

  v->halfedge() = h0->twin()->next(); //v0 points to h0->twin's next
  ht->next()->next()->vertex()->halfedge() = h0; //point next vertex to h0
  h0->vertex() = ht->next()->next()->vertex(); //h0 is given new vertex

  h0->next()->vertex()->halfedge() = h0->next(); //point other vertex to h0's next to make sure it doesn't point to h0's twin
  h0->next()->next()->vertex()->halfedge() = ht; //point vertex v1 to h1's twin
  ht->vertex() = h0->next()->next()->vertex(); //assign new vertex to h0's twin
  
  h0->next()->face() = f1; //face is swapped
  
  // now we have h where h->next = h0
  h->next() = ht->next(); //assign it's next ptr to h0 twin's next
  h2->next() = h0->next(); //assign pointer pointing to h1 to instead point to h0's next.

  h2 = h0->next()->next(); //save value in h2
  h0->next()->next() = ht;

  h0->next() = h2;
  //h0->twin() = h0->twin();

  //now finish with twin
  HalfedgeIter h1 = ht;

  h1->next()->face() = f0; //face is swapped
  h2= h1->next()->next();
  h1->next()->next() = h0;
  h1->next() = h2;

  h1->twin() = h0;
  h0->twin() = h1;
  //Probs unecessary
  f0->halfedge() = h0;
  f1->halfedge() = h1;

  return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.
  for (VertexIter v = verticesBegin(); v != verticesEnd(); v++)
    v->newPosition = v->position;

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.
  for (EdgeIter e = edgesBegin(); e != edgesEnd(); e++)
    e->newPosition = e->centroid();
  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  for (FaceIter f = facesBegin(); f != facesEnd(); f++)
    f->newPosition = f->centroid();

  return;
  //showError("computeLinearSubdivisionPositions() not implemented.");
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)
  
  // TODO face
  for (FaceIter f = facesBegin(); f != facesEnd(); f++)
    f->newPosition = f->centroid();

  // TODO edges
  for (EdgeIter e = edgesBegin(); e != edgesEnd(); e++)
    e->newPosition = (e->halfedge()->face()->newPosition + 
                      e->halfedge()->twin()->face()->newPosition + 
                      e->centroid()*2.0)/4.0;
  // TODO vertices
  for (VertexIter v = verticesBegin(); v != verticesEnd(); v++)
  {
    HalfedgeIter h = v->halfedge();
    Vector3D Q = Vector3D();
    Vector3D R = Vector3D();
    double n = 0;
     do
     {
        Q += h->face()->newPosition;
        R += h->edge()->centroid();
        h = h->twin()->next();
        n++;
     }
     while( h != v->halfedge() );
    Q = Q/n;
    R = R/n;
    v->newPosition = (Q + (2.0)*R + (n-3.0)*v->position)/n;
  }
    
  //showError("computeCatmullClarkPositions() not implemented.");
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)
  Index i = 0;
  // TODO Iterate over vertices, assigning values to Vertex::index
  for (VertexIter v = verticesBegin(); v != verticesEnd(); v++,i++)
    v->index = i;
    
  // TODO Iterate over edges, assigning values to Edge::index
  for (EdgeIter e = edgesBegin(); e != edgesEnd(); e++,i++)
    e->index = i;

  // TODO Iterate over faces, assigning values to Face::index
  for (FaceIter f = facesBegin(); f != facesEnd(); f++,i++)
    f->index = i;
    
  //std::cout << "assignSubdivisionIndices Iend: " << i << std::endl;
  //showError("assignSubdivisionIndices() not implemented.");
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.
  subDVertices.resize(vertices.size() + edges.size() + faces.size());
  Index i = 0;
  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.
  for (VertexIter v = verticesBegin(); v != verticesEnd(); v++,i++)
    subDVertices[i]= v->newPosition;
  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.
  for (EdgeIter e = edgesBegin(); e != edgesEnd(); e++,i++)
    subDVertices[i] = e->newPosition;

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  for (FaceIter f = facesBegin(); f != facesEnd(); f++,i++)
    subDVertices[i] = f->newPosition;
  //std::cout << "buildSubdivisionVertexList Iend: " << i << std::endl;
  //showError("buildSubdivisionVertexList() not implemented.");
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  int N = 0;
  Index i = 0;

  for (FaceIter f = facesBegin(); f != facesEnd(); f++)
  {
    HalfedgeIter h0 = f->halfedge();
    N= f->degree();
    // TODO loop around face
    HalfedgeIter h = h0->next();
    HalfedgeIter h2 = h0;
    vector<Index> quad( 4 );
    for(int j = 0; j < N-1; j++,i++)
    {
      
      quad[0] = h->vertex()->index;
      quad[1] = h->edge()->index;
      quad[2] = f->index;
      quad[3] = h2->edge()->index;
      h2 = h2->next();
      h = h->next();
      // TODO build lists of four indices for each sub-quad
      subDFaces.push_back(quad);  // TODO append each list of four indices to face list
    }
    //finish first one now
    //vector<Index> quad( 4 );
    quad[0] = h0->vertex()->index;
    quad[1] = h0->edge()->index;
    quad[2] = f->index;
    quad[3] = h2->edge()->index;
    subDFaces.push_back(quad);  // TODO append each list of four indices to face list
    i++;
  }
  
  
  //std::cout << "Iend: " << i << std::endl;
  //showError("buildSubdivisionFaceList() not implemented.");
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  showError("bevelVertex() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)
  if(f->isBoundary()) //return if boundary, undefined case
    return f;
  //std::cout << "bevelFace" << std::endl;

  HalfedgeIter h0 = f->halfedge();
  HalfedgeIter h = f->halfedge();
  HalfedgeIter hb = h0;
  Vector3D normal = h->face()->normal();
  normal.normalize();
  //int numEdges = (int) f->degree();
  std::vector<VertexIter> vnew;
  std::vector<EdgeIter> enew;
  std::vector<HalfedgeIter> hnew;
  std::vector<HalfedgeIter> tnew; //twins
  FaceIter fnew = newFace();
  //create inset face connectivity
  int i = 0;
  vnew.push_back(newVertex());
  hnew.push_back(newHalfedge());
  tnew.push_back(newHalfedge());
  enew.push_back(newEdge());  
    
  hnew[i]->twin() = tnew[i];
  tnew[i]->twin() = hnew[i];
  hnew[i]->face() = fnew;
  fnew->halfedge() = hnew[i];
  hnew[i]->vertex() = vnew[i];
  vnew[i]->halfedge() = hnew[i];
  hnew[i]->edge() = enew[i];
  tnew[i]->edge() = enew[i];
  enew[i]->halfedge() = hnew[i];
  h=h->next();
  i++;
  while(h != h0){
    vnew.push_back(newVertex());
    Vector3D normal = h->face()->normal();
    vnew[i]->position = (h->vertex()->position + h->next()->vertex()->position + hb->vertex()->position)/3.0 + normal*0.5;
    hnew.push_back(newHalfedge());
    tnew.push_back(newHalfedge());
    enew.push_back(newEdge());

    hnew[i]->twin() = tnew[i];
    tnew[i]->twin() = hnew[i];
    //skip face assignment, did it once
    hnew[i]->face() = fnew;
    hnew[i]->vertex() = vnew[i];
    vnew[i]->halfedge() = hnew[i];
    hnew[i]->edge() = enew[i];
    tnew[i]->edge() = enew[i];
    enew[i]->halfedge() = hnew[i];

    tnew[i-1]->vertex() = vnew[i];
    hnew[i-1]->next() = hnew[i];
    //tnew[i]->next() = tnew[i-1];
    hb=hb->next();
    h=h->next();
    i++;
  }
  i--;
  int num = i;
  //back to beginning h=h
  //finish off connectivity
  hnew[i]->next() = hnew[0];
  tnew[i]->vertex() = vnew[0];
  vnew[0]->position = (h->vertex()->position + h->next()->vertex()->position + hb->vertex()->position)/3.0 + normal*0.5;

  //for(int a =0; a < 4 ; a++)
  //  std::cout << elementAddress(hnew[a]) << " -> " << elementAddress(hnew[a]->next()) << std::endl;

  std::vector<VertexIter> vnew2;
  std::vector<EdgeIter> enew2;
  std::vector<HalfedgeIter> hnew2;
  std::vector<HalfedgeIter> hnew3;
  std::vector<FaceIter> fnew2;
  
  //Now connect with outer face! h=h already
  i=0;
  HalfedgeIter hloop = h->next();
  //make sure to do tnew face assignments
  hnew2.push_back(newHalfedge());
  hnew3.push_back(newHalfedge());
  enew2.push_back(newEdge());
  fnew2.push_back(newFace());

  tnew[i]->next() = hnew2[i];
  tnew[i]->face() = fnew2[i];
  h->face() = fnew2[i];
  fnew2[i]->halfedge() = hnew2[i];
  //hnew2[i]->twin() = tnew2[i]; //set later
  //hnew3[i]->twin() = hnew2[i]; //set later
  hnew2[i]->vertex() = vnew[i];
  hnew3[i]->vertex() = h->next()->vertex();
  hnew2[i]->face() = fnew2[i];
  hnew3[i]->face() = fnew2[i];
  hnew2[i]->next() = h;
  hnew2[i]->edge() = enew2[i];
  enew2[i]->halfedge() = hnew2[i];

  h->next() = hnew3[i];
  hnew3[i]->next() = tnew[i];

  h = hloop; 
  i++;
  
  do{
    hloop = h->next();
  
    hnew2.push_back(newHalfedge());
    hnew3.push_back(newHalfedge());
    enew2.push_back(newEdge());
    fnew2.push_back(newFace());

    tnew[i]->next() = hnew2[i];
    tnew[i]->face() = fnew2[i];
    h->face() = fnew2[i];
    fnew2[i]->halfedge() = hnew2[i];

    hnew2[i]->vertex() = vnew[i];
    hnew3[i]->vertex() = h->next()->vertex();

    hnew2[i]->twin() = hnew3[i-1];
    hnew3[i-1]->twin() = hnew2[i];
    hnew2[i]->face() = fnew2[i];
    hnew3[i]->face() = fnew2[i];
    hnew2[i]->edge() = enew2[i];
    enew2[i]->halfedge() = hnew2[i];

    hnew3[i-1]->edge() = enew2[i];
    
    hnew2[i]->next() = h;

    h->next() = hnew3[i];
    hnew3[i]->next() = tnew[i];
    

    h = hloop; 
    i++;
  } while(h != h0);
  i--;
  hnew2[0]->twin() = hnew3[i];
  hnew3[i]->twin() = hnew2[0];
  hnew3[i]->edge() = enew2[0];

  //showError("bevelFace() not implemented.");
  //std::cout << "Now connect with outer face! h=h already" << std::endl;
  deleteFace(f);
  return fnew;
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //
  int N = newHalfedges.size();

  for( int i = 0; i < N; i++ )
   {
      int a = (i+N-1) % N;
      int b = i;
      int c = (i+1) % N;
      Vector3D pbef = originalVertexPositions[a];
      Vector3D paft = originalVertexPositions[c];
      Vector3D pi = originalVertexPositions[b]; // get the original vertex position correponding to vertex i

      //Vector3D slope = (pi + paft + pbef)/3.f - pi;
      
      Vector3D tanV = (pi-(pi + paft + pbef)/3.f);
      tanV.normalize();

      Vector3D normal = newHalfedges[a]->face()->normal() + newHalfedges[c]->face()->normal();
      
      //Quaternion rtan = Quaternion();
      //rtan.from_axis_angle(normal,M_PI/2.f);
      //Matrix3x3 R = rtan.rotationMatrix();
      //Vector3D tanV = R*slope;
      //std::cout << tanV*tangentialInset << std::endl;
      //Vector3D tanV = newHalfedges[i]->next()->vertex()->neighborhoodCentroid() - pi;
      //

      //TODO constrain it so there is no bad geometry.
      //Vector3D tanV = (pi-(pi + paft + pbef)/3.f);
      //Vector3D newPos = pi + tangentialInset*tanV - normal*normalShift;
      //Vector3D newPos = (pi + paft + pbef)/3.f +slope*tangentialInset - normal*normalShift;
      Vector3D newPos = pi + tanV*tangentialInset/10.0 + normal*normalShift/10.0;
      //newHalfedges[b]->vertex()->position = 
      newHalfedges[b]->vertex()->position = newPos;
   }
  
}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.

}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // TODO: (meshedit) 
  // Triangulate a polygonal face
  //std::cout << "in here" << std::endl;
  if(f->degree() <= 3)
    return;
  HalfedgeIter h0 = f->halfedge();
  std::vector<EdgeIter> enew;
  std::vector<HalfedgeIter> hnew;
  std::vector<HalfedgeIter> tnew;
  std::vector<FaceIter> fnew;

  HalfedgeIter h = h0;
  HalfedgeIter hloop = h->next()->next();
  int i = 0;
  enew.push_back(newEdge());
  hnew.push_back(newHalfedge());
  tnew.push_back(newHalfedge());
  fnew.push_back(newFace());
  //faces
  hnew[i]->face() = fnew[i];
  h->face() = fnew[i];
  h->next()->face() = fnew[i];
  fnew[i]->halfedge() = h;
  //twins
  hnew[i]->twin() = tnew[i];
  tnew[i]->twin() = hnew[i];
  //verices
  hnew[i]->vertex() = h->next()->next()->vertex();
  tnew[i]->vertex() = h0->vertex();
  //edges
  hnew[i]->edge() = enew[i];
  tnew[i]->edge() = enew[i];
  enew[i]->halfedge() = hnew[i];
  //next ptrs
  hnew[i]->next() = h;
  tnew[i]->next() = h->next()->next();
  h->next()->next() = hnew[i];
  
  //jump forward twice the first time
  h = hloop;
  i++;
  while(h->next()->next() != h0)
  {
    hloop = h->next();

    enew.push_back(newEdge());
    hnew.push_back(newHalfedge());
    tnew.push_back(newHalfedge());
    fnew.push_back(newFace());
    //faces
    hnew[i]->face() = fnew[i];
    fnew[i]->halfedge() = h;
    tnew[i-1]->face() = fnew[i];
    h->face() = fnew[i];
    //twins
    hnew[i]->twin() = tnew[i];
    tnew[i]->twin() = hnew[i];
    //verices
    hnew[i]->vertex() = h->next()->vertex();
    tnew[i]->vertex() = h0->vertex();
    //edges
    hnew[i]->edge() = enew[i];
    tnew[i]->edge() = enew[i];
    enew[i]->halfedge() = hnew[i];
    //next ptrs
    hnew[i]->next() = tnew[i-1];
    tnew[i]->next() = h->next();
    h->next() = hnew[i];

    h = hloop;
    i++;
  }
  //finish linking, now at h->next()->next() = h0
  i--;
  //face links
  // use old face, who needs to make a new one?

  tnew[i]->face() = f;
  f->halfedge() = tnew[i];
  tnew[i]->vertex() = h0->vertex();
  h->next()->face() = f;
  h->face() = f;
  //******
  //Took this last one out late
  //h->next()->next()->face() = f;
  //******
  //next links
  //tnew[i]->next() = h;
  h->next()->next() = tnew[i];
  
  //deleteFace(f);
  return;
  //showError("splitPolygon() not implemented.");
  
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
}

void MeshResampler::upsample(HalfedgeMesh& mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  // -> At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh.
  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  // -> Now flip any new edge that connects an old and new vertex.
  // -> Finally, copy the new vertex positions into final Vertex::position.

  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.

  // Next, compute the updated vertex positions associated with edges.

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // Finally, flip any new edge that connects an old and new vertex.

  // Copy the updated vertex positions to the subdivided mesh.

  // iterate over all edges in the mesh
for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++){
  v->isNew = false;
  int n = v->degree();
  double u;
  if(n == 3)
    u = 3.0/16.0;
  else
    u = 3.0/(8.0*n);

  v->newPosition = (1-n*u)*v->position + u*n*v->neighborhoodCentroid();

}
for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
  HalfedgeIter h = e->halfedge();
  e->newPosition = (3.0/8.0)*(h->vertex()->position + h->twin()->vertex()->position) + 
                  (1.0/8.0)*(h->next()->next()->vertex()->position + h->twin()->next()->next()->vertex()->position);
  e->isNew = false;
}


int N = mesh.nEdges();
EdgeIter e = mesh.edgesBegin();
for (int i = 0; i < N; i++) {

  // get the next edge NOW!
  EdgeIter nextEdge = e;
  nextEdge++;
  VertexIter v;
  Vector3D newPos = e->newPosition;
  v = mesh.splitEdge(e); //set edges as new or false within splitedge
  v->isNew = true;
  v->newPosition = newPos;
  e = nextEdge;
}
for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
{
  VertexIter v = e->halfedge()->vertex();
  VertexIter vt = e->halfedge()->twin()->vertex();
  if( (e->isNew && v->isNew && !vt->isNew) || (e->isNew && !v->isNew && vt->isNew))
    mesh.flipEdge(e);
}

for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++){
  v->position = v->newPosition;
}
  //showError("upsample() not implemented.");
}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  showError("downsample() not implemented.");
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  double meanLength;
  double i = 0;
  double splitThresh, collapseThresh,shouldFlip;
  double gradStep = 1.0/5.0;
  
  //std::cout << "Here" << std::endl;
  i = 0;
  meanLength = 0;
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++, i++)
    meanLength += e->length();

  meanLength = meanLength/i;
  splitThresh = meanLength * 4.0 / 3.0;
  collapseThresh = meanLength * 4.0 / 5.0;

  for(int j = 0; j < 1; j++)
  {
  //std::cout << "Here 2" << std::endl;
  //splitEdge
  int N = mesh.nEdges();
  EdgeIter e = mesh.edgesBegin();
  for (int k = 0; k < N; k++) {

  // get the next edge NOW!
  EdgeIter nextEdge = e;
  nextEdge++;
  if(e->length() > splitThresh)
    mesh.splitEdge(e); 

  e = nextEdge;
  }

  int count = 0;
  //CollapseEdge
  std::list<EdgeIter> toDelete;
  N = mesh.nFaces();
  FaceIter f = mesh.facesBegin();
  for (int k = 0; k < N; k++) {
    HalfedgeIter h = f->halfedge();
    HalfedgeIter h0 = h;
    FaceIter nextF = f;
    nextF++;
    do{
      if (h->edge()->length() < collapseThresh)
      {
        mesh.collapseEdge(e);
        break;
      }
      h = h->next();
    } while(h != h0);
    f = nextF;
  }

  //std::cout << "Here 5" << std::endl;
  //EdgeFlip
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
  {
    int a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    HalfedgeIter h = e->halfedge();
    a1 = h->vertex()->degree();
    a2 = h->twin()->vertex()->degree();
    b1 = h->next()->next()->vertex()->degree();
    b2 = h->twin()->next()->next()->vertex()->degree();

    if (abs(b1 - 6) + abs(b2 - 6) < abs(a1 - 6) + abs(a2 - 6))
      mesh.flipEdge(e);
  }
  //std::cout << "Here 6" << std::endl;

  //Smoothing step
  
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
    {
      v->newPosition = v->neighborhoodCentroid();
    }

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
    {
      Vector3D normal = v->normal();
      if(isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
        normal = Vector3D();
      //std::cout << "Normal: " << normal << std::endl;
      v->position = v->position + gradStep * dot(normal, v->newPosition - v->position) * normal;
    }
  }
  
  //showError("resample() not implemented.");
}

}  // namespace CMU462
