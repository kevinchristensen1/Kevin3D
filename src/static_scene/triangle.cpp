#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle
  BBox box(mesh->positions[v1]);
  box.expand(mesh->positions[v2]);
  box.expand(mesh->positions[v3]);
  return box;
}

bool Triangle::intersect(const Ray& r) const {
  // TODO (PathTracer): implement ray-triangle intersection
  double eps = EPS_D;
  Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
  Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
  Vector3D s = r.o - mesh->positions[v1];
  Vector3D rd = r.d;
  Vector3D e1crossd = cross(e1,rd);
  double denom = dot(e1crossd,e2);
  
  if (abs(denom) <= eps) //almost zero area/volume
    return false;

  double mult = 1.0/denom;
  
  Vector3D negscrosse2 = -1. * cross(s,e2);
  double u = mult * dot(negscrosse2,rd);
  double v = mult * dot(e1crossd,s);
  double t = mult * dot(negscrosse2,e1);
  //cout << "t: " << t << "u: " << u << "v: " << v << endl;
  if ( u < eps || v < eps || u + v >= 1. - eps)
    return false;
  else if( t < r.min_t || t > r.max_t)
    return false;

  //r.max_t = (size_t) (t+1);
  //Vector3D normal = (1. - u - v) * mesh->normals[v1] + u * mesh->normals[v2] + v * mesh->normals[v3];

  return true;
}

bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  double eps = EPS_D;
  Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
  Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
  Vector3D s = r.o - mesh->positions[v1];
  Vector3D rd = r.d;
  Vector3D e1crossd = cross(e1,rd);
  double denom = dot(e1crossd,e2);
  
  if (abs(denom) <= eps) //almost zero area/volume
    return false;

  double mult = 1.0/denom;
  
  Vector3D negscrosse2 = -1. * cross(s,e2);
  double u = mult * dot(negscrosse2,rd);
  double v = mult * dot(e1crossd,s);
  double t = mult * dot(negscrosse2,e1);
  //cout << "t: " << t << "u: " << u << "v: " << v << endl;
  if ( u < eps || v < eps || u + v >= 1. - eps)
    return false;
  else if( t < r.min_t || t > r.max_t)
    return false;

  if (t < r.max_t)
    r.max_t = t;
  Vector3D normal = (1. - u - v) * mesh->normals[v1] + u * mesh->normals[v2] + v * mesh->normals[v3];
  normal.normalize();

  if (dot(-rd,normal) < 0)
    normal = -1. * normal;

  
  // fill in info for intersection
  isect->t = t;
  isect->bsdf = get_bsdf();
  isect->primitive = this;
  isect->n = normal;

  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

}  // namespace StaticScene
}  // namespace CMU462
