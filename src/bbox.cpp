#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
  // TODO (PathTracer):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  double txmin, txmax, tymin, tymax, tzmin, tzmax, temp, t0t, t1t;
  Vector3D o = r.o;
  Vector3D d = r.d;
  
  txmin = (this->min[0] - o[0])/d[0];
  txmax = (this->max[0] - o[0])/d[0];
  if(txmin > txmax)
  {
    temp = txmin;
    txmin = txmax;
    txmax = temp;
  }
    
  tymin = (this->min[1] - o[1])/d[1];
  tymax = (this->max[1] - o[1])/d[1];
  if(tymin > tymax)
  {
    temp = tymin;
    tymin = tymax;
    tymax = temp;
  }
  //if (txmin > tymax || txmax < tymin)
  //  return false;
  

  tzmin = (this->min[2] - o[2])/d[2];
  tzmax = (this->max[2] - o[2])/d[2];
  if(tzmin > tzmax)
  {
    temp = tzmin;
    tzmin = tzmax;
    tzmax = temp;
  }

  //if (txmin > tzmax || txmax < tzmin)
  //  return false;

  t0t = std::max(std::max(txmin,tymin), tzmin);
  t1t = std::min(std::min(txmax,tymax), tzmax);
  
  if (t0t > t1t)
    return false;
  
  //if ( (t0t < r.min_t && t1t < r.min_t) || t0t > r.max_t)
  if ( t1t < r.min_t || t0t > r.max_t)
    return false;

  if (t0t > t0)
    t0 = t0t;
  if (t1t < t1)
    t1 = t1t;

  //std::cout << "TRUE?" << std::endl;
  return true;
}

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();
}

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
