#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a,b,c,temp;
  Vector3D e = o - r.o;
  Vector3D d = r.d;
  a = dot(d,d);
  b = -2.*dot(e,d);
  c = dot(e,e) - r2;
  double sqr = b*b - 4.*a*c;
  if (sqr < 0.)
    return false;
  t1 = (-b - sqrt(sqr))/(2.*a);
  t2 = (-b + sqrt(sqr))/(2.*a);

  if ( t1 > t2 )
  {
    temp = t1;
    t1 = t2;
    t2 = temp;
  }

  if ( t1 >= r.min_t && t1 <= r.max_t )
    return true;
  else if( t2 >= r.min_t && t2 <= r.max_t )
  {
    t1 = t2;
    return true;
  }
  else
    return false;

}

bool Sphere::intersect(const Ray& r) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1,t2;
  bool doesIntersect = test(r,t1,t2);

  if(!doesIntersect)
    return false;

  //r.max_t = (size_t) (t1+1);

  return true;
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1,t2;
  bool doesIntersect = test(r,t1,t2);

  if(!doesIntersect)
    return false;
  Vector3D n = normal(r.o + t1 * r.d);
  if (t1 < r.max_t)
    r.max_t = t1;
  isect->t = t1;
  isect->bsdf = get_bsdf();
  isect->primitive = this;
  isect->n = n;

  return true;
}

void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

void Sphere::drawOutline(const Color& c) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

}  // namespace StaticScene
}  // namespace CMU462
