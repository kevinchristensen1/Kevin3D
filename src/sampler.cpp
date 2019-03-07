#include "sampler.h"
#include <iostream>
namespace CMU462 {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {
  // TODO (PathTracer):
  // Implement uniform 2D grid sampler
  double x = (double)(std::rand()) / RAND_MAX;
  double y = (double)(std::rand()) / RAND_MAX;
  
  return Vector2D(x, y);
}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {
  // You may implement this, but don't have to.
  /*
  double Xi_1 = (double)(std::rand()) / RAND_MAX;
  //double Xi_1 = sqrt( r2 ); //sqrt( xi_1 )
  double r = sqrt( 1. - Xi_1*Xi_1 );
  double Xi_2 = (double)(std::rand()) / RAND_MAX;         // 2pi xi_2
  double theta = 2.*PI*Xi_2;
  // *pdf = (float) 1./(2.*PI);
  *pdf = std::max( 0.f, (float) ( cos(theta)/PI ) );

  Vector3D wi( r*cos(theta), r*sin(theta), Xi_1);
  wi.normalize();
  return wi;
  */
  
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);
  //*pdf = std::abs( 0.f, (float) ( cos(theta) /PI) );
  *pdf =  (float) abs( cos(theta) / PI) ;

  Vector3D wi( xs, ys, zs);
  return wi;
  
}

}  // namespace CMU462
