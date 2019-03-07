#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement DiffuseBSDF
  *wi = sampler.get_sample(pdf); //cosine weighted sampler
  return f(wo,*wi);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return reflectance*(1./abs(wo.z));
  //just pick direction of reflection
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement MirrorBSDF
  
  reflect(wo,wi);
  *pdf = 1.f; //1 along direction of ray, 0 everywhere else.
  return f(wo,*wi);
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                  float* pdf) {
  // TODO (PathTracer):
  // Implement RefractionBSDF
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  if (abs(wi.z - wo.z) <= EPS_D )
    return reflectance*(1./abs(wi.z));
  else
    return transmittance*(1./abs(wi.z));
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Compute Fresnel coefficient and either reflect or refract based on it.

  double inIOR, outIOR;
  double dotP = wo.z;
  double sign;
  inIOR = dotP > 0. ? 1. : ior;
  outIOR = dotP > 0. ? ior : 1.;
  
  /*
  if(wo.z > 0){
    sign = 1;
    inIOR = ior;
    outIOR = 1.;
  }
  else if(wo.z < 0.) {
    sign = -1;
    inIOR = 1.;
    outIOR = ior;
  }
  */
  double eta = (inIOR/outIOR);
  bool refr = refract(wo,wi, eta);
  
  double sinIn = (eta) * sqrt(max(0. , 1. - pow(dotP,2.)));
  double costhetaIn = max(0.,sqrt(1. - pow(sinIn,2.)));
  double costhetaOut = abs(dotP);
  double rpar = (outIOR*costhetaIn - inIOR*costhetaOut) / (outIOR*costhetaIn + inIOR*costhetaOut);
  double rperp = (inIOR*costhetaIn - outIOR*costhetaOut) / (inIOR*costhetaIn + outIOR*costhetaOut);
  
  double Fr = 0.5*(rpar*rpar + rperp*rperp);
  //double R0 = pow((inIOR-outIOR)/(inIOR+outIOR),2.);
  //double Fr = R0 + (1.-R0)*pow(1.-abs(wo.z),5.);
  //reflect(wo,wi);
  //*wi = -wo;
  //*pdf = 1.f;
  //return f(wo,*wi);
  //*pdf = (float) (1. - Fr)*pow((outIOR/inIOR),2.)*abs(1./costhetaIn);

  if(!refr){ //TIR
    //return Spectrum(0.,0.,0.);
    reflect(wo,wi);
    *pdf = 1.f;
    return reflectance*(1./abs(wi->z));
    
  }
  //return f(wo,*wi);

    

  
  
  //double Fr = (outIOR - inIOR) / (inIOR + outIOR);
  //  *pdf = 1.f;
  //std::cout << Fr << std::endl;
  if( (double)(std::rand()) / RAND_MAX < Fr )
  {
    reflect(wo,wi);
    *pdf = (float) Fr;
    return Fr*reflectance*(1./abs(wi->z));
  }
  else
  {
    *pdf = (float) (1. - Fr);//*pow((outIOR/inIOR),2.)*abs(1./costhetaIn);
    return (1.-Fr)*transmittance*(1./abs(wi->z)) * eta*eta; 
  }
    

  //std::cout << Fr << std::endl;
  
  

}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO (PathTracer):
  // Implement reflection of wo about normal (0,0,1) and store result in wi.

  // -wo + 2*dot(wo,n) = -wo + 2*(0,0,wo.z)
  *wi = Vector3D(-wo.x, -wo.y, wo.z);
  return;
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // TODO (PathTracer):
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  double sinIn2 = ior*ior * max(0.,(1. - pow(wo.z,2.)));
  double sign = wo.z > 0. ? -1. : 1.;
  if(sinIn2 >= 1.)
    return false;
    // sinthetaOut = sqrt(1. - pow(dotP,2)
  
  //double angleIn = asin(sinIn);
  // z component is cosine of incoming angle.
  //*wi = Vector3D(wo.x,wo.y, sqrt(1. - pow(sinIn,2.)));
  //double eta = (outIOR/inIOR);//(inIOR / outIOR);
  //*wi = Vector3D(-wo.x*ior,-wo.y*ior, sqrt(1. - sinIn2) );
  //*wi = ior * (-wo) + Vector3D(0.,0., sqrt(1. - sinIn2)*(1.-ior) );
  double cosThetaIn = max(0.,sqrt(1. - sinIn2));
  *wi =  Vector3D(-wo.x*ior,-wo.y*ior, sign*cosThetaIn  );
  //*wi = Vector3D(-wo.x*ior,-wo.y*ior, sqrt(1. - sinIn2) );
  (*wi).normalize();
  return true;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return Spectrum();
}

}  // namespace CMU462
