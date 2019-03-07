#include "environment_light.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>

namespace CMU462 {
namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
  // TODO: (PathTracer) initialize things here as needed
  //std::cout << "begin" << std::endl;
  int w = envMap->w;
  int h = envMap->h;

  pdfMap.resize(h*w);
  rowMap.resize(h);
  pdfPixel.resize(h*w);
  double totalP = 0.;
  for (size_t y = 0; y < h; ++y)
  {
    double sinTheta = sin(M_PI * (((double)y) + 0.5) / (double)h);
    double sumRow = 0.;
    double lastPixel = 0.;
    for (size_t x = 0; x < w; ++x)
    {
      double pixelP = sinTheta * envMap->data[x + y * w].illum();
      pdfMap[x + y * w] = pixelP +lastPixel ;
      lastPixel += pixelP;
      pdfPixel[x + y * w] = pixelP;
      sumRow += pixelP;
    }
    rowMap[y] += sumRow;
    totalP += sumRow;
  }
  
  //double pdfDivide = (M_PI/h) * (2.*M_PI/w)* totalP; //2pi^2
  double prevRow = 0;
  for (size_t y = 0; y < h; ++y)
  {
    //double sinTheta = sin(M_PI * (((double)y) + 0.5) / (double)h) * pdfDivide;
    for (size_t x = 0; x < w; ++x)
    {
      pdfMap[x + y * w] /= rowMap[y];
      pdfPixel[x + y * w] /= (totalP);
    }
      
    
    rowMap[y] += prevRow;
    prevRow = rowMap[y];
    rowMap[y] /= totalP;
    //std::cout << rowMap[y] << std::endl;
  }

  //double checkSum = 0.;
  //for (size_t y = 0; y < h; ++y)
  //{
    //for (size_t x = 0; x < w; ++x)
      //checkSum += pdfPixel[x + y * w];
    //std::cout << rowMap[y] << std::endl;
  //}

  //std::cout << checkSum << std::endl;


}

EnvironmentLight::~EnvironmentLight(){
  //delete envMap;
  //envMap = NULL;
}

Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight, float* pdf) const {
  // TODO: (PathTracer) Implement
  
  *distToLight = INF_D;
  double w = envMap->w;
  double h = envMap->h;
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  std::vector<double>::const_iterator thetaIt, phiIt;
  thetaIt = std::lower_bound(rowMap.begin(),rowMap.end(), Xi1);
  double row = std::distance( rowMap.begin(), thetaIt ) ;
  //std::cout << row << std::endl;
  
  std::vector<double>::const_iterator pdfBegin = pdfMap.begin();
  std::vector<double>::const_iterator pdfEnd = pdfMap.begin();
  std::advance( pdfBegin, (int) (row * w ));
  std::advance( pdfEnd, (int) (row * w + w-1 ));
  phiIt = std::lower_bound( pdfBegin , pdfEnd, Xi2); //pdfMap.begin()+ (row + w) ,Xi2);

  double column = std::distance( pdfBegin , phiIt ) ;

  double theta = ( (row+0.5) / h ) * M_PI;
  double phi = ( (column+0.5) / w ) * 2.*M_PI;

  Vector3D dir(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
  Vector3D o(0.,0.,0.);
  Ray r(o,dir);
  *pdf = (float) pdfPixel [ ((int) column + (int) (row * w) )];
  *pdf /= (M_PI/h) * (2.*M_PI/w)*sin(theta);
  return sample_dir(r);
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: (PathTracer) Implement
  Vector3D d = r.d;
  double phi = atan2(d.z , d.x) + M_PI;
  double theta = acos(d.y);

  theta = clamp(theta, 0., M_PI);
  phi = clamp(phi, 0., 2. * M_PI);

  double w = envMap->w;
  double h = envMap->h;
  double x = phi * (w-1.) / (2.*M_PI) ;
  double y = theta * ((h-1.) / M_PI);
  x = clamp(x, 0., w-2 );
  y = clamp(y,0.,h-2);

  double x1 = floor(x);
  double y1 = floor(y);

  double x2 = ceil(x);
  double y2 = ceil(y);

  
  int ws = w;
  int hs = h;
  double x2x = (x2 - x);
  double y2y = (y2 - y);
  double xx1 = (x - x1);
  double yy1 = (y - y1);
  double denom = 1. / ((x2 - x1)*(y2-y1));

  if(x2 >= w)
    x2 = 0.;
    
  if(y2 >= h)
    y2 = 0;

  int x1s = (int) x1;
  int x2s = (int) x2;
  int y1s = (int) y1;
  int y2s = (int) y2;
  
  
  Spectrum s = denom * ( x2x * y2y * envMap->data[x1s + y1s * w] + 
                         xx1 * y2y * envMap->data[x2s + y1s * w] + 
                         x2x * yy1 * envMap->data[x1s + y2s * w] + 
                         xx1 * yy1 * envMap->data[x2s + y2s * w] );
  
  
  s.b = clamp(s.b,0.,1.);
  s.r = clamp(s.r,0.,1.);
  s.g = clamp(s.g,0.,1.);
  

  return s;
}

}  // namespace StaticScene
}  // namespace CMU462
