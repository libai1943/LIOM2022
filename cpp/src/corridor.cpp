// Chapter 5 safe travel corridors: the original up/left/down/right expansion.
#include "liom.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace liom {
double Problem::radius() const { return std::hypot((p[5]+p[6]+p[7])/4, p[8]/2); }
std::array<double,2> Problem::offsets() const {
  const double length = p[5]+p[6]+p[7];
  return {0.75*length-p[7], 0.25*length-p[7]};
}
Vector Problem::initial_vector() const {
  Vector z(n()*11+1);
  for (int j=0;j<11;++j) for (int i=0;i<n();++i) z[j*n()+i]=reference[i][j];
  z.back()=p[27];
  return z;
}
Corridors::Corridors(const Problem& problem): problem_(problem),
  dx_((problem.p[2]-problem.p[1])/problem.grid.size()),
  dy_((problem.p[4]-problem.p[3])/problem.grid[0].size()) {}
bool Corridors::point_free(double x, double y) const {
  const int ix=static_cast<int>(std::floor((x-problem_.p[1])/dx_));
  const int iy=static_cast<int>(std::floor((y-problem_.p[3])/dy_));
  return ix>=0 && iy>=0 && ix<static_cast<int>(problem_.grid.size()) &&
    iy<static_cast<int>(problem_.grid[0].size()) && problem_.grid[ix][iy]==0;
}
bool Corridors::strip_free(const Box& b) const {
  const auto& p=problem_.p;
  const double r=problem_.radius(), ds=dx_/2;
  if(b[0]<p[1]+r || b[1]>p[2]-r || b[2]<p[3]+r || b[3]>p[4]-r) return false;
  const int nx=static_cast<int>(std::ceil((b[1]-b[0])/ds))+1;
  const int ny=static_cast<int>(std::ceil((b[3]-b[2])/ds))+1;
  for(int i=0;i<nx;++i) for(int j=0;j<ny;++j) {
    const double x=nx==1? b[1] : (i==nx-1? b[1] : b[0]+(b[1]-b[0])*i/(nx-1));
    const double y=ny==1? b[3] : (j==ny-1? b[3] : b[2]+(b[3]-b[2])*j/(ny-1));
    if(!point_free(x,y)) return false;
  }
  return true;
}
Box Corridors::box(double x, double y) const {
  if(!point_free(x,y)) {
    bool found=false;
    for(int i=1;i<100000;++i) {
      const int direction=i%8;
      const double r=1+(i-direction)/8, angle=direction*std::acos(-1.)/4;
      const double xx=x+std::cos(angle)*r*dx_/2, yy=y+std::sin(angle)*r*dx_/2;
      if(point_free(xx,yy)) { x=xx; y=yy; found=true; break; }
    }
    if(!found) throw std::runtime_error("No free corridor seed was found.");
  }
  std::array<double,4> lengths{};
  std::array<bool,4> done{};
  while(!std::all_of(done.begin(),done.end(),[](bool d){return d;})) {
    for(int direction=0;direction<4;++direction) {
      if(done[direction]) continue;
      auto trial=lengths;
      trial[direction]+=problem_.p[13];
      if(trial[direction]>problem_.p[14]) { done[direction]=true; continue; }
      const auto [u,l,d,r]=lengths;
      const std::array<Box,4> strips{{
        {x-l,x+r,y+u,y+trial[0]}, {x-trial[1],x-l,y-d,y+u},
        {x-l,x+r,y-trial[2],y-d}, {x+r,x+trial[3],y-d,y+u}}};
      if(strip_free(strips[direction])) lengths=trial;
      else done[direction]=true;
    }
  }
  return {x-lengths[1],x+lengths[3],y-lengths[2],y+lengths[0]};
}
std::pair<Boxes,Boxes> Corridors::build(const Vector& z) const {
  std::pair<Boxes,Boxes> result;
  const auto offsets=problem_.offsets();
  for(int disc=0;disc<2;++disc) {
    auto& boxes=disc==0? result.first : result.second;
    for(int i=0;i<problem_.n();++i) {
      const double theta=z[2*problem_.n()+i];
      boxes.push_back(box(z[i]+offsets[disc]*std::cos(theta),
                          z[problem_.n()+i]+offsets[disc]*std::sin(theta)));
    }
  }
  return result;
}
std::pair<Vector,Vector> bounds(const Problem& problem,const Boxes& front,const Boxes& rear) {
  const auto& p=problem.p;
  const int n=problem.n();
  const double inf=std::numeric_limits<double>::infinity();
  Vector low(11*n+1,-inf), high(11*n+1,inf);
  const auto fix=[&](int i,int j,double v){low[j*n+i]=high[j*n+i]=v;};
  for(int j=3;j<=6;++j) for(int i=0;i<n;++i) {
    low[j*n+i]=-p[j+6]; high[j*n+i]=p[j+6];
  }
  for(int j=0;j<3;++j) {fix(0,j,p[21+j]);fix(n-1,j,p[24+j]);}
  for(int j:{3,5}) {fix(0,j,0);fix(n-1,j,0);}
  for(int j:{4,6}) fix(n-1,j,0);
  for(int disc=0;disc<2;++disc) {
    const auto& boxes=disc==0? front : rear;
    const int j=7+2*disc;
    for(int i=0;i<n;++i) {
      low[j*n+i]=boxes[i][0];high[j*n+i]=boxes[i][1];
      low[(j+1)*n+i]=boxes[i][2];high[(j+1)*n+i]=boxes[i][3];
    }
    for(int endpoint=0;endpoint<2;++endpoint) {
      const int i=endpoint? n-1 : 0, base=endpoint? 24 : 21;
      const double offset=problem.offsets()[disc];
      const std::array<double,2> xy{p[base]+offset*std::cos(p[base+2]),p[base+1]+offset*std::sin(p[base+2])};
      for(int axis=0;axis<2;++axis) {
        if(xy[axis]<low[(j+axis)*n+i]-1e-9 || xy[axis]>high[(j+axis)*n+i]+1e-9)
          throw std::runtime_error("An endpoint disc lies outside its corridor.");
        fix(i,j+axis,xy[axis]);
      }
    }
  }
  low.back()=.1;high.back()=n*.5;
  return {low,high};
}
} // namespace liom
