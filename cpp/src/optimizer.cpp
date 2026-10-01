// Chapter 5, equations (5-1)--(5-4), Algorithm 5-1. CasADi/IPOPT, no AMPL.
#include "liom.hpp"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

namespace liom {
static void validate(const Problem& problem,Result& result,const Boxes& front,const Boxes& rear) {
  const auto [lb,ub]=bounds(problem,front,rear);
  const auto& z=result.solution;
  for(size_t i=0;i<z.size();++i) {
    if(!std::isfinite(z[i])) throw std::runtime_error("Non-finite solution.");
    result.max_bound_violation=std::max({result.max_bound_violation,lb[i]-z[i],z[i]-ub[i]});
  }
  if(result.max_bound_violation>1e-5) throw std::runtime_error("Discrete bounds validation failed.");
  std::map<int,Matrix> obstacles;
  for(const auto& row:problem.obstacles) obstacles[static_cast<int>(row[0])].push_back({row[1],row[2]});
  double clearance=std::numeric_limits<double>::infinity();
  const int n=problem.n();
  // Check only the 200 configuration points, using the actual two-disc geometry.
  for(double offset:problem.offsets()) for(int i=0;i<n;++i) {
    const double x=z[i]+offset*std::cos(z[2*n+i]), y=z[n+i]+offset*std::sin(z[2*n+i]);
    for(const auto& entry:obstacles) {
      const auto& vertices=entry.second;
      bool positive=true,negative=true;
      for(size_t k=0;k<vertices.size();++k) {
        const auto& a=vertices[k]; const auto& b=vertices[(k+1)%vertices.size()];
        const double ex=b[0]-a[0],ey=b[1]-a[1],dx=x-a[0],dy=y-a[1],length2=ex*ex+ey*ey;
        if(length2==0) continue;
        const double cross=ex*dy-ey*dx;
        positive=positive && cross>=-1e-12;negative=negative && cross<=1e-12;
        const double t=std::clamp((dx*ex+dy*ey)/length2,0.,1.);
        clearance=std::min(clearance,std::hypot(dx-t*ex,dy-t*ey)-problem.radius());
      }
      if(positive || negative) throw std::runtime_error("A disc centre intersects an obstacle.");
    }
  }
  result.min_disc_clearance=clearance;
  if(clearance < -1e-5) throw std::runtime_error("Discrete covering-disc clearance is negative.");
}
Result optimize(const Problem& problem,const std::string& linear_solver,const std::string& hsl_library) {
  using namespace casadi;
  const auto& p=problem.p;
  const int n=problem.n();
  SX z=SX::sym("z",11*n+1), weight=SX::sym("weight"), tf=z(11*n), dt=tf/(n-1);
  auto at=[&](int i,int j)->SX{return z(j*n+i);};
  SX infeasibility=0, cost=tf;
  const auto offsets=problem.offsets();
  auto square=[](const SX& value)->SX{return value*value;};
  for(int i=0;i<n;++i) cost+=p[19]*square(at(i,4))+p[20]*square(at(i,6));
  for(int i=1;i<n;++i) {
    infeasibility+=square(at(i,0)-at(i-1,0)-dt*at(i-1,3)*cos(at(i-1,2)));
    infeasibility+=square(at(i,1)-at(i-1,1)-dt*at(i-1,3)*sin(at(i-1,2)));
    infeasibility+=square(at(i,3)-at(i-1,3)-dt*at(i-1,4));
    infeasibility+=square(at(i,2)-at(i-1,2)-dt*at(i-1,3)*tan(at(i-1,5))/p[5]);
    infeasibility+=square(at(i,5)-at(i-1,5)-dt*at(i-1,6));
    for(int disc=0;disc<2;++disc) {
      infeasibility+=square(at(i,7+2*disc)-at(i,0)-offsets[disc]*cos(at(i,2)));
      infeasibility+=square(at(i,8+2*disc)-at(i,1)-offsets[disc]*sin(at(i,2)));
    }
  }
  Dict options{{"print_time",false},{"ipopt.print_level",0},{"ipopt.tol",1e-7},
    {"ipopt.max_iter",9000},{"ipopt.max_cpu_time",120.},{"ipopt.mu_strategy","adaptive"},
    {"ipopt.linear_solver",linear_solver}};
  if(!hsl_library.empty()) options["ipopt.hsllib"]=hsl_library;
  Function solver=nlpsol("liom","ipopt",SXDict{{"x",z},{"p",weight},{"f",cost+weight*infeasibility}},options);
  Function metrics("metrics",{z},{cost,infeasibility});
  Corridors corridors(problem);
  Vector previous=problem.initial_vector();
  double penalty=p[17];
  Result result;
  for(int iteration=1;iteration<=20;++iteration) {
    const auto [front,rear]=corridors.build(previous);
    const auto [lb,ub]=bounds(problem,front,rear);
    const DMDict answer=solver(DMDict{{"x0",DM(previous)},{"lbx",DM(lb)},{"ubx",DM(ub)},{"p",DM(penalty)}});
    const auto status=solver.stats();
    if(!static_cast<bool>(status.at("success")))
      throw std::runtime_error("IPOPT failed: "+status.at("return_status").to_string()+". Check linear solver/HSL installation.");
    Vector current=answer.at("x").get_elements();
    const auto values=metrics(std::vector<DM>{DM(current)});
    const double nominal=static_cast<double>(values[0]), inf=static_cast<double>(values[1]);
    double difference=0;
    for(size_t i=0;i<current.size();++i) difference+=std::pow(current[i]-previous[i],2);
    difference=std::sqrt(difference);
    result.statistics.push_back({double(iteration),penalty,nominal,inf,difference});
    result.history.push_back(current);
    std::cout << "LIOM " << iteration << " | weight " << penalty << " | J " << nominal
              << " | infeasibility " << inf << " | difference " << difference << std::endl;
    if(inf<p[15] && difference<p[16]) {
      result.solution=current;
      validate(problem,result,front,rear);
      return result;
    }
    previous=std::move(current);penalty*=p[18];
  }
  throw std::runtime_error("Both stopping criteria were not met. No converged result is claimed.");
}
} // namespace liom
