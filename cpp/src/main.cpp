#include "liom.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace liom {
static Matrix read_csv(const std::filesystem::path& path) {
  std::ifstream file(path);
  if(!file) throw std::runtime_error("Cannot read "+path.u8string());
  Matrix rows;
  std::string line,cell;
  while(std::getline(file,line)) {
    if(line.empty()) continue;
    std::stringstream stream(line);Vector row;
    while(std::getline(stream,cell,',')) {
      const double value=std::stod(cell);
      if(!std::isfinite(value)) throw std::runtime_error("Non-finite CSV input.");
      row.push_back(value);
    }
    if(row.empty() || (!rows.empty() && row.size()!=rows[0].size())) throw std::runtime_error("Ragged CSV input.");
    rows.push_back(row);
  }
  if(rows.empty()) throw std::runtime_error("Empty CSV input.");
  return rows;
}
Problem Problem::load(const std::filesystem::path& directory) {
  Problem result;
  auto rows=read_csv(directory/"parameters.csv");
  for(const auto& row:rows) result.p.insert(result.p.end(),row.begin(),row.end());
  result.reference=read_csv(directory/"reference.csv");
  result.grid=read_csv(directory/"dilated_map.csv");
  result.obstacles=read_csv(directory/"obstacles.csv");
  if(result.p.size()!=28 || result.n()<2 || result.reference.size()!=size_t(result.n()) ||
      result.reference[0].size()!=11 || result.obstacles[0].size()!=3)
    throw std::runtime_error("Unexpected input dimensions; see data/README.md.");
  for(const auto& row:result.grid) for(double value:row)
    if(value!=0 && value!=1) throw std::runtime_error("The dilated map must contain only 0 and 1.");
  return result;
}
static void write_csv(const std::filesystem::path& path,const std::string& header,const Matrix& rows) {
  std::ofstream file(path);
  if(!file) throw std::runtime_error("Cannot write "+path.u8string());
  file << std::setprecision(17) << header << '\n';
  for(const auto& row:rows) {
    for(size_t j=0;j<row.size();++j) file << (j?",":"") << row[j];
    file << '\n';
  }
}
void save(const Problem& problem,const Result& result,const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  Matrix trajectory,history;
  const int n=problem.n();
  for(int i=0;i<n;++i) {
    Vector row;
    for(int j=0;j<11;++j) row.push_back(result.solution[j*n+i]);
    trajectory.push_back(row);
  }
  for(size_t k=0;k<result.history.size();++k) for(int i=0;i<n;++i)
    history.push_back({double(k+1),result.history[k][i],result.history[k][n+i],result.history[k][2*n+i]});
  write_csv(directory/"trajectory.csv","x,y,theta,v,a,phy,w,xf,yf,xr,yr",trajectory);
  write_csv(directory/"iterations.csv","iteration,weight,cost,infeasibility,difference",result.statistics);
  const auto& last=result.statistics.back();
  write_csv(directory/"summary.csv","terminal_time,cost,infeasibility,difference,iterations",
            {{result.solution.back(),last[2],last[3],last[4],double(result.history.size())}});
  write_csv(directory/"intermediate.csv","iteration,x,y,theta",history);
}
} // namespace liom

static int run_program(int argc,const std::vector<std::string>& argv) {
  try {
    std::string data="data",output="results",linear_solver="ma27",hsl;
#ifdef _WIN32
    if(const wchar_t* value=_wgetenv(L"LIOM_HSL_LIBRARY")) hsl=std::filesystem::path(value).u8string();
#else
    if(const char* value=std::getenv("LIOM_HSL_LIBRARY")) hsl=value;
#endif
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--help") {
        std::cout << "liom --data data --output results --linear-solver ma27|mumps [--hsl-library PATH]\n";
        return 0;
      }
      if(i+1>=argc) throw std::runtime_error("Missing value for "+arg);
      const std::string value=argv[++i];
      if(arg=="--data") data=value;
      else if(arg=="--output") output=value;
      else if(arg=="--linear-solver") linear_solver=value;
      else if(arg=="--hsl-library") hsl=value;
      else throw std::runtime_error("Unknown option: "+arg);
    }
    if(linear_solver!="ma27" && linear_solver!="mumps") throw std::runtime_error("Choose ma27 or mumps explicitly.");
    if(!hsl.empty()) {
      const auto library=std::filesystem::absolute(std::filesystem::u8path(hsl));
      if(!std::filesystem::exists(library)) throw std::runtime_error("HSL library does not exist.");
#ifdef _WIN32
      const wchar_t* old=_wgetenv(L"PATH");
      const std::wstring path=std::wstring(old?old:L"")+L";"+library.parent_path().wstring();
      _wputenv_s(L"PATH",path.c_str());
      hsl=library.filename().u8string();
#else
      hsl=library.u8string();
#endif
    }
    std::cout << std::setprecision(10);
    const auto problem=liom::Problem::load(std::filesystem::u8path(data));
    const auto result=liom::optimize(problem,linear_solver,hsl);
    liom::save(problem,result,std::filesystem::u8path(output));
    std::cout << "Converged. J = " << result.statistics.back()[2] << ", T = " << result.solution.back()
              << " s, bound violation = " << result.max_bound_violation
              << ", minimum disc clearance = " << result.min_disc_clearance << " m\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';return 1;
  }
}

// Keep command-line paths Unicode on Windows; the numerical core is unchanged.
#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {
  std::vector<std::string> arguments;
  for(int i=0;i<argc;++i) arguments.push_back(std::filesystem::path(argv[i]).u8string());
  return run_program(argc,arguments);
}
#else
int main(int argc,char** argv) {
  return run_program(argc,std::vector<std::string>(argv,argv+argc));
}
#endif

