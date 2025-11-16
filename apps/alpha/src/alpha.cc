/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <cmath>
#include <array>
#include <core/io/c_stdio_file.h>
#include <core/proc/levmar.h>
#include <core/proc/levmar3.h>
#include <core/debug.h>

namespace {

constexpr double WB = 5e3;
constexpr double WC = 3;
constexpr double WY = 100;


class c_levmar_solver_callback :
    public c_levmard_solver::callback
{

  const std::vector<double> & vx;
  const std::vector<double> & vy;

public:
  c_levmar_solver_callback(const std::vector<double> & _vx, const std::vector<double> & _vy) :
    vx(_vx), vy(_vy)
  {
  }


  bool compute(const std::vector<double> & params, std::vector<double> & rhs, cv::Mat1d * J, bool * have_analytical_jac) final
  {
    const double B = params[0];
    const double C = params[1];
    const size_t n = vx.size();

    rhs.resize(n);

    double * jb = nullptr;
    double * jc = nullptr;

    if ( J ) {
      J->create(2, n);
      jb = (*J)[0];
      jc = (*J)[1];
      * have_analytical_jac = true;
    }

    for (size_t i = 0; i < n; ++i) {

      const double x = vx[i];
      const double y = vy[i];
      const double den = 1 / (1 + C * x);
      const double w = WB / ((1 + WC * x) * (y + WY));
      const double w2 = w;// * w;

      rhs[i] = (B * den - y) * w2;

      if (J) {
        jb[i] = den * w2;
        jc[i] = -B * den * den * x * w2;
      }
    }

    return true;
  }

};


class c_levmar3_solver_callback :
    public c_levmar3_solver::callback
{
  const std::vector<double> & vx;
  const std::vector<double> & vy;

public:

  c_levmar3_solver_callback(const std::vector<double> & _vx, const std::vector<double> & _vy) :
    vx(_vx), vy(_vy)
  {
  }

  double compute(const std::vector<double> & params, cv::Mat1d * H, cv::Mat1d * v) const final
  {
    const double B = params[0];
    const double C = params[1];
    const size_t n = vx.size();

    double rhs = 0;
    double h00 = 0;
    double h01 = 0;
    double h10 = 0;
    double h11 = 0;
    double v0 = 0;
    double v1 = 0;

    for (size_t i = 0; i < n; ++i) {

      const double x = vx[i];
      const double y = vy[i];
      const double den = 1 / (1 + C * x);
      const double w = WB / ((1 + WC * x) * (y + WY));
      const double w2 = w;// * w;
      const double dy = (B * den - y) * w2;
      rhs += dy * dy;

      if (H) {

        const double jb = den * w2;
        const double jc = -B * den * den * x * w2;

        v0 += jb * dy;
        v1 += jc * dy;

        //H[i][j] = _J.row(i).dot(_J.row(j));
       h00 += jb * jb;
       h01 += jb * jc;
       h10 += jc * jb;
       h11 += jc * jc;
      }
    }


    if ( H ) {
      H->create(2, 2);
      (*H)(0,0) = h00;
      (*H)(0,1) = h01;
      (*H)(1,0) = h10;
      (*H)(1,1) = h11;

      v->create(2, 1);
      (*v)(0, 0) = v0;
      (*v)(1, 0) = v1;
    }


    return (rhs);
  }
};

}

int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  std::string input_file_name;
  std::string output_file_name;
  std::vector<double> vx, vy;
  std::vector<double> params;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ||strcmp(argv[i], "-h") == 0 ) {
      fprintf(stdout, "Usage:\n"
          "  ./alpha input_file_name.txt [-o output_file_name.txt]\n");
      return 0;
    }


    if ( strcmp(argv[i], "-o") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Invalid arg: output file name is expected after '%s' option\n", argv[i-1]);
        return 1;
      }

      output_file_name = argv[i];
      continue;
    }


    if (!input_file_name.empty()) {
      fprintf(stderr, "Invalid arg: input_file_name specified twice, option '%s' is unexpected\n", argv[i - 1]);
      return 1;
    }

    input_file_name = argv[i];
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input_file_name specified, try --help for arguments\n");
    return 1;
  }


  if ( true ) {

    c_stdio_file fp;

    if ( !fp.open(input_file_name, "rt") ) {
      fprintf(stderr, "Can not read input file '%s': %s\n", input_file_name.c_str(), strerror(errno));
      return 1;
    }


    double x, y;
    int nlines = 0;


    while ( !feof(fp) ) {
      ++nlines;

      if (fscanf(fp, "%lf %lf", &x, &y) != 2) {
        CF_DEBUG("Can not parse line %d, skipped", nlines);
        while (!feof(fp) && fgetc(fp) != '\n') {
        }
        continue;
      }

      vx.emplace_back(x);
      vy.emplace_back(y);
    }

    CF_DEBUG("%zu pairs read", vx.size());
  }


  params.emplace_back(6000);
  params.emplace_back(0.5);

  c_levmar3_solver_callback cb(vx, vy);
  c_levmar3_solver lm;

  lm.set_epsx(1);

  const int iterations = lm.run(cb, params);
  const double rmse = lm.rhs() / (vx.size() - 2U);

  CF_DEBUG("lm.run(): %d iterations, rmse=%g B=%g C=%g", iterations, rmse, params[0], params[1]);

  if ( !output_file_name.empty() ) {

    c_stdio_file fp;

    if ( !fp.open(output_file_name, "wt") ) {
      fprintf(stderr, "Can not write output file '%s': %s\n", output_file_name.c_str(), strerror(errno));
      return 1;
    }

    fprintf(fp, "X\tY\tYP\tdY\tw\n" );

    const double B = params[0];
    const double C = params[1];
    const size_t n = vx.size();

    for ( size_t i = 0; i < n; ++i ) {

      const double x = vx[i];
      const double y = vy[i];
      const double yp = B / (1 + C * x);
      const double dy = y - yp;
      const double w = WB / ((1 + WC * x) * (y + WY));
      const double w2 = w;// * w;

      fprintf(fp, "%6g\t%9g\t%9g\t%9g\t%g\n",
          x, y, yp, dy, w2);
    }

  }


  return 0;
}
