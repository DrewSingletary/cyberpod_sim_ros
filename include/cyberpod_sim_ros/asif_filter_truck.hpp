#ifndef ASIF_FILTER_H
#define ASIF_FILTER_H

#include "asif++.h"
#include <Eigen/Dense>
#include <algorithm>

static const uint32_t nx = 5;
static const uint32_t nu = 2;
static const uint32_t npSS = 1;

static const double lb[nu] = {-20.0,-20.0};
static const double ub[nu] = {20.0,20.0};

static const double model[15] = {44.798,            //mb
                                 2.485,             //mw
                                 0.055936595310797, //Jw
                                 -0.02322718759275, //a2
                                 0.166845864363019, //c2
                                 3.604960049044268, //A2
                                 3.836289730154863, //B2
                                 1.069672194414735, //C2
                                 1.261650363363571, //K
                                 0.195,             //r
                                 0.5,               //L
                                 9.81,              //gGravity
                                 0.,                //FricCoeff 3.185188257847262
                                 1.0e-3,            //velEps
                                 1.225479467549329  //FricCoeff 1.225479467549329
                                 };

ASIF::ASIF *asif;

void safetySet(const double x[nx], double h[npSS], double Dh[npSS*nx])
{
  h[0] = 2*ub[0]*(1-x[0])-x[2]*x[2];
  //
  Dh[0] = -2*ub[0];
  Dh[1] = 0;//2*ub[0]*x[1]/(sqrt(x[0]*x[0]+x[1]*x[1])+0.001);
  Dh[2] = -2*x[2];
  Dh[3] = 0.;
  Dh[4] = 0.;
}

void dynamics(const double X[nx], double f[nx], double g[nu*nx])
{
  f[0] = X[2]*cos(X[3]);
  f[1] = X[2]*sin(X[3]);
  f[2] = 0;
  f[3] = X[4];
  f[4] = 0;

  g[0] = 0;
  g[1] = 0;
  g[2] = 1/(model[0]*model[9]);
  g[3] = 0;
  g[4] = -model[10]/(2*model[7]*model[9]);

  g[5] = 0;
  g[6] = 0;
  g[7] = 1/(model[0]*model[9]);
  g[8] = 0;
  g[9] = model[10]/(2*model[7]*model[9]);
}



#endif
