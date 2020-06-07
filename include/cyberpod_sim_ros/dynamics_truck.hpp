#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "cyberpod_sim_ros/common.hpp"
#include <Eigen/Dense>
#include <math.h>

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

void dynamics(const double t,
              const double X[STATE_LENGTH],
              const double U[INPUT_LENGTH],
                    double xDot[STATE_LENGTH])
{
	double g[STATE_LENGTH*INPUT_LENGTH];
  double f[STATE_LENGTH];
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
  for(int i=0; i<STATE_LENGTH; i++)
  {
    xDot[i] = f[i];
    for(int j=0; j<INPUT_LENGTH; j++)
    {
      xDot[i]+=g[i+j*STATE_LENGTH]*U[j];
    }
  }
}

#endif
