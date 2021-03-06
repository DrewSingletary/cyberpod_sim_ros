#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <math.h>
#include <vector>
#include <stdint.h>
#include <cstring>

// static const uint32_t STATE_LENGTH = 7;
// static const uint32_t INPUT_LENGTH = 2;
static const uint32_t MEASUREMENT_LENGTH = 4;

// static const double model[15] = {44.798,            //mb
//                                  2.485,             //mw
//                                  0.055936595310797, //Jw
//                                  -0.02322718759275, //a2
//                                  0.166845864363019, //c2
//                                  3.604960049044268, //A2
//                                  2*3.836289730154863, //B2
//                                  1.069672194414735, //C2
//                                  1.5, //K
//                                  0.195,             //r
//                                  0.5,               //L
//                                  9.81,              //gGravity
//                                  1.1,                //FricCoeff 3.185188257847262
//                                  1.0e-3,            //velEps
//                                  1.225479467549329  //FricCoeff 1.225479467549329
//                                  };

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

inline void dynamics(const double t,
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
      xDot[i]=f[i];
      for(int j=0; j<INPUT_LENGTH; j++)
      {
         xDot[i]+=g[i+j*STATE_LENGTH]*U[j];
      }
   }
}

inline void dynamicsGradientsRaw(const double x[STATE_LENGTH], double Df[STATE_LENGTH*STATE_LENGTH], double Dg[STATE_LENGTH*INPUT_LENGTH*STATE_LENGTH])
{
  double t2;
  double t3;
  (void)model;

  /* JFFUN */
  /*     JF = JFFUN(IN1,IN2) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.3. */
  /*     21-Mar-2020 17:35:04 */
  t2 = cos(x[3]);
  t3 = sin(x[3]);
  memset(&Df[0], 0, 10U * sizeof(double));
  Df[10] = t2;
  Df[11] = t3;
  Df[12] = 0.0;
  Df[13] = 0.0;
  Df[14] = 0.0;
  Df[15] = -t3 * x[2];
  Df[16] = t2 * x[2];
  Df[17] = 0.0;
  Df[18] = 0.0;
  Df[19] = 0.0;
  Df[20] = 0.0;
  Df[21] = 0.0;
  Df[22] = 0.0;
  Df[23] = 1.0;
  Df[24] = 0.0;
  memset(&Dg[0], 0, 50U * sizeof(double));
}

inline void dynamicsGradient(const double t,
                      const double X[STATE_LENGTH],
                      const double U[INPUT_LENGTH],
                      double Df[STATE_LENGTH*STATE_LENGTH])
{

   double Dg[STATE_LENGTH*INPUT_LENGTH*STATE_LENGTH];
   dynamicsGradientsRaw(X, Df, Dg);
   for(uint32_t i=0; i<STATE_LENGTH; i++)
   {
      for(uint32_t j=0; j<STATE_LENGTH; j++)
      {
         double tmp = Df[i+j*STATE_LENGTH];
         for(uint32_t k=0; k<INPUT_LENGTH; k++)
         {
            tmp+=Dg[i+k*(STATE_LENGTH)+j*STATE_LENGTH*INPUT_LENGTH]*U[k];
         }
         Df[i+j*STATE_LENGTH] = tmp;
      }
   }
}

inline void measurementH(const double X[STATE_LENGTH],
                  double h[MEASUREMENT_LENGTH])
{
   h[0] = X[2]; //velocity
   h[1] = X[4]; //thetaDot
}

inline void measurementHGradient(const double X[STATE_LENGTH],
                          double Dh[MEASUREMENT_LENGTH*STATE_LENGTH])
{
   //with respect to x[0]
   Dh[0] = 0.;
   Dh[1] = 0.;
   Dh[2] = 0.;
   Dh[3] = 0.;

   //with respect to x[1]
   Dh[0+4] = 0.;
   Dh[1+4] = 0.;
   Dh[2+4] = 0.;
   Dh[3+4] = 0.;

   //with respect to x[2]
   Dh[0+4*2] = 1.;
   Dh[1+4*2] = 0.;
   Dh[2+4*2] = 0.;
   Dh[3+4*2] = 0.;

   //with respect to x[3]
   Dh[0+4*3] = 0.;
   Dh[1+4*3] = 0.;
   Dh[2+4*3] = 0.;
   Dh[3+4*3] = 0.;

   //with respect to x[4]
   Dh[0+4*4] = 0.;
   Dh[1+4*4] = 1.;
   Dh[2+4*4] = 0.;
   Dh[3+4*4] = 0.;
}
#endif
