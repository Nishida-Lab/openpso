#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <errno.h>

#include <cv.h>       // OpenCV
#include <highgui.h>  // OpenCV
#include <cvaux.h>    // OpenCV
#include <cxcore.h>   // OpenCV

typedef struct{
  CvPoint2D32f x;    // position
  CvPoint2D32f v;     //velocity 
  CvPoint2D32f a;     //
  CvPoint2D32f pbest; // parsonal best
  CvPoint2D32f tmppbest; // parsonal best
  double f_pbest;     // f(pbest)
  double tmp_pbest;     // f(pbest)
  double v_norm;     // f(pbest)
  double Q;
}STATE;

typedef struct{
  STATE st;           //state
}PARTICLE;

typedef struct{
  int    d;           //dimention
  PARTICLE* ptcl;
  CvPoint2D32f gbest; // grobal best
  CvPoint2D32f tmpgbest; // grobal best
  double f_gbest;     //g(gbest)
  double tmp_gbest;     //g(gbest)
}PSO;

#define WIDTH  500       // window size
#define HEIGHT 500       // window size
#define DIM 2            // CRL dimention of input vectors
#define NofParticles 100 // the number of particles
#define NEUTRAL      50  // the number of neutral particle
#define MOVE

#define v_max 16 
static double p=1;
static double p_core=2*sqrt(3*v_max);
static double L=1;




