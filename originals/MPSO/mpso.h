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
  CvPoint2D32f x;      // position
  CvPoint2D32f v;      //velocity 
  CvPoint2D32f pbest;  // parsonal best
  double f_pbest;      // f(pbest) F(pbest(t),G(t))
  double w;      // weight

  CvPoint2D32f l_pbest;  // parsonal best
  double l_f_pbest;      // f(pbest) F(pbest'(t-1),G(t))
  double l_tmp_f_pbest;  // f(pbest) F(pbest(t-1),G(t))
}STATE;

typedef struct{
  STATE st;           //state
}PARTICLE;

typedef struct{
  int    d;           //dimention
  PARTICLE* ptcl;
  CvPoint2D32f gbest; // grobal best
  double f_gbest;        //g(gbest)   F(gbest(t-1),G(t-1)) 

  CvPoint2D32f l_gbest; // grobal best 
  double l_f_gbest;     //g(gbest)     F(gbest'(t-1),G(t))
}PSO;


#define WIDTH  500        // window size
#define HEIGHT 500        // window size
#define DIM 2             // CRL dimention of input vectors
#define NofParticles 100 //the number of particles
#define MOVE


