#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

//#include <cv.h>       // OpenCV
//#include <highgui.h>  // OpenCV
//#include <cvaux.h>    // OpenCV
//#include <cxcore.h>   // OpenCV

//OpenCV2.-
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

typedef struct{
  CvPoint2D32f x;    // position
  CvPoint2D32f v;     //velocity
  CvPoint2D32f pbest; // parsonal best
  CvPoint2D32f tmppbest; // parsonal best
  double f_pbest;     // f(pbest)
  double tmp_pbest;     // f(pbest)
  double norm_v;     // f(pbest)
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

