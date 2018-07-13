/// 2010 Aug. 25
// m-PSO with iteration in each time setep.
// for comparison NEW and OLD algorithms.

#include "mpso.h"

#define WIDTH  500        // window size
#define HEIGHT 500        // window size
#define DIM 2             // CRL dimention of input vectors
#define NofParticles 100 //the number of particles
#define MOVE

double my_clock2()
{
struct timeval tv;
gettimeofday(&tv, NULL);
return tv.tv_sec + (double)tv.tv_usec*1e-6;
}
///////////////////////////////////////////////////
// $B%,%&%9J,I[$K=>$&Mp?t@8@.(B
//
// $B%\%C%/%9%_%e!<%i!<K!$rMxMQ!#%,%&%9J,I[$K=>$&Fs$D$N(B
// $BFHN)$7$?JQ?t$r@8@.$9$k$,!$0l$D$7$+JV$7$F$$$J$$!#(B
// $B$b$&0l$DJV$7$?$$>l9g$K$O!$(B
// sigma*sqrt(-2*log(x))*(sin(2*CV_PI*y))+m;
// $B$r7W;;$9$l$P$h$$!#(B
//
///////////////////////////////////////////////////
double n_rand(double m,double sigma)
{
  static double x,y,r;
  static CvRNG rng=cvRNG(-1);
  
  x=cvRandReal(&rng);
  y=cvRandReal(&rng);

  return sigma*sqrt(-2*log(x))*(cos(2*CV_PI*y))+m;
}

double l2norm(CvPoint2D32f pt1, CvPoint2D32f pt2)
{ return sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2));}

/////////////////////////////////////////////////////////
// $BI>2ACM7W;;(B
/////////////////////////////////////////////////////////
double evaluate(CvPoint2D32f p, int cnt)
{
  static int i;
  static double f;
  static double bx1=250;
  static double by1=250;
  static double s1_x=40;
  static double s1_y=40;

  f=1-exp(-0.5*(pow((p.x-bx1-125*sin(0.01*cnt)),2)/pow((s1_x),2)+
		pow((p.y-by1-125*cos(0.01*cnt)),2)/pow((s1_y),2)));

  return f;
}

///////////////////////////////////////////////////
// initialization of particles
///////////////////////////////////////////////////
int init_particles(PSO *pso)
{
  static int i;
  static double f;
  static CvRNG rng=cvRNG(-1);

  pso->d=DIM;
  pso->f_gbest=10e6;
  pso->l_f_gbest=10e6;

  for(i=0;i<NofParticles;i++){
    pso->ptcl[i].st.x.x=n_rand(0,1)+250;
    pso->ptcl[i].st.x.y=n_rand(0,1)+250;
    pso->ptcl[i].st.v.x=n_rand(0,1);
    pso->ptcl[i].st.v.y=n_rand(0,1);
    pso->ptcl[i].st.pbest.x=pso->ptcl[i].st.x.x;
    pso->ptcl[i].st.pbest.y=pso->ptcl[i].st.x.y;
    pso->ptcl[i].st.f_pbest=10e6;
    pso->ptcl[i].st.l_tmp_f_pbest=10e6;
  }

  return 0;
}

/////////////////////////////////////////////////////////
// $BI>2ACM7W;;(B
/////////////////////////////////////////////////////////
int time_evolution(PSO *pso, int C1, int C2)
{
  static int i;
  static double pbest;
  static double w,c1,c2;
  static CvRNG rng=cvRNG(-1);

  // The trackbars can deal with only int type.
  // So, in here the parameters are converted int to double.
  c1=1.40;
  c2=1.40;
  
  for(i=0;i<NofParticles;i++){
    pso->ptcl[i].st.v.x=
      pso->ptcl[i].st.w*pso->ptcl[i].st.v.x+
      c1*cvRandReal(&rng)*(pso->ptcl[i].st.pbest.x-pso->ptcl[i].st.x.x)+
      c2*cvRandReal(&rng)*(pso->gbest.x-pso->ptcl[i].st.x.x);

    pso->ptcl[i].st.v.y=
      pso->ptcl[i].st.w*pso->ptcl[i].st.v.y+
      c1*cvRandReal(&rng)*(pso->ptcl[i].st.pbest.y-pso->ptcl[i].st.x.y)+
      c2*cvRandReal(&rng)*(pso->gbest.y-pso->ptcl[i].st.x.y);

    pso->ptcl[i].st.x.x+=pso->ptcl[i].st.v.x;
    pso->ptcl[i].st.x.y+=pso->ptcl[i].st.v.y;

    if(pso->ptcl[i].st.x.x>WIDTH ) pso->ptcl[i].st.x.x=WIDTH -10;
    if(pso->ptcl[i].st.x.y>HEIGHT) pso->ptcl[i].st.x.y=HEIGHT-10;
    if(pso->ptcl[i].st.x.x<0) pso->ptcl[i].st.x.x=10;
    if(pso->ptcl[i].st.x.y<0) pso->ptcl[i].st.x.y=10;
  }

  return 0;
}

///////////////////////////////////////////////////
// $BN3;R$NIA2hMQ4X?t(B
///////////////////////////////////////////////////
int DrawParticles(IplImage* img, PSO pso, CvScalar color)
{
  int i;
  static int size=2;
  static CvPoint pt;

  for(i=0;i<NofParticles;i++){
    pt.x=(int)pso.ptcl[i].st.x.x; 
    pt.y=(int)pso.ptcl[i].st.x.y;
    cvCircle(img,pt,size,color,-1,8,0); 
  }
  return 0;
}

///////////////////////////////////////////////////
// $BN3;R$NIA2hMQ4X?t(B($BB.EY$rF1;~$KIA2h$9$k(B)
///////////////////////////////////////////////////
int DrawParticles2(IplImage* img, PSO pso, CvScalar color)
{
  int i,c;
  static int size=2;
  static double max, norm_v;
  static CvPoint pt1,pt2;

  for(i=0;i<NofParticles;i++){
    pt1.x=(int)pso.ptcl[i].st.x.x; 
    pt1.y=(int)pso.ptcl[i].st.x.y;
    pt2.x=(int)pso.ptcl[i].st.x.x+(int)pso.ptcl[i].st.v.x; 
    pt2.y=(int)pso.ptcl[i].st.x.y+(int)pso.ptcl[i].st.v.y;
    cvLine(img,pt1,pt2,cvScalar(0,255,0),1,CV_AA,0); 
    cvCircle(img,pt1,2,cvScalar(255,0,0),-1,8,0); 
  }
  return 0;
}

///////////////////////////////////////////////////
// $BI>2ACM$,:GBg$N>l=j$rIA2h(B
///////////////////////////////////////////////////
int DrawGbest(IplImage* img, CvPoint pt, CvScalar color)
{
  int i;
  static int size=4;
  static double max;

  cvCircle(img,pt,size,color,4,8,0); 
  return 0;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
int DrawTrajectory(IplImage* tra, CvPoint pt1, 
		   CvPoint pt2, CvScalar color)
{
  cvLine(tra,pt1,pt2, color,2,8,0);

  return 0;
}

///////////////////////////////////////////////////
// $BI>2ACM$,:GBg$N>l=j$rIA2h(B
///////////////////////////////////////////////////
int DrawTrue(IplImage* img, int cnt)
{
  int i;
  static CvPoint2D32f pt;
  
  pt.x=250+125*sin(0.01*cnt);
  pt.y=250+125*cos(0.01*cnt);

  cvCircle(img,cvPointFrom32f(pt),15,CV_RGB(100,100,100),2,8,0); 
  cvCircle(img,cvPointFrom32f(pt),20,CV_RGB(100,100,100),2,8,0); 
  return 0;
}
///////////////////////////////////////////////
//
// $B<4$NIA2h(B
//
///////////////////////////////////////////////
void DrawAxis(IplImage* img)
{
  static CvPoint ax0,ax1,ay0,ay1;
  static CvFont font;
  static CvFont font_it;
  static CvFont font_it_s;
  static char text[256];

  cvInitFont(&font,CV_FONT_HERSHEY_TRIPLEX,1,1,0.0,1,8);
  cvInitFont(&font_it,CV_FONT_HERSHEY_TRIPLEX | CV_FONT_ITALIC,1,1,0.0,1,8);
  cvInitFont(&font_it_s,CV_FONT_HERSHEY_TRIPLEX,0.7,0.7,0.0,1,8);

  ax0.x=ax1.x=WIDTH/2;
  ax0.y=HEIGHT;
  ax1.y=ay0.x=0;
  ay0.y=ay1.y=HEIGHT/2;
  ay1.x=WIDTH;
  
  cvLine(img,ax0,ax1,CV_RGB(100,100,100),1,8,0);
  cvLine(img,ay0,ay1,CV_RGB(100,100,100),1,8,0);

  sprintf(text,"x");  
  cvPutText(img,text,cvPoint(WIDTH-35,HEIGHT-10),&font_it,CV_RGB(100,100,100));
  sprintf(text,"1");  
  cvPutText(img,text,cvPoint(WIDTH-15,HEIGHT-6),&font_it_s,CV_RGB(100,100,100));
  sprintf(text,"x");  
  cvPutText(img,text,cvPoint(10,30),&font_it,CV_RGB(100,100,100));
  sprintf(text,"2");  
  cvPutText(img,text,cvPoint(30,34),&font_it_s,CV_RGB(100,100,100));

  //sprintf(text,"%1.1f",0.5);  
  //cvPutText(img,text,cvPoint(10,HEIGHT-10),&font,CV_RGB(100,100,100));
  sprintf(text,"%d",250);  
  cvPutText(img,text,cvPoint(WIDTH/2+10,HEIGHT-10),&font,CV_RGB(100,100,100));
  //sprintf(text,"%1.1f",0.5);  
  //cvPutText(img,text,cvPoint(WIDTH-60,HEIGHT-10),&font,CV_RGB(100,100,100));
  sprintf(text,"%d",250);  
  cvPutText(img,text,cvPoint(10,HEIGHT/2+30),&font,CV_RGB(100,100,100));
  sprintf(text,"%d",0);  
  cvPutText(img,text,cvPoint(10,HEIGHT-10),&font,CV_RGB(100,100,100));
}

///////////////////////////////////////////////////
// main
///////////////////////////////////////////////////
int main()
{
  char text[256];
  char comand[256];
  int i,j,k,l,n, code = 0;
  int C1=1400, C2=1400, W=800;
  int i_flag=0;
  long unsigned int cnt;
  double f;
  double eta=5.0; //fogetting rate
  double minf=0;
  double eval;
  double t1,t2,sumT=0;
  int cntT=1;

  CvPoint pt1,pt2;
  CvRNG rng=cvRNG(-1);
  CvFont font;
  IplImage *img;
  IplImage *tra;
  PSO pso;
  CvPoint2D32f pre_gbest;

  // colors
  CvScalar red  =CV_RGB(255,  0,  0);
  CvScalar blue =CV_RGB(  0,  0,255);
  CvScalar green=CV_RGB(  0,255,  0);
  CvScalar black=CV_RGB(  0,  0,  0);
  CvScalar white=CV_RGB(255,255,255);
  CvScalar gray =CV_RGB(125,125,125);

  FILE *gp;
  FILE *ev;
  ev=fopen("e_mpso.dat","w");
  if(ev==NULL){fprintf(stderr, "Do Not Opne File");exit(EXIT_FAILURE);}


  cvInitFont(&font,CV_FONT_VECTOR0,0.3,0.3,0.0,1,8);

  img = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
  tra = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
  cvSet(img,white,0);
  cvSet(tra,white,0);
  pt1.x=pt1.y=0; 
  pt2.x=WIDTH-1; 
  pt2.y=HEIGHT-1;

  // $BI=<(Ak$NMQ0U(B
  cvNamedWindow("PSO",CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("W*1e3", "PSO", &W, 1000, 0 );
  cvCreateTrackbar("C1*1e3", "PSO", &C1, 2000, 0 );
  cvCreateTrackbar("C2*1e3", "PSO", &C2, 2000, 0 );


  ///////////////////////////////////////////////////
  // GNUPLOT $B$rMxMQ$7$?F~NO4X?t%"%K%a!<%7%g%s$H3F<oJ?6Q8m:9%0%i%U$NI=<((B
  // ($B>/$7=E$$$N$GMxMQ$7$?$/$J$1$l$P(B#define GNUPLOT$B$r%3%a%s%H%"%&%H(B)
  // ($B=hM}$,DI$$IU$+$:$K(Bgnuplot$B$,%U%!%$%k%(%i!<$r=P$9$,L5;k$7$F$h$$(B)
  //
  // $B%"%K%a!<%7%g%s$+$i$Q$i$Q$iF02h$r@8@.$7$?$1$l$P!"(B
  // 1$B!#(B#define SAVE_INPUTIMAGE $B$rM-8z$K$7$F!"B??t$N(Bpng$B2hA|$rJ]B8!#(B
  // 2$B!#(Bpng$B$NL>A0$NIU$1J}$K$O7h$^$j$,$"$k$N$GCm0U!#(B
  // 3$B!#(Bpng$B$r0J2<$N%3%^%s%I$G(Bgif$B%U%!%$%k$KJQ49(B
  //    for f in *.png; do convert $f ${f%.png}.gif; done
  // 4$B!#(Bconvert$B$G0l$D$N(Bgif$B%"%K%a$r@=:n!#(Bdelay $B$NCM$r>.$5$/$9$k$HB.$/F0$/!#(B 
  //      convert -delay 10 i*.gif pfd_anim.gif
  // 5$B!#%"%K%a%U%!%$%k$N%A%'%C%/$O!"%&%'%V%V%i%&%6$G!#(B
  //
  ///////////////////////////////////////////////////
  //#define GNUPLOT
#ifdef GNUPLOT
  //GNUPLOT_INPUT
  gp=popen("gnuplot -geometry 500x350+510+00","w");
  if(gp==NULL){
    fprintf(stderr, "Do Not Opne GNUPLOT");
    exit(EXIT_FAILURE);
  }
  fprintf(gp,"set terminal x11 \n");
  //#define SAVE_INPUTIMAGE  
#ifdef SAVE_INPUTIMAGE  
  fprintf(gp,"set terminal png \n");
#endif
  fprintf(gp,"unset key        \n");
  fprintf(gp,"set xrange[0:500] \n");
  fprintf(gp,"set xlabel \"x1\" \n");
  fprintf(gp,"set yrange[0:500] \n");
  fprintf(gp,"set ylabel \"x2\" \n");
  fprintf(gp,"set zrange[0:1] \n");
  //fprintf(gp,"set pm3d \n");
  fprintf(gp,"set ticslevel 0 \n");
  //fprintf(gp,"set pm3d at b \n");
  fprintf(gp,"set isosample 30, 30 \n");
  //fprintf(gp,"set isosample 60, 60 \n");
  //fprintf(gp,"set hidden3d \n");
  fprintf(gp,"set view 40,300,1,1 \n");
  fprintf(gp,"pi=3.1415926535 \n");
  fprintf(gp,"a=1 \n");
  fprintf(gp,"s1_x=40 \n");
  fprintf(gp,"s1_y=40 \n");
  fprintf(gp,"bx1=250 \n");
  fprintf(gp,"by1=250 \n");
  fprintf(gp,"rp=pi/180*2\n");
  fprintf(gp,"t=0\n");
  fprintf(gp,"dt=1\n");
#ifdef MOVE
  fprintf(gp,"f(x,y,t)=1-exp(-0.5*((x-bx1-125*sin(0.01*t))**2/(s1_x)**2+(y-by1+125*cos(0.01*t))**2/(s1_y)**2))\n");
#else 
  fprintf(gp,"f(x,y,t)=1-exp(-0.5*((x-bx1-125)**2/(s1_x)**2+(y-by1+125)**2/(s1_y)**2))\n");
#endif
  fflush(gp);
#endif

  /////////////////////////////////////////////////////
  //PSO$B$N=i4|2=(B
  /////////////////////////////////////////////////////
  pso.ptcl=(PARTICLE*)malloc(sizeof(PARTICLE)*NofParticles);
  init_particles(&pso); //STEP 1
 
  //$B$3$3$+$i$,(Bsimulation$B$N%3%"(B
  for(cnt=0;;cnt++){    

    //$BIA2h$N=i4|2=!#GX7J$rGr?'$K(B     
    cvSet(img,white,0); 
    //$BOH$rIU$1$k(B($B<qL#$NLdBj(B)$B!#(B
    cvRectangle(img,pt1,pt2,black,1,4,0);
    //$B<4$rIA2h(B
    DrawAxis(img);
    DrawTrue(img,cnt);
   
    pre_gbest=pso.gbest;

    //t1=getrusage_sec(); //$B;~4V7WB,(B
    t1=my_clock2();

    static double f_x_last;
    static double den;
    static double eta;
    static double w;
    static double lammbda=0.1;
    static double eta_P, eta_G;

    //update the fitness function
    for(i=0;i<NofParticles;i++){
      //reevaluate the fitness value of pbest(t-1) and x(t-1)
      pso.ptcl[i].st.l_tmp_f_pbest=evaluate(pso.ptcl[i].st.pbest,cnt); 
      f_x_last=evaluate(pso.ptcl[i].st.x,cnt); 
      //choose pbest'(t-1) and F(pbest'(t-1),G(t-1))
      if(pso.ptcl[i].st.l_tmp_f_pbest>f_x_last){
	pso.ptcl[i].st.l_pbest=pso.ptcl[i].st.pbest;
	pso.ptcl[i].st.l_f_pbest=pso.ptcl[i].st.l_tmp_f_pbest;
      }
      else{
	pso.ptcl[i].st.l_pbest=pso.ptcl[i].st.x;
	pso.ptcl[i].st.l_f_pbest=f_x_last;
      }
    }
    //Choose the best of gbest'(t-1) from pbest'(t-1)
    for(pso.l_f_gbest=10e10,i=1;i<NofParticles;i++){
      if(pso.ptcl[i].st.l_f_pbest<pso.l_f_gbest){
	pso.l_gbest=pso.ptcl[i].st.l_pbest;
	pso.l_f_gbest=pso.ptcl[i].st.l_f_pbest;
      }
    }

    //calculate w
    for(i=0;i<NofParticles;i++){
      den=pso.ptcl[i].st.l_f_pbest-pso.ptcl[i].st.l_tmp_f_pbest;
      if(den!=0)
	eta_P=(pso.ptcl[i].st.l_f_pbest-pso.ptcl[i].st.f_pbest)/(double)den;
      else eta_P=0;
      den=pso.l_f_gbest-evaluate(pso.gbest,cnt);
      if(den!=0) eta_G=(pso.l_f_gbest-pso.f_gbest)/(double)den;
      else eta_G=0;
      
      eta=lammbda*eta_P+(1-lammbda)*eta_G;
      //pso.ptcl[i].st.w=(eta+1)/(double)2;
      pso.ptcl[i].st.w=(eta+1)*.9; //modified by Takeshi Nishida
      if(pso.ptcl[i].st.w<0.1) pso.ptcl[i].st.w=0.1; 
      if(pso.ptcl[i].st.w>1)   pso.ptcl[i].st.w=1;
    }
    
    if(cnt>0) time_evolution(&pso,C1,C2);
    
    for(i=0;i<NofParticles;i++){
      f=evaluate(pso.ptcl[i].st.x,cnt); 
      if(f<pso.ptcl[i].st.l_f_pbest){
	pso.ptcl[i].st.pbest=pso.ptcl[i].st.x; 
	pso.ptcl[i].st.f_pbest=f;	
      }
      else pso.ptcl[i].st.f_pbest=pso.ptcl[i].st.l_f_pbest;
    }
    for(pso.f_gbest=10e10,i=0;i<NofParticles;i++){
      if(pso.ptcl[i].st.f_pbest<pso.f_gbest){
	pso.gbest=pso.ptcl[i].st.pbest;
	pso.f_gbest=pso.ptcl[i].st.f_pbest;
      }
    }
    //t2=getrusage_sec(); //$B;~4V7WB,(B
    t2=my_clock2();

    sumT+=(t2-t1); cntT++;
    if(cntT%600==0){
      printf("%d %lf\n",cntT, sumT/(double)6);
      cntT=1; sumT=0;
    }

    DrawParticles2(img,pso,red);    
    DrawGbest(img,cvPointFrom32f(pso.gbest),red);    

    if(pre_gbest.x>10)
      DrawTrajectory(tra,cvPointFrom32f(pre_gbest),
		     cvPointFrom32f(pso.gbest),red);

    cvAddWeighted(img,0.7,tra,0.3,0,img);
    cvShowImage("PSO",img);

    eval=evaluate(pso.gbest, cnt);
    fprintf(ev,"%d %lf\n",cnt,eval);

    //////////////////////////////////////////////////////
    // $BF~NO?.9f$N%"%K%a!<%7%g%s$r:n@.$9$k(B
    //////////////////////////////////////////////////////
    //GNUPLOT 
#ifdef GNUPLOT
#ifdef SAVE_INPUTIMAGE  
    if(0.01*cnt>2*CV_PI) sprintf(text,"000",cnt);
    sprintf(comand,"set output \"./img_26/i%s.png\"\n",num);  
    fprintf(gp, comand);
#endif      
    sprintf(comand,"splot f(x,y,%d)\n",cnt);
    fprintf(gp, comand);
    fflush(gp);
#endif
    
    //////////////////////////////////////////////////////
    // $B=hM}7k2L2hA|$r(Bpgm$B7A<0$GJ]B8(B
    //
    // $BJ]B8@h$N%G%#%l%/%H%j$OA0$b$C$F:n$C$F$*$/$3$H(B!!
    //////////////////////////////////////////////////////
    //#define SAVEIMAGE_SNAP
#ifdef SAVEIMAGE_SNAP
    //save images
    if(0.01*cnt>2*CV_PI && i_flag==0){
      sprintf(text,"res");
      i_flag=1;
      sprintf(comand,"./%s.png",text); 
      cvSaveImage(comand,img);
    }
#endif

    //#define MAKE_AMIME
#ifdef MAKE_AMIME
    if(cnt<10)                    sprintf(text,"00%d",cnt);
    else if(10<=cnt  && cnt<100)  sprintf(text, "0%d",cnt);
    else if(100<=cnt && cnt<628) sprintf(text,  "%d",cnt);
    sprintf(comand,"./img0922_pso8/p%s.png",text);  
    cvSaveImage(comand,img);
#endif      
    if(cnt==471) cvSaveImage("mpso.png",img);
      
    //$B$3$3$NCM$rBg$-$/$9$k$H<B9T$,CY$/$J$k!#(B
    code = cvWaitKey(10);

    //OpenCV$B$NI=<(2hLL>e$G(B q $B$r2!$9$H%W%m%0%i%`$,=*N;$9$k!#(B
    if(code==27 || code=='q' || code =='Q') break;
  }
  return 0;
}



