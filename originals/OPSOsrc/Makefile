# robov用Makefile
# マクロ定義
#CC	= gcc 
#CC	= g++ 
CC	= icpc
#OPENCV_INC	= -I/usr/local/include/opencv
#OPENCV_LIB	= -L/usr/local/lib -lcxcore -lcv -lhighgui -lcvaux
#OPT_FLAG	= -O3 -mcpu=i686
#OPT_FLAG	= -O3 -mcpu=athlon
#OPT_FLAG	= -O3 -tpp7 -xK -unroll16 -c99 -prec_div -fno-alias -fno-fnalias -ansi_alias -ip -nobss_init -Zp16 -vec_report5
#CPPFLAGS	= -g  $(OPT_FLAG) $(OPENCV_INC)  $(IPL_INC)
#LDLIBS		= $(OPENCV_LIB) $(IPL_LIB) $(GPG_LIB) -lm

CXXFLAGS = `pkg-config --cflags opencv`
LDFLAGS = `pkg-config --libs opencv`
CFLAGS = -Wall -I./wrapplot -I./wrapunix -g
OPENCV_INC = -I/usr/local/include/opencv2
OPT_FLAG = -O3 -mcpu=i686
CPPFLAGS = -g $(OPT_FLAG) $(OPENCV_INC) $(IPL_INC) $(CXXFLAGS) $(LDFLAGS)
LDLIBS = $(IPL_LIB) $(GPG_LIB) -lm

all:	$(TARGET)

clean: 
	rm -f *.o $(TARGETS)


