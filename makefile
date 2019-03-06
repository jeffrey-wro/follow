OBJS	= main.o
SOURCE	= main.cpp
HEADER	= 
OUT		= follow.out
CC		= /usr/local/oecore-x86_64/sysroots/x86_64-nilrtsdk-linux/usr/bin/arm-nilrt-linux-gnueabi/arm-nilrt-linux-gnueabi-g++ 
CFLAGS	= --sysroot=/usr/local/oecore-x86_64/sysroots/cortexa9-vfpv3-nilrt-linux-gnueabi -std=c++11 -pthread
INCLUDE = -I../imageSender/imageSender -I../opencvLib/include/opencv4/ -I../myrio/myrio -I../myrio/myrio/csupport -I"/usr/local/oecore-x86_64/sysroots/cortexa9-vfpv3-nilrt-linux-gnueabi/usr/include" -I"/usr/local/oecore-x86_64/sysroots/cortexa9-vfpv3-nilrt-linux-gnueabi/usr/include/c++/4.9.2/arm-nilrt-linux-gnueabi" 
FLAGS	= -g -c -Wall -DMyRio_1900 -O0 -g3 -Wall -std=gnu++11 -fmessage-length=0 -mfpu=vfpv3 -mfloat-abi=softfp
LFLAGS 	= -L../imageSender/imageSender -L/usr/lib/opencv -L../opencvLib/opencv -L../myrio/myrio -L../myrio/myrio/csupport -limagesender -lcsupport -lmyrio -limagesender -ldl -lopencv_calib3d -lopencv_core -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_gapi -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_videoio

all: $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDE) -g $(OBJS) -o $(OUT)  $(LFLAGS)

main.o: main.cpp
	$(CC) $(CFLAGS) $(INCLUDE) $(FLAGS) main.cpp

clean:
	rm -f $(OBJS) $(OUT)

