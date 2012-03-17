CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 `pkg-config opencv --cflags ` -I /usr/include/libusb-1.0



OBJS =	testhull.o RobotMath.o beagleSender.o mongoose.o axisSender.o


LIBS =	`pkg-config opencv --libs` -lfreenect --ldl --pthread

TARGET =	kinectopencv

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

