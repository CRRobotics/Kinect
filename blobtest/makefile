CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 `pkg-config opencv --cflags ` -I /usr/include/libusb-1.0



OBJS =	testblobs.o


LIBS =	`pkg-config opencv --libs` -lfreenect -lblob

TARGET =	kinectblobs

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

