TARGET  = niodump.exe
CC      = wcl
CFLAGS  = -q -0 -bt=dos -ms -I../include

$(TARGET): niodump.c ../sys/ioctl.h ../include/nio.h
	$(CC) $(CFLAGS) -fe=$@ niodump.c

clean : .SYMBOLIC
	rm -f $(TARGET) *.obj *.err
