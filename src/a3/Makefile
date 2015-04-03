include ../common.mk

COMMON_FLAGS = $(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
	$(CFLAGS_EIGEN) \
	$(CFLAGS_USB)

CFLAGS = $(COMMON_FLAGS) $(CFLAGS_STD) -g -DNDEBUG
CXXFLAGS = $(COMMON_FLAGS) $(CXXFLAGS_STD) -g -DNDEBUG

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_MAPPING) \
	$(LDFLAGS_GSL) \
	$(LDFLAGS_EIGEN) \
	$(LDFLAGS_USB)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_A3 = $(LIB_PATH)/liba3.a
LIB_A3_OBJS = LineFitter.o \
	CoordinateConverter.o \
	BlobDetector.o \
	Mutex.o \
	Freenect.o \
	Arm.o \
	LcmHandler.o

ALL = $(LIB_A3)

all: $(ALL)

$(LIB_A3): $(LIB_A3_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
