include ../common.mk

COMMON_FLAGS = $(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
	$(CFLAGS_EIGEN)

CFLAGS = $(COMMON_FLAGS) $(CFLAGS_STD) -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_MAPPING) \
	$(LDFLAGS_GSL) \
	$(LDFLAGS_EIGEN)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_A3 = $(LIB_PATH)/liba3.a
LIB_A3_OBJS = LineFitter.o \
	CoordinateConverter.o \
	BlobDetector.o

ALL = $(LIB_A3)

all: $(ALL)

$(LIB_A3): $(LIB_A3_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
