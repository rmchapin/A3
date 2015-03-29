include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_ARM = $(LIB_PATH)/libarm.a
LIB_ARM_OBJS = 

ALL = $(LIB_ARM)

all: $(ALL)

$(LIB_ARM): $(LIB_ARM_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
