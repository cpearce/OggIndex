.PHONY: all clean install tags

VPATH=src
FFMPEG=$(shell which ffmpeg)
BINDIR=$(shell dirname $(FFMPEG))
PREFIX=$(shell dirname $(BINDIR))
INCDIR=$(PREFIX)/include
LIBDIR=$(PREFIX)/lib

LIBS=-logg -ltheoradec -lvorbis 

INCLUDES=-I/opt/local/include -I$(INCDIR)
LDFLAGS=-L/opt/local/lib $(LIBS) -L$(LIBDIR)
CPPFLAGS=-Wall -O3 $(INCLUDES)
INSTALL_PREFIX=$(PREFIX)

INDEXER_OBJS=\
	     src/Decoder.o \
	     src/OggIndex.o \
	     src/Options.o \
	     src/SkeletonEncoder.o \
	     src/Utils.o \
	     src/Validate.o

INDEXER=OggIndex

VALIDATOR_OBJS=\
	       src/Decoder.o \
	       src/OggIndexValid.o \
	       src/Options.o \
	       src/SkeletonEncoder.o \
	       src/Utils.o \
	       src/Validate.o

VALIDATOR=OggIndexValid

OBJS=$(INDEXER_OBJS) $(VALIDATOR_OBJS)

all: $(INDEXER) $(VALIDATOR)

$(INDEXER): $(INDEXER_OBJS)
	g++ $(LDFLAGS) -o $@ $^

$(VALIDATOR): $(VALIDATOR_OBJS)
	g++ $(LDFLAGS) -o $@ $^

clean:
	rm -rf $(OBJS) $(INDEXER) $(VALIDATOR) *.dSYM $(INDEXER)-*.tar.gz
	rm -f *.gcov *.gcda *.gcno

install: $(INDEXER) $(VALIDATOR)
	install -d $(INSTALL_PREFIX)/bin
	install $^ $(INSTALL_PREFIX)/bin

tags:
	ctags -R .
