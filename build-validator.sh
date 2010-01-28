SRC="src/Decoder.cpp src/OggIndexValid.cpp src/Options.cpp src/SkeletonEncoder.cpp src/Utils.cpp src/Validate.cpp"

if test -x `which pkg-config`
then
  pkg-config --exists oggkate
  if test $? -eq 0; then EXTRA_FLAGS="`pkg-config --cflags --libs oggkate` -DHAVE_KATE"; else echo "libkate not found"; fi
fi

g++ $EXTRA_FLAGS -O0 -g $SRC -Wall -l ogg -l theoradec -l vorbis -o OggIndexValid
