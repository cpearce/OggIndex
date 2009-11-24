SRC="src/Decoder.cpp src/OggIndex.cpp src/Options.cpp src/SkeletonEncoder.cpp src/Utils.cpp src/Validate.cpp"

g++ -O2 -Wall $SRC -l ogg -l theoradec -l vorbis -o OggIndex
