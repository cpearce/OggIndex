SRC="src/Decoder.cpp src/OggIndexValid.cpp src/Options.cpp src/SkeletonEncoder.cpp src/Utils.cpp src/Validate.cpp"

g++ -g $SRC -Wall -l ogg -l theoradec -l vorbis -o OggIndexValid
