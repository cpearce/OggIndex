OggIndex - Indexes ogg theora/vorbis files for faster seeking.
OggIndexValid - Validates a file's keyframe index.

See Skeleton-4.0-Index-Specification.txt for index format details.

BUILDING ON LINUX

To build on Linux you require the ogg, theora and vorbis libraries to be 
installed. Provided those libraries are installed, building the indexer
should be as simple as running "build-indexer.sh". To build the validator
run "build-validator.sh". If you have libkate installed, it will 
automatically build with kate support.

BUILDING ON WINDOWS

To build on Windows, use the Visual Studio 200{5,8} solution at 
win32/VS200{5,8}/OggIndex.sln. You must have the ogg, theora, vorbis 
and libkate libraries available in OggIndex's parent directory, e.g.
the theora, vorbis and ogg directories containing those libraries
should be in the same directory as the OggIndex directory. Build
the OggIndex project to build the indexer, and OggIndexValid to build
the validator.

You can disable Kate support by removing the HAVE_KATE preprocessor
definitions from the OggIndex and OggIndexValid's project properties.
