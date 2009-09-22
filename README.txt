OggIndex - Indexes ogg theora/vorbis files for faster seeking.

See IndexSpecificationVersion1.txt for index format details.

BUILDING ON LINUX

To build on Linux you require the ogg, theora and vorbis libraries to be installed. Provided those 
libraries are installed, building should be as simple as running "build.sh".

BUILDING ON WINDOWS

To build on Windows, use the Visual Studio 2005 solution at win32/VS2005/OggIndex.sln. You must have 
the ogg, theora and vorbis libraries available in OggIndex's parent directory, e.g. the theora, vorbis 
and ogg directories containing those libraries should be in the same directory as the OggIndex 
directory.
