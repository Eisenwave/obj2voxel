#!/bin/sh
ln -sf tinyobjloader/tiny_obj_loader.h tinyobj.hpp
ln -sf tayweeargs/args.hxx             args.hpp
# git bash on Windows doesn't allow us to create the symlinks in place, so we need to mv instead
mv tinyobj.hpp src/3rd_party/tinyobj.hpp
mv args.hpp src/3rd_party/args.hpp