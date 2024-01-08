rmdir -r .\exports\
mkdir exports

call %EMSDK%/upstream/emscripten/tools/webidl_binder.bat box2d.idl exports/box2d_glue
emcc -O2 main.cpp glue.cpp -std=c++11 --post-js exports/box2d_glue.js -o exports/collisions.js  -s NO_FILESYSTEM=1  -s NO_EXIT_RUNTIME=1 -sENVIRONMENT=web -s EXPORT_ES6=1 -s MODULARIZE=1 -s EXPORT_NAME="Collisions"

# 