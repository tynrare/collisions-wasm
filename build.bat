rmdir .\exports\
mkdir exports

call %EMSDK%/upstream/emscripten/tools/webidl_binder.bat box2d.idl exports/box2d_glue
emcc main.cpp glue.cpp -std=c++11 --post-js exports/box2d_glue.js -o exports/collisions.js