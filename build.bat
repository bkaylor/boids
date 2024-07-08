@pushd bin
cl ..\src\main.c /Feboids.exe /Zi /I..\msvc_sdl\SDL2-2.0.9\include /I..\msvc_sdl\SDL2_ttf-2.0.15\include /I..\msvc_sdl\SDL2_image-2.0.4\include /link /LIBPATH:..\msvc_sdl\SDL2-2.0.9\lib\x64 /LIBPATH:..\msvc_sdl\SDL2_ttf-2.0.15\lib\x64 /LIBPATH:..\msvc_sdl\SDL2_image-2.0.4\lib\x64 /SUBSYSTEM:CONSOLE "SDL2_ttf.lib" "SDL2_image.lib" "SDL2main.lib" "SDL2.lib"
@popd