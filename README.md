# Haptic-rendering-game-via-CHAI3D
A small football game implement on CHAI3D. The goal is to push the football object from the top right plane to bottom right goal with a suitable force.
## Setup
- download CHAI3D open source from [https://www.chai3d.org/download/releases].
- download this repo
- find modules->ODE->bin->resources folder, add images & models & sounds files to the related folders
- find modules->ODE->examples->GLFW->01-ODE-CUBE folder, add final.cpp file
- connect a Haptic Device, in the test repo,we use Novint Falcon
- run 'final.cpp' file, move haptic device to push the football go ahead
## Button control
[r] - Reset the position  
[w] - reduce magnetic param K  
[g] - Enable/Disable gravity  
[f] - Enable/Disable full screen mode  
[m] - Enable/Disable vertical mirroring  
[q] - Exit application  
