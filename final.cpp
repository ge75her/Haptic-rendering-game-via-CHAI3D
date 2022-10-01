//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2007 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// GENERAL SETTINGS
//---------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

string resourceRoot;


//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE objects
cODEGenericBody* ODEBody0;
cODEGenericBody* ODEBody1;
cODEGenericBody* ODEBody2;
cODEGenericBody* ODEBody3;
cODEGenericBody* ODEBody4;
cODEGenericBody* ODEBody5;

cMesh* object0;
cMesh* object1;
cMesh* object2;
cMesh* object3;
cMultiMesh* ball;
cMultiMesh* goal;

// ODE objects
cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

//radius of ode
double radius;
//radius of tool
double toolRadius;
// Stiffness
double K;
//Audio
// audio device to play sound
cAudioDevice* audioDevice;

// audio buffers to store sound files
cAudioBuffer* audioBuffer1;
cAudioBuffer* audioBuffer2;
cAudioBuffer* audioBuffer3;
cAudioBuffer* audioBuffer4;




//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);



//===========================================================================
/*
    We build a small game. Push the football from top right to left bottom 
    with the haptic device. 
    object0: the plane on the left bottom
    object1: the plane in the center
    object2: the plane on the top right
    object3: football
    object4: goal
 */
 //===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "final project---------level 1" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[r] - Reset the position" << endl;
    cout << "[w] - reduce magnetic param K" << endl;
    cout << "[g] - Enable/Disable gravity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.9 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);
    background->setFixedAspectRatio(true);
    background->loadFromFile("../../../bin/resources/images/sky.jpg");
    // position and orient the camera
    camera->set(cVector3d(2.5, 0.0, 0.3),    // camera position (eye)
        cVector3d(0.0, 0.0, -0.5),    // lookat position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.0, 1.2);

    // define the direction of the light beam
    light->setDir(0.0, 0.0, -1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (gripper or pointer)

    tool = new cToolCursor(world);

    // insert tool into world
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the virtual tool contact points (sphere)
    toolRadius = 0.06;
    tool->setRadius(toolRadius, toolRadius);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    //-----------------------------------------------------------------------
    // SETUP AUDIO
    //-----------------------------------------------------------------------
    // create an audio device to play sounds
    audioDevice = new cAudioDevice();

    // attach audio device to camera
    camera->attachAudioDevice(audioDevice);

    // create an audio buffer and load audio wave file
    audioBuffer1 = new cAudioBuffer();
    bool fileload1 = audioBuffer1->loadFromFile("../../../bin/resources/sounds/metal-scraping.wav");
    if (fileload1) {
        cout << "metal-scraping loaded" << endl;
    }
    // create an audio buffer and load audio wave file
    audioBuffer2 = new cAudioBuffer();
    bool fileload2 = audioBuffer2->loadFromFile("../../../bin/resources/sounds/metal-impact.wav");
    if (fileload2) {
        cout << "metal-impact loaded" << endl;
    }
    // create an audio buffer and load audio wave file
    audioBuffer3 = new cAudioBuffer();
    bool fileload3 = audioBuffer3->loadFromFile("../../../bin/resources/sounds/wood-scraping.wav");
    if (fileload3) {
        cout << "wood-scraping loaded" << endl;
    }
    audioBuffer4 = new cAudioBuffer();
    bool fileload4 = audioBuffer3->loadFromFile("../../../bin/resources/sounds/wood-impact.wav");
    if (fileload4) {
        cout << "wood-impact loaded" << endl;
    }
    audioBuffer1->convertToMono();
    audioBuffer2->convertToMono();
    audioBuffer3->convertToMono();
    audioBuffer4->convertToMono();
    // create an audio source for this tool.
    tool->createAudioSource(audioDevice);



    //-----------------------------------------------------------------------
    // CREATE ODE WORLD AND OBJECTS
    //-----------------------------------------------------------------------

    /////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxDamping = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;
    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.00, 0.00, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);


    //////////////////////////////////////////////////////////////////////////
    // 3 ODE BLOCKS
    //////////////////////////////////////////////////////////////////////////

    // create texture
    cTexture2dPtr texture = cTexture2d::create();
    // load texture file
    bool fileload = texture->loadFromFile("../../../bin/resources/images/grass.jpg");

    // create a new ODE object that is automatically added to the ODE world

    ODEBody0 = new cODEGenericBody(ODEWorld);
    ODEBody1 = new cODEGenericBody(ODEWorld);
    ODEBody2 = new cODEGenericBody(ODEWorld);
    ODEBody3 = new cODEGenericBody(ODEWorld);
    ODEBody4 = new cODEGenericBody(ODEWorld);
    ODEBody5 = new cODEGenericBody(ODEWorld);
    // create a virtual mesh  that will be used for the geometry representation of the dynamic body
    cMesh* object0 = new cMesh();
    cMesh* object1 = new cMesh();
    cMesh* object2 = new cMesh();



    // create a cube mesh
    double size = 0.04;
    cCreateBox(object0, 10 * size, 40 * size, size);
    object0->createAABBCollisionDetector(toolRadius);

    cCreateBox(object1, 10 * size, 35 * size, size);
    object1->createAABBCollisionDetector(toolRadius);

    cCreateBox(object2, 10 * size, 40 * size, size);
    object2->createAABBCollisionDetector(toolRadius);

    // define some material properties for each cube
    cMaterial mat0, mat1, mat2, mats;
    //mat0.setRedIndian();
    mat0.setStiffness(0.3 * maxStiffness);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    object0->setMaterial(mat0, true);
    object0->setTexture(texture);
    object0->m_texture->setSphericalMappingEnabled(true);
    object0->setUseTexture(true);
    object0->m_material->setViscosity(1000.0 * maxDamping);            // % of maximum linear damping

    // create a haptic surface effect
    object0->createEffectSurface();

    // create a haptic viscous effect
    object0->createEffectViscosity();
    // set audio properties
    object0->m_material->setAudioFrictionBuffer(audioBuffer1);
    object0->m_material->setAudioFrictionGain(0.8);
    object0->m_material->setAudioFrictionPitchGain(0.2);
    object0->m_material->setAudioFrictionPitchOffset(0.8);
    object0->m_material->setAudioImpactBuffer(audioBuffer2);
    object0->m_material->setAudioImpactGain(0.2);

    //mat1.setBlueRoyal();
    mat1.setStiffness(0.3 * maxStiffness);
    mat1.setDynamicFriction(0.6);
    mat1.setStaticFriction(0.6);
    object1->setMaterial(mat1, true);
    object1->setTexture(texture);
    object1->m_texture->setSphericalMappingEnabled(true);
    object1->setUseTexture(true);

    //set material properties
    object1->m_material->setMagnetMaxForce(0.6 * maxLinearForce);   // % of maximum linear force 
    object1->m_material->setMagnetMaxDistance(0.15);
    object1->m_material->setViscosity(10 * maxDamping);
    object1->createEffectSurface();
    object1->createEffectMagnetic();
    object1->createEffectViscosity();
    // set audio properties
    object1->m_material->setAudioFrictionBuffer(audioBuffer1);
    object1->m_material->setAudioFrictionGain(0.8);
    object1->m_material->setAudioFrictionPitchGain(0.2);
    object1->m_material->setAudioFrictionPitchOffset(0.8);
    object1->m_material->setAudioImpactBuffer(audioBuffer2);
    object1->m_material->setAudioImpactGain(0.2);



    mat2.setStiffness(10 * maxStiffness);
    mat2.setDynamicFriction(0.6);
    mat2.setStaticFriction(1.0);
    object2->setMaterial(mat2, true);
    object2->setTexture(texture);
    object2->m_texture->setSphericalMappingEnabled(true);
    object2->setUseTexture(true);

    // set audio properties
    object2->m_material->setAudioFrictionBuffer(audioBuffer1);
    object2->m_material->setAudioFrictionGain(0.8);
    object2->m_material->setAudioFrictionPitchGain(0.2);
    object2->m_material->setAudioFrictionPitchOffset(0.8);
    object2->m_material->setAudioImpactBuffer(audioBuffer2);
    object2->m_material->setAudioImpactGain(0.2);


    

    // add mesh to ODE object
    ODEBody0->setImageModel(object0);
    ODEBody1->setImageModel(object1);
    ODEBody2->setImageModel(object2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEBody0->createDynamicBox(10 * size, 40 * size, size, true);
    ODEBody1->createDynamicBox(10 * size, 35 * size, size, true);
    ODEBody2->createDynamicBox(10 * size, 40 * size, size, true);

    // define some mass properties for each cube
    ODEBody0->setMass(0.05);
    ODEBody1->setMass(0.05);
    ODEBody2->setMass(0.05);

    // set position of each cube
    ODEBody0->setLocalPos(0.0, -1.0, -0.7);
    ODEBody1->setLocalPos(0.0, 0.3, -0.5);
    ODEBody2->setLocalPos(0.0, 1.5, -0.3);


    cMultiMesh* ball = new cMultiMesh();
    cMultiMesh* goal = new cMultiMesh();
    fileload = ball->loadFromFile("../../../bin/resources/models/ball 3DS.3ds");
    if (fileload) {
        cout << "texture ball correctly loaded" << endl;
    }


    ball->scale(0.08);
    ball->createAABBCollisionDetector(toolRadius);

    ball->setMaterial(mat1, true);

    // set audio properties
    for (int i = 0; i < (ball->getNumMeshes()); i++) {
        (ball->getMesh(i))->m_material->setAudioFrictionBuffer(audioBuffer3);
        (ball->getMesh(i))->m_material->setAudioFrictionGain(0.8);
        (ball->getMesh(i))->m_material->setAudioFrictionPitchGain(0.8);
        (ball->getMesh(i))->m_material->setAudioFrictionPitchOffset(0.8);
        (ball->getMesh(i))->m_material->setAudioImpactBuffer(audioBuffer4);
        (ball->getMesh(i))->m_material->setAudioImpactGain(0.8);
    }

    ODEBody3->setImageModel(ball);
    ball->computeBoundaryBox(true);
    radius = 0.5 * (ball->getBoundaryMax().x() - ball->getBoundaryMin().x());
    ODEBody3->createDynamicSphere(radius);
    ODEBody3->setMass(0.05);
    ODEBody3->setLocalPos(0.0, 1.9, -0.09);

    fileload = goal->loadFromFile("../../../bin/resources/models/football goal.3ds");
    if (fileload) {
        cout << "texture goal correctly loaded" << endl;
    }
    goal->scale(0.0008);
    goal->createAABBCollisionDetector(toolRadius);
    goal->setMaterial(mat0, true);
    goal->m_material->setVibrationFrequency(50);
    goal->m_material->setVibrationAmplitude(0.1 * maxLinearForce);
    goal->m_material->setStiffness(0.1 * maxStiffness);

    // set audio properties
    for (int i = 0; i < (goal->getNumMeshes()); i++) {
        (goal->getMesh(i))->m_material->setAudioFrictionBuffer(audioBuffer3);
        (goal->getMesh(i))->m_material->setAudioFrictionGain(0.8);
        (goal->getMesh(i))->m_material->setAudioFrictionPitchGain(0.8);
        (goal->getMesh(i))->m_material->setAudioFrictionPitchOffset(0.8);
        (goal->getMesh(i))->m_material->setAudioImpactBuffer(audioBuffer4);
        (goal->getMesh(i))->m_material->setAudioImpactGain(0.8);
    }
    ODEBody4->setImageModel(goal);
    goal->computeBoundaryBox(true);
    // create a haptic vibration effect
    goal->createEffectVibration();

    // create a haptic surface effect
    goal->createEffectSurface();


    ODEBody4->createDynamicMesh(true);
    ODEBody4->setMass(0.7);
    ODEBody4->setLocalPos(-0.15, -1.6, -0.66);




    //////////////////////////////////////////////////////////////////////////
    // ODE INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 3 static walls to contains the 3 cubes within a limited workspace
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    //z,y,x
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, 2.0), cVector3d(0.0, 0.0, -1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -1.0), cVector3d(0.0, 0.0, 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0, 2.3, 0.0), cVector3d(0.0, -1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -1.8, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d(-0.2, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(0.2, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));

    


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable gravity
    else if (a_key == GLFW_KEY_G)
    {
        if (ODEWorld->getGravity().length() > 0.0)
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        }
        else
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
        }
    }
    //option - reset the position
    else if (a_key == GLFW_KEY_R)
    {
        ODEBody3->setLocalPos(cVector3d(0.0, 1.9, -0.09));

    }

    //option - reduce magnetic field
    else if (a_key == GLFW_KEY_W)
    {
        K = K - 0.2;

    }
    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
    delete audioDevice;
    delete audioBuffer1;
    delete audioBuffer2;
    delete audioBuffer3;
    

}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while (simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

        K = 1.0;
        cVector3d force;
        force.zero();
        cVector3d left = ODEBody2->getBoundaryMin();
        cVector3d right = ODEBody2->getBoundaryMax();
        cVector3d pos0 = ODEBody4->getLocalPos();
        cVector3d pos1 = ODEBody3->getLocalPos();
        cVector3d dir_left = cNormalize(pos0 - pos1);
        double distance_left = cDistance(pos0, pos1);

        if (distance_left < 0.6)
        {
            //force.add(dir01);
            force.add(-K * (distance_left - 0.6 * 0.04) * dir_left);
        }

        ODEBody3->addExternalForce(-force);
        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly

        int numInteractionPoints = tool->getNumHapticPoints();
        for (int i = 0; i < numInteractionPoints; i++)
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(i);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();

            for (int i = 0; i < numContacts; i++)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                        collisionEvent->m_globalPos);


                }

            }
        }

        // update simulation
        ODEWorld->updateDynamics(timeInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------


