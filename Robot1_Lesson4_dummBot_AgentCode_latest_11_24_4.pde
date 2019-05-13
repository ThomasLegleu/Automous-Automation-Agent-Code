import toxi.geom.*; 
import toxi.geom.mesh.*;
import toxi.volume.*;
import toxi.processing.*;
import java.text.DecimalFormat;
import SimpleOpenNI.*;
import java.util.*;
import processing.net.*;


import peasy.test.*;
import peasy.org.apache.commons.math.*;
import peasy.*;
import peasy.org.apache.commons.math.geometry.*;
import processing.opengl.*;
import processing.opengl.*;


import controlP5.*;
import damkjer.ocd.*;




/////////click on window and press 'c' to close the simulation and the sockets///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  Agent Code Group 3 

////////  pre pseudo code // to do  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  add gui to the code not as crucial asa ctually getting the code to run

//  1 mport ocd cam 

// for presentation and animations
// see casey's code from the class on camera set up
// which class code was that

//  2 create a class of attractors

//  3 put those attractor points on different points of the sampled surface 

//  4 tell agents to seek the attractor points set limit pretty low so the agents stay on the mesh surface and
// never stay on the attractor point though i.e.constantly moving

//  5 after a certain amount of time release a new set of agents to do something once the stigmergy is in place
// sample the points and generate a certain plunge movement
// could track along the trail for a certain amount of time and then break off 
// bezier curve between two points 
// other examples
// spiral plunges based on certain points 


SimpleOpenNI context;

ArrayList pts; 
ArrayList matter; 
ArrayList pathList;
ArrayList meshList;
ArrayList dumList; 
ArrayList agentList;


ArrayList attList;
ArrayList trailList;


int picTake;
ArrayList avPts;
ArrayList captureList;

float nearClip = 100;
float farClip = 500;

int lock = 0;
int rM = 1;
int screenSwitch = 0;


WETriangleMesh mesh;
float isoDens = 1;  // 
float ISO_THRESHOLD = .1;
float NS = 0.03;



///////////////////// isosurface stuff   /////////////////////////////////////////////////////

VolumetricSpaceArray volume; // three dimensional voxel grid of points that contain a value
VolumetricBrush brush; // actual calculation of the threshold of which is exterior or interior of the isosurface
IsoSurface surface;   // class that allows a point to provide information to a voxel 

TriangleMesh mesh2 = new TriangleMesh("mesh2");
float ISO = 0.3;
int GRID = 40;
int DIM = 600;

Vec3D scale = new Vec3D(DIM,DIM,DIM);

ToxiclibsSupport gfx;
int sendSwitch = 1;



///////littleGuy is setup as the simulating instance of the robot, 
robot littleGuy;


////////socketIn is the receiving port, socketNum is the sending port///////////
Client inClient;
String dataIn;
int inPort = 1103;
String ip = "192.168.2.20";
Server inServer;
Server outServer;
boolean myServerRunning;
Client outClient;
int outPort = 1001;


////////step is the resolution of the kinect, slack is the distance of error before the send function erases the stack////////////////
int resolution = 2;
float slack = 20;

float zRot;
float xRot;
float yRot;
Vec3D worldAdjust;


// gui stuff ////////////////////////////////////////////////////////////

ControlP5 controlP5;
PMatrix3D currCameraMatrix;
PGraphics3D g3;


boolean showAgents = false;
boolean showTrail = false;

public float separation=0.5;    // separation default value = 0.5
public float alignment=0.2; 
public float cohesion=0.2;    // separation default value = 0.2
public float seekTrail_threshold=1;
public float velocity=0.5;



public float life1 =500;
public float tLength = 5;


////// camera stuff ////////////////////////////////////////////////////

//PeasyCam cam;
Camera camera1;

void setup() {
  
  
  size(1280, 720, P3D);
  g3 = (PGraphics3D)g;
  //frameRate(10);
  smooth();
//  cam = new PeasyCam(this,100);
//  cam.setRotations(-PI/2, -PI/6, 0); 

  controlP5 = new ControlP5(this);
  controlP5.setAutoDraw(false);

// toggles 

controlP5.addToggle("showAgents")
 .setPosition(20,300);
controlP5.addToggle("showTrail")
 .setPosition(20,340); 


 
  
// agent stuff

  controlP5.addSlider("separation",0,5,1,20,20,100,10);
  controlP5.addSlider("cohesion",0,1,0.2,20,40,100,10);
  controlP5.addSlider("alignment",0,1,0.2,20,60,100,10);
  controlP5.addSlider("seekTrail_threshold",0,5,1,20,80,100,10);
  controlP5.addSlider("velocity",0,10,0.5,20,100,100,10);

//trail stuff  
  controlP5.addSlider("life1",0,1000,500,20,140,100,10);
  controlP5.addSlider("tLength",0,20,5,20,160,100,10);
  
  camera1 = new Camera(this, 24, -986, 39, -120, -1447, -96);
  //camera1 = new Camera(this, -50, -1920.814, 10.428);
  camera1.roll(-0.9);
  
  
  ////////////////////////// dummy point import  //////////////////////////////// 
  picTake = 0;
  captureList = new ArrayList();
  avPts = new ArrayList();
  
  
 // for some reason when I turn on import point function in the import class i get a unexpected token: void in the agent class 
 // importPts("pts_124.txt");
   
  pts = new ArrayList();
  matter = new ArrayList();
  frameRate(24);

// INITIATE SPACE ARRAY

volume = new VolumetricSpaceArray (scale,GRID,GRID,GRID); // volumetric spacearray passes the information into the isosurface
surface = new ArrayIsoSurface(volume); //      
brush = new RoundBrush(volume,scale.x/2); // size of  painting information in the volumetric space array

  agentList = new ArrayList();
  trailList = new ArrayList();
  attList = new ArrayList();


  meshList = new ArrayList();
  mesh = new WETriangleMesh();
  stlImport("dummyFile2.stl");

  dumList = new ArrayList();

  //specify kinect position relative to world origin here
  worldAdjust = new Vec3D(-146.1, -551.46, -225.11);
  zRot = radians(-0.17084); // PI/2 aligns with world x Axis,  so y axis is PI
  xRot = radians(-90.5403);
  yRot = radians(1.26946);
  ///////end kinectStuff//////////////

  gfx = new ToxiclibsSupport(this);


  //  for(int i = 0; i < 20; i++){
  //    Vec3D pos = new Vec3D(random(width), random(width), random(width));
  //    attractor newA = new attractor(pos);
  //    attList.add(newA);
  //  }


  littleGuy = new robot();


  pathList = new ArrayList();
  
    
}



void draw() {

  scale(-1, 1, 1);

  lights();
  background(0);
  
 
  camera1.feed();
  // trying to fix the rotated camera tried tilt as well
  //camera1.roll(radians(1.57079633));
  
  
  
  if (frameCount%200 == 0) {
    agentList = new ArrayList();
    for (Vertex v : mesh.getVertices()) {
      if (random(10) < 5) {
        float rad = random(5, 10);
        Vec3D newV = new Vec3D(v.x, v.y, v.z);
        agent newA = new agent(rad, newV);
        agentList.add(newA);
      }
    }
    
    
    
  }

//////////////  capture list  ////////////////////////////////////
//  for(int i = 0; i < captureList.size(); i++){
//    colPt cp = (colPt) captureList.get(i);
//    cp.render();
//  }


  for (int i = 0; i < agentList.size(); i++) {
    agent a = (agent) agentList.get(i);
    a.update();
    a.render();
  }


  //  arraylist to bring in arrays tail class 

  for (int i = 0; i < trailList.size(); i++) {
    trail b = (trail) trailList.get(i);
    b.renderT();
  }

for(int i = 0; i < attList.size(); i++){
   attractor a = (attractor) attList.get(i);
   a.render();
  }


  if (frameCount>20) {
    //update Kinect input

    littleGuy.dummyUpdate();
    if (littleGuy.realPos.distanceTo(littleGuy.objective) < 10) {



      littleGuy.protocol();
    }
    littleGuy.render();
    
    
  noLights();
  
  } 


  noStroke();
  //fill(0,100,255);
  //gfx.mesh(mesh);
  pushMatrix();
  //translate(-174,-1396, -95.99);
  // box(200);
  popMatrix();
  stroke(0, 255, 0);
  strokeWeight(2);
  line(0, 0, 0, 100);
  //stroke(255, 0, 0);
  line(0, 0, 100, 0);
  
  gui();
  
}

void gui() {
  currCameraMatrix = new PMatrix3D(g3.camera);
  camera();
  //fill(10); 
  noStroke();
 // camera1.circle(radians(0) / 2.0);
 // camera1.roll(radians(0) / 2.0);
  //rect(0,0,width,80);
  controlP5.draw();
  g3.camera = currCameraMatrix;
}

void keyPressed() {
  if (keyPressed == true) {
    saveFrame("agent_CodeImages-####.png");
  }
}


void rotateMesh() {
  Collection vList = mesh.getVertices();
  Object[] vArray =  vList.toArray();
  Vec3D rCent = new Vec3D(-174, -1396, -95.99);
  meshList = new ArrayList();
  println("rotating");
  for (int i = 0; i < vArray.length; i++) {

    Vertex v1 = (Vertex) vArray[i];
    Vec3D tV = new Vec3D(v1.x, v1.y, v1.z);
    tV.subSelf(rCent);
    float mag = tV.magnitude();
    tV.normalize();
    tV.rotateZ(PI/10);
    tV.normalize();
    tV.scaleSelf(mag);
    tV.addSelf(rCent);
    v1.x = tV.x;
    v1.y = tV.y;
    v1.z = tV.z;
  }
  for (Face f : mesh.getFaces ()) {
    Triangle3D newM = f.toTriangle();
    meshList.add(newM);
  }
}





