import oscP5.*;
import netP5.*;
import org.openkinect.*;
import org.openkinect.processing.*;
import ddf.minim.*;
import peasy.*;
import java.util.Vector;
import toxi.geom.*;

Minim minim;
AudioInput player;

//Size of winow
final int[] SIZE = {
  1600, 1200
};


// Kinect Library object
Kinect kinect;

//Camera variables
float rotationY = 1.0;
float scal = 1.0;
PVector rot = new PVector();
PVector tran = new PVector(width/2+40, height/2+40, -50);
PeasyCam cam;
Vec3D globalOffset, avg, cameraCenter;
float camD0;
float camDMax;
float camDMin;
float audioFactor = 1;

//OSC input var
PVector touch;
PVector pos;

//Depth thresholds
float leftThresh = 0;
float rightThresh = 0;
boolean applyLeftThresh = false;
boolean applyRightThresh = false;
float MAX_THRESH = 255;
boolean useAccl = false;


// Size of kinect image
int w = 640;
int h = 480;

//Touch OSC
OscP5 oscP5;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  size(SIZE[0], SIZE[1], P3D);
  minim = new Minim(this);
  player = minim.getLineIn(Minim.STEREO, 512);
  kinect = new Kinect(this);
  kinect.start();
  kinect.enableDepth(true);
  // We don't need the grayscale image in this example
  // so this makes it more efficient
  kinect.processDepthImage(false);
  /* start oscP5, listening for incoming messages at port 8000 */
  oscP5 = new OscP5(this, 8001);

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  //noStroke();
  // println(gl);
  camD0 = 1400;
  camDMax = 10000;
  camDMin = -10000;
  cam = new PeasyCam(this, camD0);
  cam.setDistance(camD0);
  cam.setMinimumDistance(camDMin);
  cam.setMaximumDistance(camDMax);
  cameraCenter = new Vec3D();
  avg = new Vec3D();
  globalOffset = new Vec3D(0, 0, 0);//1.f / 5, 2.f / 3);
  touch = new PVector();
  pos = new PVector();
}

void oscEvent(OscMessage theOscMessage) {

  String addr = theOscMessage.addrPattern();

  if (addr.equals("/1/fader1")) {
    float  val  = theOscMessage.get(0).floatValue();
    leftThresh = map(val, 0, 1, 0, MAX_THRESH);
    println("leftThresh:" + leftThresh);
  }
  if (addr.equals("/1/toggle1")) {
    applyLeftThresh = !applyLeftThresh;
    println("applyLeftThresh: " + applyLeftThresh);
  }
  if (addr.equals("/1/toggle2")) {
    applyRightThresh = !applyRightThresh;
    println("applyRightThresh:" + applyRightThresh);
  }
  if (addr.equals("/1/toggle4")) {
    useAccl = !useAccl;
    println("useAccl:" + useAccl);
  }
  if (addr.equals("/1/fader2")) {
    float  val  = theOscMessage.get(0).floatValue();
    println(val);
    rightThresh = map(val, 0, 1, 0, MAX_THRESH);
    println("rightThresh:" + rightThresh);
  }
  if (addr.equals("/1/fader3")) {
    float  val  = theOscMessage.get(0).floatValue();
    println(val);
    MAX_THRESH = map(val, 0, 1, 0, 60000);
    println("max thresh:" + MAX_THRESH);
  }
  if (addr.equals("/1/fader5")) {
    float  val  = theOscMessage.get(0).floatValue();
    tran.z = map(val, 0, 1, camDMin, camDMax);
    println("tran.z "+tran.z);
  }
  if (addr.equals("/1/fader4")) {
    float  val  = theOscMessage.get(0).floatValue();
    audioFactor = map(val, 0, 1, 0, 3);
    println("audioFactor "+audioFactor);
  }
  if(addr.equals("/3/xy")) {
    float y = theOscMessage.get(0).floatValue();
    float x = theOscMessage.get(1).floatValue();
    tran.x = map(x,0,1,-3000,3000);
    tran.y = map(y,0,1,-3000,3000);
    println(x+", "+y);
  }
  if (addr.equals("/1/reset")) {
    float  val  = theOscMessage.get(0).floatValue();
    println("reset");
    if (val == 1) {
      println("reset!!!!");
      rot.x = 0;
      rot.y =0;
      rot.z =0;
      scal = 1;
      tran.x = SIZE[0]/2 + 40;
      tran.y = SIZE[1]/2 + 40;
      tran.z = -50;
    }
  }
  if (addr.equals("/accxyz")) {
    if (useAccl) {
      float x = theOscMessage.get(0).floatValue()*.002;
      float y = theOscMessage.get(1).floatValue()*.002;
      float z = theOscMessage.get(2).floatValue()*.002;
      //    x = map(x, -1.5, 1.5, 
      if ( abs(x) > .003)
        rot.y += x;
      if ( abs(y) > .003)
        rot.x += y;
      if ( abs(z) > .02)
        rot.z += z;
      println(rot);
    }
  }
  else {
    println(addr);
  }
}

void draw() {
  background(0);
  lights();
  fill(255);
  //textMode(SCREEN);
  //text("Kinect FR: " + (int)kinect.getDepthFPS() + "\nProcessing FR: " + (int)frameRate,10,16);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 4;

  // Translate and rotate
  cam.rotateX(rot.x);
  rot.x = 0;
  cam.rotateY(rot.y);
  rot.y = 0;
  cam.rotateY(rot.z);
  rot.z = 0;
  cam.setDistance(tran.z);
  translate(tran.x,tran.y,0);
  int buf = 0;
  for (int x=0; x<w; x+=skip) {
    for (int y=0; y<h; y+=skip) {
      int offset = x+y*w;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);

      stroke(255);
      float factor = 500;
      if (buf < player.bufferSize() - 1 ) {
        buf++;
      }
      else {
        buf = 0;
      }
      shapeMode(CENTER);
      beginShape(POINT);
      float rightChannel = player.right.get(buf) * audioFactor;
      float leftChannel = player.left.get(buf) * audioFactor;
      float rc = rightChannel;
      float lc = leftChannel;
      rightChannel = map(rightChannel, -1, 1, 0,255);
      leftChannel = map(leftChannel, -1, 1, 0, 255);
      float r = rightChannel + 80;
      float g = rightChannel * .8;
      float b = 255 - rightChannel * .73;
      float z = (rawDepth * rc);
      if (applyRightThresh && z < rightThresh) {
        continue;
      }
      else {
        stroke(r, g, b);
        vertex(v.x*factor, v.y*factor,  -1*z + -2000 );
      }
      r = 255 - leftChannel;
      g = leftChannel;
      b = leftChannel * .3;
      z = rawDepth * lc;
      if (applyLeftThresh && z  < leftThresh) {
        continue;
      }
      else {
        stroke(r, g, b);
        vertex(v.x*factor, v.y*factor, z);
      }
      endShape(CLOSE);
      //popMatrix();
    }
  }
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void keyPressed() {
  if (keyCode == LEFT) {
    rotationY -= .5;
    println("rotationy: "+rotationY);
  }
  else if (keyCode == RIGHT) {
    rotationY += .5;
    println("rotationy: "+rotationY);
  }
  if (keyCode == CONTROL) {
    rot.x = mouseX;//(x - mouseX)+ x;
    rot.y = mouseY;//(y - mouseY)+ y;
  }
  else if ( key == 'p' ) {
    print("playing...\n");
    //player.play();
  }
  else if (keyCode == SHIFT) {
    rot.z =  mouseY;//(z - mouseY)+ z;
  }
  else if (keyCode == ALT) {
    tran.x = mouseX;
    tran.y = mouseY;
    println("translate at: "+tran.x+", "+tran.y);
  }
  else if (key == 'a') {
    tran.x -= 10;
    println("tran.x :: "+tran.x);
  }
  else if (key == 'd') {
    tran.x += 10;
    println("tran.x :: "+tran.x);
  }
  else if (key == 'w') {
    tran.y += 10;
    println("tran.y :: "+tran.y);
  }
  else if (key == 's') {
    tran.y -= 10;
    println("tran.y :: "+tran.y);
  }
  else if (key == '=') {
    tran.z += 10;
    println("tran.z :: "+tran.z);
  }
  else if (key == '-') {
    tran.z -= 10;
    println("tran.z :: "+tran.z);
  }
}

void stop() {
  kinect.quit();
  // always close Minim audio classes when you are done with them
  player.close();
  // always stop Minim before exiting
  minim.stop();
  super.stop();
}

