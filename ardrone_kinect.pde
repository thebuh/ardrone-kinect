import processing.opengl.*;

import com.shigeodayo.ardrone.manager.*;
import com.shigeodayo.ardrone.navdata.*;
import com.shigeodayo.ardrone.utils.*;
import com.shigeodayo.ardrone.processing.*;
import com.shigeodayo.ardrone.command.*;
import com.shigeodayo.ardrone.*;

ARDroneForP5 ardrone;

import SimpleOpenNI.*;

SimpleOpenNI  context;

PImage img;
PFont textFont;
Drone drone;

boolean ardrone_enable = true;
boolean ardrone_engaged = false;
int ardrone_engage_timer = 0;



boolean kinect_enable = true;

void setup() {

  // instantiate a new context
  size(640, 480, P3D); 
  background(0, 0, 0);
  stroke(0, 0, 255);
  strokeWeight(3);
  smooth();
  textFont = createFont("Helvetica Neue", 16);
  textFont(textFont);

  if (kinect_enable) {
    initKinect();
  }
  if (ardrone_enable) {
    initArdrone();
  }
  //init drone object for controls
  //drone = new Drone();
}


void draw() {
  background(0, 0, 0);
  if (kinect_enable) {
    context.update();
    img = context.rgbImage();
    background(img);

    processSkeleton();
  }
  drawInfo();
  /*drone.update();
   drone.display();*/
  if (ardrone_enable) {
    //ardronePosition();
  }
}


void drawInfo() {
  text("FPS: " + (int)frameRate, width-60, 15); 

  text("Velocity:\nx:" + round(drone.velocity.get().x) + "\ny:" + round(drone.velocity.get().y) + "\nz:" + round(drone.velocity.get().z), 0, 16);
  if (ardrone_enable) {
    // print out AR.Drone information
    ardrone.printARDroneInfo();

    // getting sensor information of AR.Drone
    float pitch = ardrone.getPitch();
    float roll = ardrone.getRoll();
    float yaw = ardrone.getYaw();
    float altitude = ardrone.getAltitude();
    float[] velocity = ardrone.getVelocity();
    int battery = ardrone.getBatteryPercentage();

    String attitude = "pitch:" + pitch + "\nroll:" + roll + "\nyaw:" + yaw + "\naltitude:" + altitude;
    text(attitude, 60, 85);
    String vel = "vx:" + velocity[0] + "\nvy:" + velocity[1];
    text(vel, 60, 140);
    String bat = "battery:" + battery + " %";
    text(bat, 60, 170);
  }
}


void initKinect() {
  context = new SimpleOpenNI(this);
  // enable depthMap generation 
  context.enableDepth();
  context.enableRGB();
  // enable skeleton generation for all joints
  context.enableUser();
  context.setMirror(false);
}

void initArdrone() {
  ardrone=new ARDroneForP5("192.168.1.1");
  //AR.Droneに接続，操縦するために必要
  ardrone.connect();
  //AR.Droneからのセンサ情報を取得するために必要
  ardrone.connectNav();
  //AR.Droneからの画像情報を取得するために必要
  ardrone.connectVideo();
  //これを宣言すると上でconnectした3つが使えるようになる．
  ardrone.start();
}

void processSkeleton() {
  int[] userList = context.getUsers();
  for (int i=0; i<userList.length; i++)
  {
    if (context.isTrackingSkeleton(userList[i])) {
      //drawSkeleton(userList[i]);
      processHands(userList[i]);
    }
  }
}

void processHands(int userId) {

  PVector handLeftPosWorld = new PVector();
  PVector handRightPosWorld = new PVector();
  PVector neckPosWorld = new PVector();
  PVector handLeftPos = new PVector(); 
  PVector handRightPos = new PVector();
  PVector neckPos = new PVector();
  float  confidence;
  confidence = context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HAND, handLeftPosWorld);
  if (confidence < 0.001f) 
    // nothing to draw, orientation data is useless
    return;

  confidence = context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HAND, handRightPosWorld);
  if (confidence < 0.001f) 
    // nothing to draw, orientation data is useless
    return;

  confidence = context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_NECK, neckPosWorld);
  if (confidence < 0.001f) 
    // nothing to draw, orientation data is useless
    return;

  context.convertRealWorldToProjective(handLeftPosWorld, handLeftPos);    //left
  context.convertRealWorldToProjective(handRightPosWorld, handRightPos);  //right
  context.convertRealWorldToProjective(neckPosWorld, neckPos);  //neck




  stroke(0);
  fill(175);
  ellipse(handLeftPos.x, handLeftPos.y, 16, 16);

  ellipse(handRightPos.x, handRightPos.y, 16, 16);
  text('L', handLeftPos.x, handLeftPos.y);
  text('R', handRightPos.x, handRightPos.y);

  ellipse(neckPos.x, neckPos.y, 16, 16);

  PVector handsMidPos = PVector.sub(handLeftPos, handRightPos);
  handsMidPos.div(2);

  if (handsMidPos.mag() < 20) {
    ardrone_engage_timer++;
    if (ardrone_engaged && ardrone_engage_timer > 5) {
      ardroneDisengage();
    } else if (!ardrone_engaged && ardrone_engage_timer > 5) {
      ardroneEngage();
    }
  }

  translate(handRightPos.x, handRightPos.y);
  line(0, 0, handsMidPos.x, handsMidPos.y);

  float roll = degrees(handsMidPos.heading());

  int ArRoll = 0;
  if (roll > -170 && roll < -155) { //light left
    //println("light left");
    ArRoll = -5;
  } else if (roll > -155 && roll <= -120) { //hard left
    //println("hard left");
    ArRoll = -10;
  } else if (roll <= 170 && roll > 155) { //light right
    //println("light right");
    ArRoll = 5;
  } else if (roll <= 155 && roll > 120) { //hard right
    //println("hard right");
    ArRoll = 10;
  } else { //steady
    //println("right left steady");
    ArRoll = 0;
  }
  translate(0, 0);
  PVector hdir = PVector.sub(neckPos, handLeftPos);
  PVector dir = PVector.add(handsMidPos, hdir);
  dir.limit(80);
  translate(handsMidPos.x, handsMidPos.y);
  line(0, 0, dir.x, dir.y);

  int softTh = 20;
  int hardTh = 70;

  int ArHeight = 0;
  if (dir.y>softTh && dir.y < hardTh) {
    //println("light up");
    ArHeight = 5;
  } else if (dir.y >= hardTh) {
    //println("hard up");
    ArHeight = 10;
  } else if (dir.y < -1*softTh && dir.y > -1*hardTh) {
    //println("light down");
    ArHeight = -5;
  } else if (dir.y <= -1*hardTh) {
    //println("hard down");
    ArHeight = -10;
  } else {
    //println("up down steady");
    ArHeight = 0;
  }
  int ArFwd = 0;
  if (dir.z>softTh && dir.z < hardTh) {
    //println("light fwd");
    ArFwd = 5;
  } else if (dir.z >= hardTh) {
    //println("hard fwd");
    ArFwd = 10;
  } else if (dir.z < -1*softTh && dir.z > -1*hardTh) {
    //println("light back");
    ArFwd = -5;
  } else if (dir.z <= -1*hardTh) {
    //println("hard back");
    ArFwd = -10;
  } else {
    //println("front back steady");
    ArFwd = 0;
  }
  //println(dir);
  if (ardrone_engaged) {
    String parms = "Front:"+ArFwd+"\n"+"Up:"+ArHeight+"\nLeft:"+ArRoll;
    println(parms);
    //drone.applyForce(new PVector(ArFwd, ArRoll, ArHeight));
    ardrone.move3D(ArFwd, -1*ArRoll, -1*ArHeight, 0);
  }
}

void ardroneEngage() {
  println("Engage!");
  ardrone_engaged = true;
  ardrone_engage_timer = 0;
  if (ardrone_enable) {
    ardrone.takeOff();
  }
}

void ardroneDisengage() {
  println("ABANDON!");
  ardrone_engaged = false;
  ardrone_engage_timer = 0;
  if (ardrone_enable) {
    ardrone.landing();
  }
}

void ardronePosition() {
  ardrone.move3D(round(drone.velocity.get().x), round(drone.velocity.get().y), round(drone.velocity.get().z), 0);
}

// Event-based Methods
void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      drone.applyForce(new PVector(0, -10, 0));
    } else if (keyCode == DOWN) {
      drone.applyForce(new PVector(0, 10, 0));
    } else if (keyCode == LEFT) {
      drone.applyForce(new PVector(-10, 0, 0));
    } else if (keyCode == RIGHT) {
      drone.applyForce(new PVector(10, 0, 0));
    } else if (keyCode == SHIFT) {
      drone.applyForce(new PVector(0, 0, 10));
    } else if (keyCode == CONTROL) {
      drone.applyForce(new PVector(0, 0, -10));
    }
  } else {
    if (keyCode == BACKSPACE) {
      ardroneDisengage();
    } else if (keyCode == ENTER) {
      ardroneEngage();
    }
  }
}

// when a person ('user') enters the field of view
void onNewUser(SimpleOpenNI curContext, int userId)
{
  ardroneDisengage();
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");

  context.startTrackingSkeleton(userId);
}

// when a person ('user') leaves the field of view 
void onLostUser(int userId)
{
  println("User Lost - userId: " + userId);
}

