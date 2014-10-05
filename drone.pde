class Drone {
 
  PVector location;
  PVector velocity;
  PVector acceleration;
  float topspeed;
  float mass;
 
  Drone() {
    location = new PVector(0,0,0);
    velocity = new PVector(0,0,0);
    acceleration = new PVector(0,0,0);
    topspeed = 100;
    mass = 1;
  }
 
  void update() {
    drag();
    velocity.add(acceleration);
    velocity.limit(topspeed);
    location.add(velocity);
    acceleration.mult(0);
  }
 
  void display() {
    stroke(0);
    fill(255,0,0);
    ellipse(width/2+location.x/10,height/2+location.y/10,16+location.z/10,16+location.z/10);
  }
  
  void drag() {
     float c = 0.5;
     float speed = velocity.mag();
     float dragMagnitude = c * speed * speed;
     PVector drag = velocity.get();
     drag.mult(-1);
     drag.normalize();
     drag.mult(dragMagnitude);

     applyForce(drag); 
  }
  void applyForce(PVector force) {
    PVector f = PVector.div(force,mass);
    acceleration.add(f);
  }
}
