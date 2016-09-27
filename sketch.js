movers = [];
vals = [];
vals2 = [];
mm = 100;
level = 0;
areas = [];
a1 = 0;
var s;
var a;
var c;
var sMax = 5;
var aMax = 5;
var cMax = 5;

function setup() {
  var cnv = createCanvas(windowWidth - 100, windowHeight);
  cnv.position(100, 0);
  newMovers();
  newAreas();
  sepSlider = createSlider(0, 100, 0);
  sepSlider.position(50, 100);
  cohSlider = createSlider(0, 100, 0);
  cohSlider.position(100, 250);
  aliSlider = createSlider(0, 100, 0);
  aliSlider.position(100, 350);
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight)
}

function newAreas() {
  level++;
  for (i = 0; i < ((width / 100) - level); i++) {
    vals[i] = random(((i - 1) * 100) + 150, i * 100);
    vals2[i] = random(height);
  }
  for (i = 0; i < ((width / 100) - level); i++) {
    areas[i] = new Area();
    areas[i].pos = createVector(vals[i], vals2[i]);
  }
}

function Area() {
  this.pos = 0;
}

function newAreasRun() {
  for (i = 1; i < ((width / 100) - level); i++) {
    fill(255);
    ellipse(vals[i], vals2[i], 50 * (1 - (level / 10)), 50 * (1 - (level / 10)));
  }
}

function newMovers() {
  for (i = 0; i < mm; i++) {
    movers[i] = new Boid()
    movers[i].pos = createVector(random(600), random(600));
    movers[i].vel = createVector(random(10), random(10));
    movers[i].acc = createVector(0, 0);
    movers[i].col = color(random(254), random(254), random(254));
    movers[i].r = 10;
    movers[i].maxSpeed = 2;
    movers[i].maxForce = random(1, 3);
  }
  repeller = new Repeller();
  repeller.pos = createVector(random(600), random(600));
  repeller.r = random(30, 100);
  repeller.col = color(random(255), random(255), random(255));
  repeller.maxForce = 30;
}

function runMovers() {
  for (i = 0; i < mm; i++) {
    movers[i].run();
  }
  repeller.update();
  repeller.render();
}

function draw() {
  background(0);
  s = map(sepSlider.value(), 0, 100, 0, sMax);
  a = map(aliSlider.value(), 0, 100, 0, aMax);
  c = map(cohSlider.value(), 0, 100, 0, cMax);
  newAreasRun();
  runMovers();
  if (movers.length === 0) {
    newAreas(level);
  }
}

function displayText(){
  textSize(24);
  fill(10, 20, 100);
  text("Separation = " + floor(s*100), 20, 150);
  text("Cohesion = " + cohSlider.value() + "% of " + cMax, 20, 250)
  text("Alignment = " + aliSlider.value() + "% of " + aMax, 20, 350);
  
}

Mover.prototype.flock = function(movers) {
  var sep = this.separate(movers); // Separation
  var ali = this.align(movers);    // Alignment
  var coh = this.cohesion(movers); // Cohesion
  // Use sliders weight these forces
  
  sep.mult(s);
  ali.mult(a);
  coh.mult(c);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

Mover.prototype.seek = function(target) {
  var desired = p5.Vector.sub(target, this.pos); // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired, this.velocity);
  steer.limit(this.maxforce); // Limit to maximum steering force
  return steer;
}

Mover.prototype.separate = function(movers) {
  var desiredseparation = 55.0;
  var steer = createVector(0, 0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < movers.length; i++) {
    var d = p5.Vector.dist(this.pos, movers[i].pos);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.pos, movers[i].pos);
      diff.normalize();
      diff.div(d); // Weight by distance
      steer.add(diff);
      count++; // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

Mover.prototype.align = function(movers) {
  var neighbordist = 50;
  var sum = createVector(0, 0);
  var count = 0;
  for (var i = 0; i < movers.length; i++) {
    var d = p5.Vector.dist(this.pos, movers[i].pos);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(movers[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

Mover.prototype.cohesion = function(movers) {
  var neighbordist = 50;
  var sum = createVector(0, 0); // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < movers.length; i++) {
    var d = p5.Vector.dist(this.pos, movers[i].pos);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(movers[i].pos); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum); // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}

function Mover() {
  this.pos = 0;
  this.vel = 0;
  this.acc = 0;
  this.col = 0;
  this.r = 0;
  this.maxSpeed = 0;
  this.maxForce = 0;
}

Mover.prototype.run = function() {
  this.flock(movers)
  this.update();
  this.checkEdges();
  this.render();
}

Mover.prototype.update = function() {
  this.applyForce = function(force) {
    this.acc.add(force);
  }

  this.rpl = new p5.Vector.sub(this.pos, repeller.pos);
  this.rpl.normalize();
  this.rpl.mult(3);

  if (this.pos.dist(repeller.pos) < 150) {
    this.applyForce(this.rpl);
  }

  this.att = new p5.Vector.sub(areas[a1].pos, this.pos)
  this.att.normalize();
  this.att.mult(215);
  this.applyForce(this.att);

  this.atr = new p5.Vector.sub(repeller.pos, this.pos);
  this.atr.normalize();
  this.atr.mult(1);

  if (this.pos.dist(repeller.pos) > 180) {
    this.applyForce(this.atr);
  }

  this.vel.add(this.acc);
  this.vel.limit(this.maxSpeed);
  this.pos.add(this.vel);
  this.acc.mult(0);
}

Mover.prototype.checkEdges = function() {
  if (this.pos.x > width) {
    this.pos.x = 0;
  } else if (this.pos.x < 0) {
    this.pos.x = width;
  }

  if (this.pos.y > height) {
    this.pos.y = 0;
  } else if (this.pos.y < 0) {
    this.pos.y = height;
  }
}

Mover.prototype.render = function() {
  stroke(0);
  fill(this.col);
  ellipse(this.pos.x, this.pos.y, this.r, this.r);
}

Boid.prototype = new Mover();

function Boid() {

}

Repeller.prototype = new Mover();

function Repeller() {
  this.update = function() {
    this.pos.x = mouseX;
    this.pos.y = mouseY;
  }
}

Attractor.prototype = new Mover();

function Attractor() {
  this.update = function() {
    this.applyForce = function(force) {
      this.acc.add(force);
    }
    this.rpl = new p5.Vector.sub(this.pos, repeller.pos);
    this.rpl.normalize();
    this.rpl.mult(2);

    if (this.pos.dist(repeller.pos) < 100) {
      this.applyForce(this.rpl);
    }

    this.vel.add(this.acc);
    this.vel.limit(this.maxSpeed);
    this.pos.add(this.vel);
    this.acc.mult(0);
  }
}