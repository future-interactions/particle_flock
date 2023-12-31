

let flock;
let pCount = 0;//particle count
let pCountX = 40;
let pCountY;
let pSize = 50; //particle size
let pSeparation = 10;
let simGo = true;
let DMSans, suisseMono;
let ballSizeSlider, ballCountSlider;
let xOffset = 0;
let saveCalled = false;
let frameCount = 0;
function preload() {
  DMSans = loadFont('assets/DMSans-Medium.ttf');
  suisseMono = loadFont('assets/SuisseIntlMono-Regular.otf');

}

function setup() {
  createCanvas(windowWidth, windowHeight);
  pixelDensity(1);
  pCountX = pCount;
  pCountY = pCount * (height / width);
  resetSketch();
  // drawInterface();
  colorMode(RGBA);
}

function draw() {
background(0,200,0,10);
  // pSize = ballSizeSlider.value();
  // pCount = ballCountSlider.value();
  pSize = 200;
  pCount = 20;
  pCountX = pCount;
  pCountY = pCount * (height / width);
  flock.run();

  // console.log(pSize);
  // if (saveCalled && frameCount < 2400) {
  //   frameRate(5);
  //   saveCanvas('test-output' + frameCount, 'png');
  // frameCount++;
  // }
}

// Add a new boid into the System
function mouseDragged() {
  flock.addBoid(new Boid(mouseX, mouseY));
}

// Approach modified from The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function () {


  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually

  }
}

Flock.prototype.addBoid = function (b) {
  this.boids.push(b);
}


// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = pSize / 2;
  this.maxspeed = 5;    // Maximum speed
  this.maxforce = 0.2; // Maximum steering force
}

Boid.prototype.run = function (boids) {
  this.flock(boids);
  this.r = pSize / 2;

  this.update();
  this.borders();
  this.render();
}

Boid.prototype.applyForce = function (force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function (boids) {
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion
  // Arbitrarily weight these forces
  sep.mult(1.5);
  ali.mult(1.5);
  coh.mult(1.0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function () {
  if (simGo) {
    // Update velocity
    this.velocity.add(this.acceleration);

    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
  }
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function (target) {
  let desired = p5.Vector.sub(target, this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired, this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function () {
  fill(255,0, 0);
  noStroke();
  push();
  translate(this.position.x, this.position.y);
  ellipse(0, 0, -this.r / 2);
  pop();
  fill(255);
  noStroke();
  push();
  translate(this.position.x-pSize*0.05, this.position.y-pSize*0.01);
  ellipse(0, 0, -this.r * 0.175);
  pop();
  push();
  translate(this.position.x+pSize*0.05, this.position.y-pSize*0.01);
  ellipse(0, 0, -this.r * 0.175);
  pop();
  fill(0);
  noStroke();
  push();
  translate(this.position.x-pSize*0.05, this.position.y-pSize*0.01);
  ellipse(0, 0, -this.r * 0.05);
  pop();
  push();
  translate(this.position.x+pSize*0.05, this.position.y-pSize*0.01);
  ellipse(0, 0, -this.r * 0.05);
  pop();
}

// Wraparound
Boid.prototype.borders = function () {
  if (this.position.x < -this.r) this.position.x = width + this.r;
  if (this.position.y < -this.r) this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function (boids) {
  let desiredseparation = pSize * 10;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position, boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
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

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function (boids) {
  let neighbordist = pSize * 5;
  let sum = createVector(0, 0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position, boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    let steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function (boids) {
  let neighbordist = pSize * 500;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position, boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location

      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}

function resetSketch() {
  flock = new Flock();
  for (let i = 0; i < pCount + 1; i++) {
    for (let j = 0; j < pCountY + 1; j++) {
      if (j % 2 == 0) {
        xOffset = -100;
      } else {
        xOffset = 0;
      }
      let b = new Boid(random(width), random(height));

      flock.addBoid(b);
    }
  }
}

function keyPressed() {
  if (keyCode == RETURN) {
    if (simGo) {
      simGo = false;
    } else {
      simGo = true;
    }
  }
  if (keyCode === 82) {
    // resetSketch();
  }
  if (keyCode === 83 && !saveCalled) {
    saveCalled = true;
  }

}


function drawInterface() {
  textFont(DMSans);
  ballSizeSlider = createSlider(1, 200, 50);
  ballSizeSlider.position(10, 16);
  ballSizeSlider.style('width', '160px');


  let ballSizeText = createElement('desc', 'Particle Size');
  ballSizeText.style('color', '#ffffff');
  ballSizeText.position(16, 36);

  ballCountSlider = createSlider(0, 70, 12);
  ballCountSlider.position(200, 16);
  ballCountSlider.style('width', '160px');

  let ballCountText = createElement('desc', 'Particle Count');
  ballCountText.style('color', '#ffffff');
  ballCountText.position(206, 36);
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
}