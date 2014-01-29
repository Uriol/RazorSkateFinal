
// Sensivity and interval
float S = 0.00390625;
float time = 0.02;
float q[4],  rx, ry, rz, gx, gy, gz, cx, cy, cz, qxc[4], qconj[4], trueacc[4];
float _yaw,_pitch, _roll;

// timer
int counter = 0;
boolean start = false;

float initialYaw;
float totalAngleDifference;
float previousYaw;
float angleDifference;

// Speed
float totalSpeed = 1;
float speedX = totalSpeed; 
float speedY = 0;

// Positions
float finalPosX, initialPosX;
float finalPosY, initialPosY;


// booleans
boolean jumping = false;
boolean landing = false;

// Jumping accels
const int numZaccels = 5;
float z_accels[numZaccels];
int indexZ = 0;

boolean z_accel_jumping = false;

// Calculate jump
float airtime = 0;
float jumpDegree = 0.785398163;
float speedZ = 0;
float initialPosZ, finalPosZ;
float jumpSpeed = 1;
float jumpSpeedNow;


// Yaw on jumping
float initialYaw_onJumping, initialYaw_onJumpingToZero;

float yawOnLanding;
float rotationAir;


// compensate yaw after 180s
boolean minus180 = false;
boolean plus180 = false;

void output_angles()
{ 
 
  yaw = TO_DEG(yaw);
   Serial.println(yaw);

  // Serial.print("YPR : ");Serial.print(yaw);Serial.print(" ,");Serial.print(TO_DEG(pitch));Serial.print(" ,"); Serial.print(TO_DEG(roll));Serial.println(" ,");
  
  // Get accel in g
  rx = accel[0] * S;
  ry = accel[1] * S;
  rz = accel[2] * S;
//  Serial.print("#RAW-"); Serial.print('=');
//  Serial.print(rx); Serial.print(",");
//  Serial.print(ry); Serial.print(",");
 // Serial.print(rz); Serial.println();
  // Get quaternions data
  float c1 = cos(yaw/2);
  float c2 = cos(pitch/2);
  float c3 = cos(roll/2);
  float s1 = sin(yaw/2);
  float s2 = sin(pitch/2);
  float s3 = sin(roll/2);
  q[0] = c1*c2*c3 - s1*s2*s3; // w
  q[1] = c1*s2*c3 - s1*c2*s3; // z
  q[2] = s1*s2*c3 + c1*c2*s3; // x
  q[3] = s1*c2*c3 + c1*s2*s3; // y
  // Get Gravity 
  gy = 2 * (q[1] * q[3] - q[0] * q[2]);
  gx = 2 * (q[0] * q[1] + q[2] * q[3]);
  gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//  Serial.print("#GRAVITY-");  
//  Serial.print('=');
//  Serial.print(gx); 
//  Serial.print(",");
//  Serial.print(gy); 
//  Serial.print(",");
//  Serial.print(gz); 
//  Serial.println();
  cx = rx + gx;
  cy = ry + gy;
  cz = rz - gz;
//  Serial.print("#GRAVITY COMPENSATED-"); Serial.print('=');
//  Serial.print(cx); Serial.print(",");
//  Serial.print(cy); Serial.print(",");
//  Serial.print(cz); Serial.println();
  
 
 
 // Timer 
  if (start == false){
    counter += 20;
    initialYaw = yaw;
    previousYaw = yaw;
  }

  if (counter >= 6000) {
    start = true;
    //Serial.print(start);
  }
   
  // Start everything on start = true 
  if (start == true) {
    if (jumping == false) {
      on_the_ground();
    }
    in_the_air();
    
  }
  
}

void on_the_ground(){ // Calculates speed X,Y and positions X,Y ------------------------------------------------------------------------------
    
  
 // Serial.println(yaw);
  // Restart airtime value and speedz
  airtime = 0;
  speedZ = 0;
  
  if ( plus180 == true ) { yaw = yaw + 180; }
 if ( minus180 == true ) { yaw = yaw - 180; } 
  
  
  
  angleDifference = yaw - previousYaw;
  totalAngleDifference = yaw - initialYaw;
   Serial.print("totalAngleDifference");
   Serial.println(totalAngleDifference);
  // to radians
  totalAngleDifference = totalAngleDifference*M_PI/180;
//  Serial.print("totalAngleDifference----------");
//  Serial.println(totalAngleDifference);
  
  
  
  
  previousYaw = yaw;
  speedX = totalSpeed*cos(totalAngleDifference);
  speedY = totalSpeed*sin(totalAngleDifference);
  
  // If angle difference is big enough
//  if ( angleDifference >= 0.01 || angleDifference <= -0.01) {
//    
//    speedX = totalSpeed*cos(totalAngleDifference);
//    speedY = totalSpeed*sin(totalAngleDifference);
//    
//  }
   
  
  Serial.print("speedX"); Serial.println(speedX);
  Serial.print("speedY"); Serial.println(speedY);
   
  // Calculate landing direction ---------------------------------------------
  if (landing == true ) {
    calculateLanding(); 
    totalAngleDifference = yawOnLanding - initialYaw;
   Serial.print("totalAngleDifference");
   Serial.println(totalAngleDifference);
  }
  
  
  
  // Calculate x position
  finalPosX = initialPosX + speedX*time;
  //Serial.print("finalPosX"); Serial.println(finalPosX);
  
  // Calculate y position
  finalPosY = initialPosY + speedY*time;
 // Serial.print("finalPosY"); Serial.println(finalPosY);
  
  // Restart values
  initialPosX = finalPosX;
  initialPosY = finalPosY;
  
  
}


void in_the_air(){ // while jumping calculates : time in the air, pos X,Y. First jump, initial yaw on jumping
  
  // if cz is very small set to 0
  // Serial.print("real cz : "); Serial.println(cz);
  if (cz <= 0.35 && cz >= -0.35) {
    cz = 0;
    if ( jumping == true ) {landing = true;}
    jumping = false;
    
    checkPreviousAccelsZ();
    
  } else if (cz >= 0.35 || cz <= -0.35) {
    // calculate initial speed on the first moment of the jump
    if ( jumping == false ) {Serial.println(" First JUMP "); initialJumpSpeed(); calculateJump(); initialYaw_onJumping = yaw; 
   

    Serial.print( "initialYaw_onJumping "); Serial.println(initialYaw_onJumping);
    
    
  }
    jumping = true;
    landing = false;
  } 
  
  //Serial.print("cz compensated : "); Serial.println(cz);
  
  
  
  // Store last 5 accels
  if ( indexZ < 4 ) {
    z_accels[indexZ] = cz;
    indexZ = indexZ + 1;
  } // if the array is full
  else {
    z_accels[0] = z_accels[1];
    z_accels[1] = z_accels[2];
    z_accels[2] = z_accels[3];
    z_accels[3] = z_accels[4];
    z_accels[4] = cz;
  }
  
  // If it is jumping calculate jump
  if ( jumping == true ) { Serial.println("Jumping --------------------------- ");  calculateJump(); }
  
  Serial.print("speedX"); Serial.println(speedX);
  Serial.print("speedY"); Serial.println(speedY);
  
  // Calculate x position
  finalPosX = initialPosX + speedX*time;
 // Serial.print("finalPosX"); Serial.println(finalPosX);
  
  // Calculate y position
  finalPosY = initialPosY + speedY*time;
//  Serial.print("finalPosY"); Serial.println(finalPosY);
  
  // Restart values
  initialPosX = finalPosX;
  initialPosY = finalPosY;
   
}


void checkPreviousAccelsZ(){ // check if it stills in the air 
  // Check values of last 5 accels to check if it still jumping
  if ( z_accels[0] >= 0.35 || z_accels[0] <= -0.35 && cz == 0) { z_accel_jumping = true; }
  if ( z_accels[1] >= 0.35 || z_accels[1] <= -0.35 && cz == 0) { z_accel_jumping = true; }
  if ( z_accels[2] >= 0.35 || z_accels[2] <= -0.35 && cz == 0) { z_accel_jumping = true; }
  if ( z_accels[3] >= 0.35 || z_accels[3] <= -0.35 && cz == 0) { z_accel_jumping = true; }
  if ( z_accels[4] >= 0.35 || z_accels[4] <= -0.35 && cz == 0) { z_accel_jumping = true; }
  
  if ( z_accel_jumping == true ) { 
   // Serial.println(" Still jumping ");
    jumping = true;
    z_accel_jumping = false;
  } else { z_accel_jumping = false; }
}





// Calculate landing direction
void calculateLanding(){ // Calculate yaw on landing and check for 180s
 // Serial.println("Landing");
  plus180 = false;
  minus180 = false;
  landing = false;
  yawOnLanding = yaw ;
  Serial.print( "yawOnLanding "); Serial.println(yawOnLanding);
  
  
  // Calculate 180s
  // first case initialYaw < 0 > 90
  if ( initialYaw_onJumping > 0 && initialYaw_onJumping < 90 ) {
    if ( -90 + initialYaw_onJumping < yawOnLanding && 90 + initialYaw_onJumping > yawOnLanding) { yawOnLanding = yawOnLanding; Serial.print( "case1: final yawOnLanding not 180 "); Serial.println(yawOnLanding); }
    else if ( 90 + initialYaw_onJumping < yawOnLanding && yawOnLanding < 179 ) { yawOnLanding = yawOnLanding - 180; Serial.print( "case1: final yawOnLanding "); Serial.println(yawOnLanding); minus180 = true; }
    else if ( -90 + initialYaw_onJumping > yawOnLanding && yawOnLanding > -179) {yawOnLanding = yawOnLanding + 180; Serial.print( "case1: final yawOnLanding "); Serial.println(yawOnLanding); plus180 = true; }
   
  }
  
  // second case initialYaw > 90 < 179
  if ( initialYaw_onJumping > 90 && initialYaw_onJumping < 179 ) {
     
    if ( yawOnLanding > 0 && 90 - ( 179 - initialYaw_onJumping ) > yawOnLanding ) { yawOnLanding = yawOnLanding - 180; Serial.print( "case2: final yawOnLanding "); Serial.println(yawOnLanding); minus180 = true; }
    else if ( -90 - ( 179 - initialYaw_onJumping ) < yawOnLanding && yawOnLanding < 0 ) { yawOnLanding = yawOnLanding + 180; Serial.print( "case2: final yawOnLanding "); Serial.println(yawOnLanding);  plus180 = true;}
    else { yawOnLanding = yawOnLanding; Serial.print( "case2: final yawOnLanding not 180 "); Serial.println(yawOnLanding); }
    
  }
  
  
  // Third case initial yaw < 0 > -90
  if ( initialYaw_onJumping < 0 && initialYaw_onJumping > -90 ) {
    
    if ( yawOnLanding > -179 && yawOnLanding < -90 + initialYaw_onJumping ) {  yawOnLanding = yawOnLanding + 180; Serial.print( "case3: final yawOnLanding "); Serial.println(yawOnLanding); plus180 = true;}
    else if ( 90 + initialYaw_onJumping < yawOnLanding && yawOnLanding < 179 ) { yawOnLanding = yawOnLanding - 180; Serial.print( "case3: final yawOnLanding "); Serial.println(yawOnLanding); minus180 = true; }
    else  { yawOnLanding = yawOnLanding; Serial.print( "case3: final yawOnLanding not 180 "); Serial.println(yawOnLanding); }
    
  }
  
  
  // fourth case initial yaw > -90 < -179
  if ( initialYaw_onJumping < -90 && initialYaw_onJumping > -179 ) {
    
    if ( yawOnLanding > 0 && 90 + ( 179 + initialYaw_onJumping) > yawOnLanding ) { yawOnLanding = yawOnLanding - 180; Serial.print( "case4: final yawOnLanding "); Serial.println(yawOnLanding); minus180 = true; }
    else if ( -90 + (179 + initialYaw_onJumping ) < yawOnLanding && yawOnLanding < 0 ) { yawOnLanding = yawOnLanding + 180; Serial.print( "case4: final yawOnLanding "); Serial.println(yawOnLanding); plus180 = true;}
    else { yawOnLanding = yawOnLanding; Serial.print( "case4: final yawOnLanding not 180 "); Serial.println(yawOnLanding); }
   
  }
  
  
  
  
}
















// not implemented yet -------------------------------------------------


// Calculate initial speed of the jump
void initialJumpSpeed(){
  speedZ = jumpSpeed*sin(jumpDegree);
  //Serial.print("speedZ :"); Serial.println(speedZ);
}

// if jumping == true calculate the jump
void calculateJump(){ 
  airtime += 0.02;
 // Serial.print("airtime :"); Serial.println(airtime);
 // Serial.print("speedZ :"); Serial.println(speedZ);
  
  
  jumpSpeedNow = speedZ-9.8*airtime;
  //Serial.print("jumpSpeedNow :"); Serial.println(jumpSpeedNow);
  // Calculate position Z
 finalPosZ = initialPosZ + speedZ*airtime - 9.8*airtime*airtime*0.5;
// Serial.print("finalPosZ :"); Serial.println(finalPosZ);
 
 initialPosZ = finalPosZ;
  
}







