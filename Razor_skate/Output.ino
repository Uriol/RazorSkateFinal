
// Sensivity and interval
float S = 0.00390625;
float time = 0.02;

// General values 
float q[4],  rx, ry, rz, gx, gy, gz, cx, cy, cz, qxc[4], qconj[4], trueacc[4];
float _yaw,_pitch, _roll;

// X acceleration variables
const int numXaccels = 5;
float x_accels[numXaccels];
int indexX = 0;
int xZeros = 0;

boolean x_accels_Zero = false;
boolean x_accels_One = false;
boolean x_accels_Two = false;
boolean x_accels_Three = false;
boolean x_accels_Four = false;

float initialSpeedX = 0;
float finalSpeedX = 0;

float finalPosX, initialPosX;

boolean XstartedMoving = false;
boolean gravity_x_Off = false;

// Y acceleration variables
const int numYaccels = 5;
float y_accels[numYaccels];
int indexY = 0;
int yZeros = 0;

boolean y_accels_Zero = false;
boolean y_accels_One = false;
boolean y_accels_Two = false;
boolean y_accels_Three = false;
boolean y_accels_Four = false;

float initialSpeedY = 0;
float finalSpeedY = 0;

float finalPosY, initialPosY;

boolean YstartedMoving = false;
boolean gravity_y_Off = false;
float previousRoll = 0;

// Z acceleration variables
const int numZaccels = 5;
float z_accels[numZaccels];
int indexZ = 0;
int zZeros = 0;

boolean z_accels_Zero = false;
boolean z_accels_One = false;
boolean z_accels_Two = false;
boolean z_accels_Three = false;
boolean z_accels_Four = false;

float initialSpeedZ = 0;
float finalSpeedZ = 0;

float finalPosZ, initialPosZ;
float previousZaccel = 0;



void output_angles()
{  
  
//  Serial.print("#GYRO-");  
//  Serial.print('=');
 // Serial.println("gyro[0]"); 
//  Serial.println(gyro[0]);
//  Serial.println("gyro[1]"); 
//  Serial.println(gyro[1]);
////  Serial.print(gyro[2]); 
//  Serial.println();
  
  
  
  
  // Get accel in g
  rx = accel[0] * S;
  ry = accel[1] * S;
  rz = accel[2] * S;
  Serial.print("#RAW-"); Serial.print('=');
  Serial.print(rx); Serial.print(",");
  Serial.print(ry); Serial.print(",");
  Serial.print(rz); Serial.println();
 
//    Serial.print("#A");  Serial.print('=');
//    Serial.print(rx); Serial.print(",");
//    Serial.print(ry); Serial.print(",");
//    Serial.print(rz); Serial.println();
    
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
  Serial.print("#GRAVITY-");  
  Serial.print('=');
  Serial.print(gx); 
  Serial.print(",");
  Serial.print(gy); 
  Serial.print(",");
  Serial.print(gz); 
  Serial.println();


 
  
//  Serial.print("#RAW-"); Serial.print('=');
//  Serial.print(cx); Serial.print(",");
//  Serial.print(cy); Serial.print(",");
//  Serial.print(cz); Serial.println();
  
  // Get ypr from quaternion data
  _yaw = atan2(2*q[3]*q[0]-2*q[2]*q[1], 1-2*q[3]*q[3] - 2*q[1]*q[1]);
  _pitch = asin(2*q[2]*q[3] + 2*q[1]*q[0]);
  _roll = atan2(2*q[2]*q[0]-2*q[3]*q[1], 1-2*q[2]*q[2] - 2*q[1]*q[1]);
  
  _yaw = TO_DEG(_yaw);
  _pitch = TO_DEG(_pitch);
  _roll = TO_DEG(_roll);
//  Serial.print("calculated yaw");
//  Serial.print(_yaw);
//  Serial.println("calculated pitch"); 
// Serial.println(_pitch);
 // Serial.print("calculated roll"); 
 // Serial.print(_roll);
//  Serial.println();
  
  
  
  
  // Calculate accelerations for all angles
  // X edge -------------------------------------------------------------------------------------------------------------
  
  // Calculate Pitch angle
  if (_pitch >= -15 && _pitch <= 15) {
      gravity_x_Off = true;
     // Serial.println(_pitch);
     // Serial.println("pitch with no gravity");
  } else { gravity_x_Off = false; 
  // Serial.println("pitch with gravity"); 
  }
  
  
  // Calculate when x started moving
  if ( XstartedMoving == true ) {
   // Serial.println("x started moving");
  } else if ( initialSpeedX >= 0.03 || initialSpeedX <= -0.03 ) {
    XstartedMoving = true;
   // Serial.println("x started moving first time");
  } else { //Serial.println("x not moving yet");
 }
 
 
 // Check when both conditions are happenning
 if ( XstartedMoving == true &&  gravity_x_Off == true ) {
  // Serial.println("gravity not counting");
   cx = rx;
 } else { cx = gx + rx; 
 //Serial.println("gravity substracted");
 }
 
 
 // Calculate when gyro is not moving enough to rotate
 if ( gyro[1] <= 0 && gyro[1] >= -20 || gyro[1] <= 20 && gyro[1] >= 0) {
  // Serial.print("Gyro small");
 }
 
  // Calculate X edge position --------------------------------------------------------------------
  if ( cx <= 0 && cx >= -0.04 || cx >= 0 && cx <= 0.04 ) {
    cx = 0;
  }
  
  // Calculate speed
  finalSpeedX = initialSpeedX + (cx*9.8)*time;
  
  // If last 5 accels are 0 set speed to 0
  if ( indexX < 4 ) {

    x_accels[indexX] = cx;
    indexX = indexX + 1;

  }
  
  // if the array is full
  else  {

    x_accels[0] = x_accels[1];
    x_accels[1] = x_accels[2];
    x_accels[2] = x_accels[3];
    x_accels[3] = x_accels[4];
    x_accels[4] = cx;


    if (  x_accels[0] <= 0 && x_accels[0] >= -0.04 || x_accels[0] >= 0 && x_accels[0] <= 0.04) {
      x_accels_Zero = true;
      //Serial.println("[0]");
    }
    if (  x_accels[1] <= 0 && x_accels[1] >= -0.04 || x_accels[1] >= 0 && x_accels[1] <= 0.04) {
      x_accels_One = true;
      // Serial.println("[1]");
    }
    if (  x_accels[2] <= 0 && x_accels[2] >= -0.04 || x_accels[2] >= 0 && x_accels[2] <= 0.04) {
      x_accels_Two = true;
      // Serial.println("[2]");
    }
    if (  x_accels[3] <= 0 && x_accels[3] >= -0.04 || x_accels[3] >= 0 && x_accels[3] <= 0.04) {
      x_accels_Three = true;
      // Serial.println("[3]");
    }
    if (  x_accels[4] <= 0 && x_accels[4] >= -0.04 || x_accels[4] >= 0 && x_accels[4] <= 0.04) {
      x_accels_Four = true;
      // Serial.println("[4]");
    }

    if ( x_accels_Zero == true && x_accels_One == true && x_accels_Two == true && x_accels_Three == true && x_accels_Four == true){
      // Serial.println("now");
      finalSpeedX = 0;
      x_accels_Zero = false;
      x_accels_One = false;
      x_accels_Two = false;
      x_accels_Three = false;
      x_accels_Four = false;
    } 
    else {
      x_accels_Zero = false;
      x_accels_One = false;
      x_accels_Two = false;
      x_accels_Three = false;
      x_accels_Four = false;
    }
  }
  
  // Calculate distance
  finalPosX = initialPosX + initialSpeedX * time + 0.5 * cx * time*time;
  // Store Variables
  initialSpeedX = finalSpeedX;
  initialPosX = finalPosX;
  
//  Serial.print("#acceleration");  
//  Serial.print('=');
//  Serial.print(cx);
//  Serial.println();
//
//  Serial.print("#speedX");  
//  Serial.print('=');
//  Serial.print(finalSpeedX); 
//  Serial.print("PosX");
//  Serial.print(finalPosX); 
//  Serial.println();
  
  
  // Calculate Y edge ----------------------------------------------------------------------------------------------
  
  // Calculate when gyro is not moving enough to rotate
// if ( gyro[0] <= 0 && gyro[0] >= -50 || gyro[0] <= 50 && gyro[0] >= 0) {
//   Serial.println("Gyro small");
// }
  
  // Calculate Roll angle
  if (_roll >= -5 && _roll <= 5) {
    //  Serial.print("smaller than 5");
      cy = gy + ry;
  } else if (_roll >= -15 && _roll <= 15) {
    //Serial.print("smaller than 15");
    cy = ry;
  } else {
   // Serial.print("bigger than 15");
    cy = gy + ry;
  }
  
  
  // Check when the board is flipping
  if ( previousRoll >= _roll+3 || previousRoll <= _roll-3){
   // Serial.print("flipping");
    cy = 0;
  }
  
  
  if ( cy <= 0 && cy >= -0.04 || cy >= 0 && cy <= 0.04 ) {
    cy = 0;
  }
  
  // Calculate Speed
  finalSpeedY = initialSpeedY + (cy*9.8)*time;
  
  
  if ( indexY < 3 ) {

    y_accels[indexY] = cy;
    indexY = indexY + 1;

  }

  // if the array is full
  else  {

    y_accels[0] = y_accels[1];
    y_accels[1] = y_accels[2];
    y_accels[2] = y_accels[3];
    y_accels[3] = cy;


    if (  y_accels[0] <= 0 && y_accels[0] >= -0.02 || y_accels[0] >= 0 && y_accels[0] <= 0.02) {
      y_accels_Zero = true;
      // Serial.println("[0]");
    }
    if (  y_accels[1] <= 0 && y_accels[1] >= -0.02 || y_accels[1] >= 0 && y_accels[1] <= 0.02) {
      y_accels_One = true;
      // Serial.println("[1]");
    }
    if (  y_accels[2] <= 0 && y_accels[2] >= -0.02 || y_accels[2] >= 0 && y_accels[2] <= 0.02) {
      y_accels_Two = true;
      // Serial.println("[2]");
    }
    if (  y_accels[3] <= 0 && y_accels[3] >= -0.02 || y_accels[3] >= 0 && y_accels[3] <= 0.02) {
      y_accels_Three = true;
      //Serial.println("[3]");
    }

    if ( y_accels_Zero == true && y_accels_One == true && y_accels_Two == true && y_accels_Three == true){
      // Serial.println("now");
      finalSpeedY = 0;
      y_accels_Zero = false;
      y_accels_One = false;
      y_accels_Two = false;
      y_accels_Three = false;
    } 
    else {
      y_accels_Zero = false;
      y_accels_One = false;
      y_accels_Two = false;
      y_accels_Three = false;
    }
  }
  
  // Calculate position
  finalPosY = initialPosY + initialSpeedY * time + 0.5 * cy * time*time;
  // Restart Values
  initialSpeedY = finalSpeedY;
  initialPosY = finalPosY;
  previousRoll = _roll;
  
//    Serial.println("roll");
//    Serial.println(_roll);
//    Serial.print("#speedY");  Serial.print('=');
//    Serial.print(finalSpeedY);
//    Serial.print("#acceleration");  Serial.print('=');
//    Serial.print(cy);
//    Serial.print("positionY");
//    Serial.print(finalPosY);
//    Serial.println();
   
   
   
    
  // Calculate Z angle -------------------------------------------------------------------------------------------
  
 // cz = gz + rz;
  
//  if ( rz > 0 ) {
//    cz = gz - rz;
//  } else if ( rz < 0 && gz < 0 ) {
//    cz = gz + rz;
//  }
  
  cz = gz - rz;
  
   if ( cz <= 0 && cz >= -0.09 || cz >= 0 && cz <= 0.09 ) {
    cz = 0;
  }
  
  // Calculate Speed
  finalSpeedZ = initialSpeedZ + (cz*9.8)*time;
  
  // if previous accel is < 0.98 && this > 0.98 set speed to 0
  if (previousZaccel < 0.98 && rz > 0.98 ) {
    finalSpeedZ = 0;
  }
  
  
  
  
   // If last 5 accels are 0 set speed to 0
  if ( indexZ < 4 ) {

    z_accels[indexZ] = cz;
    indexZ = indexZ + 1;
    
  }
  
  // if the array is full
  else  {

    z_accels[0] = z_accels[1];
    z_accels[1] = z_accels[2];
    z_accels[2] = z_accels[3];
    z_accels[3] = z_accels[4];
    z_accels[4] = cz;


    if (  z_accels[0] <= 0 && z_accels[0] >= -0.04 || z_accels[0] >= 0 && z_accels[0] <= 0.04) {
      z_accels_Zero = true;
      //Serial.println("[0]");
    }
    if (  z_accels[1] <= 0 && z_accels[1] >= -0.04 || z_accels[1] >= 0 && z_accels[1] <= 0.04) {
      z_accels_One = true;
      // Serial.println("[1]");
    }
    if (  z_accels[2] <= 0 && z_accels[2] >= -0.04 || z_accels[2] >= 0 && z_accels[2] <= 0.04) {
      z_accels_Two = true;
      // Serial.println("[2]");
    }
    if (  z_accels[3] <= 0 && z_accels[3] >= -0.04 || z_accels[3] >= 0 && z_accels[3] <= 0.04) {
      z_accels_Three = true;
      // Serial.println("[3]");
    }
    if (  z_accels[4] <= 0 && z_accels[4] >= -0.04 || z_accels[4] >= 0 && z_accels[4] <= 0.04) {
      z_accels_Four = true;
      // Serial.println("[4]");
    }

    if ( z_accels_Zero == true && z_accels_One == true && z_accels_Two == true && z_accels_Three == true && z_accels_Four == true){
      // Serial.println("now");
      finalSpeedZ = 0;
      z_accels_Zero = false;
      z_accels_One = false;
      z_accels_Two = false;
      z_accels_Three = false;
      z_accels_Four = false;
    } 
    else {
      z_accels_Zero = false;
      z_accels_One = false;
      z_accels_Two = false;
      z_accels_Three = false;
      z_accels_Four = false;
    }
  }
  
  
  // Calculate distance
  finalPosZ = initialPosZ + initialSpeedZ * time + 0.5 * cz * time*time;
  
  // Restart Values
  previousZaccel = rz;
  initialSpeedZ = finalSpeedZ;
  initialPosZ = finalPosZ;
     Serial.print("#acceleration");  Serial.print('=');
     Serial.print(cz);
     Serial.print("#speedZ");  Serial.print('=');
     Serial.print(finalSpeedZ);
     Serial.print("positionZ");  
     Serial.print(finalPosZ);
     Serial.println();
  
}


