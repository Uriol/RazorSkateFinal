
// Sensivity and interval
float S = 0.00390625;
float time = 0.02;

// General values 
float q[4], qprev[4], rx, ry, rz, gx, gy, gz, cx, cy, cz, qxc[4], qconj[4], trueacc[4];
float _yaw,_pitch, _roll;



// Z position variables
float initialSpeedZ = 0;
float finalSpeedZ = 0;
float finalPosZ, initialPosZ;
float previous_cz;


const int numZaccels = 5;
float z_accels[numZaccels];
int indexZ = 0;
int zZeros = 0;

boolean z_accels_Zero = false;
boolean z_accels_One = false;
boolean z_accels_Two = false;
boolean z_accels_Three = false;
boolean z_accels_Four = false;



void output_angles()
{  

   
  // Get accel in g
  rx = accel[0] * S;
  ry = accel[1] * S;
  rz = accel[2] * S;
//Serial.print("#A");  Serial.print('=');
//    Serial.print(rx); Serial.print(",");
//    Serial.print(ry); Serial.print(",");
//    Serial.print(rz); Serial.println();
 
//    Serial.print("#A");  Serial.print('=');
//    Serial.print(rx); Serial.print(",");
//    Serial.print(ry); Serial.print(",");
//    Serial.print("Raw accel : "); Serial.println(rz);
    
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

 
  




//  Serial.print("#quaternion-");  
//  Serial.print("q[0]");
//  Serial.print(q[0]); 
//  Serial.print("q[1]");
//  Serial.print(q[1]); 
//  Serial.print("q[2]");
//  Serial.print(q[2]);
//  Serial.print("q[3]");
//  Serial.print(q[3]);
//  Serial.println();
  
  
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
  
  // Compansate Gravity factor
  cx = rx + gx;
  cy = ry + gy;
  cz = rz - gz;
  
  
  Serial.print("cx"); Serial.print(cx);
  Serial.print("cy"); Serial.print(cy);
  Serial.print("cz: "); Serial.print(cz);
  Serial.println();  
 

  //double integration
  //find the quaternion product of the previous quaternion vector and the current dynamic acceleration vector
  qxc[0] = (qprev[0]*0) - (qprev[1]*cx) - (qprev[2]*cy) - (qprev[3]*cz);
  qxc[1] = (qprev[0]*cx) + (qprev[1]*0) + (qprev[2]*cz) - (qprev[3]*cy);
  qxc[2] = (qprev[0]*cy) - (qprev[1]*cz) + (qprev[2]*0) + (qprev[3]*cx);
  qxc[3] = (qprev[0]*cz) + (qprev[1]*cy) - (qprev[2]*cx) + (qprev[3]*0);

  //conjugate the previous quaternion vector
  qconj[0] = qprev[0];
  qconj[1] = -1*qprev[1];
  qconj[2] = -1*qprev[2];
  qconj[3] = -1*qprev[3];

  //find the quaternion product of qxc and the conjugate of the previous quaternion vector
  trueacc[0] = (qxc[0]*qconj[0]) - (qxc[1]*qconj[1]) - (qxc[2]*qconj[2]) - (qxc[3]*qconj[3]);
  trueacc[1] = (qxc[0]*qconj[1]) + (qxc[1]*qconj[0]) + (qxc[2]*qconj[3]) - (qxc[3]*qconj[2]);
  trueacc[2] = (qxc[0]*qconj[2]) - (qxc[1]*qconj[3]) + (qxc[2]*qconj[0]) + (qxc[3]*qconj[1]);
  trueacc[3] = (qxc[0]*qconj[3]) + (qxc[1]*qconj[2]) - (qxc[2]*qconj[1]) + (qxc[3]*qconj[0]);

  //save the current quaternions in the previous quaternion area for the next calculation
  qprev[0] = q[0];
  qprev[1] = q[1];
  qprev[2] = q[2];
  qprev[3] = q[3];
  
  
  
//    Serial.print("trueacc[0]"); Serial.print(trueacc[0]);
//  Serial.print("trueacc[1]"); Serial.print(trueacc[1]);
//  Serial.print("trueacc[2]"); Serial.print(trueacc[2]);
//  Serial.print("trueacc[3]"); Serial.print(trueacc[3]);
//  Serial.println();  
  
  
  // Calculate z position ------------------------------------------------------------------------------------
   //Serial.print("cz : "); Serial.println(cz);
  cz = trueacc[3];
  
  // if cz is small set it to zero
  if ( cz <= 0 && cz >= -0.05 || cz >= 0 && cz <= 0.05 ) {
    cz = 0;
  } 
  
  //Serial.print("true accel : "); Serial.println(cz);
  
  
  
  // Calculate Speed
  finalSpeedZ = initialSpeedZ + (cz*9.8)*time;
  //Serial.print("speed: "); Serial.println(finalSpeedZ);
  
  
  
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
  
  // Detect when jumping up
  if ( finalSpeedZ > 0) {
  //  Serial.println("Going up");
  } else if (  finalSpeedZ < 0 ) {
  //  Serial.println("Going down");
//    finalSpeedZ = 9.8*time + initialSpeedZ;
  }
  
 
  
  
  
  if (previous_cz < 0 && cz > 0 ) {
   // Serial.println("speed set to zero");
    finalSpeedZ = 0;
  } else if ( previous_cz > 0 && cz < 0 ) {
    
  }
  
  
  
  
  // Calculate position
  finalPosZ = initialPosZ + initialSpeedZ * time + 0.5 * cz * time*time;
   Serial.print("finalPosZ: "); Serial.println(finalPosZ);
  // Restart Values
  previous_cz = cz;
  initialSpeedZ = finalSpeedZ;
  initialPosZ = finalPosZ;
  
   
//  if (cz > 0) {
//    Serial.print("jumping up");
//  }
 
  
}


