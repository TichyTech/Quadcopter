void drawAxes(float size){
  // Draw simple 3D (x-y-z) axes in at the origin
  noFill();
  strokeWeight(16);
  stroke(1,0,0);
  translate(size, 0, 0);
  line(0,0,0,-size,0,0);
  sphere(10);

  stroke(0,1,0);
  translate(-size, size, 0);
  line(0,0,0,0,-size,0);
  sphere(10);

  stroke(0,0,1);
  translate(0, -size, size);
  line(0,0,0,0,0,-size);
  sphere(10);
  translate(0,0,-size);

}

void drawRPYm(float[] RPY, float[] mp, float alpha){
  // Draw a drone representation from RPY angles together with motor percentages
  pushMatrix();
  rotateZ(PI*RPY[2]/180);
  rotateY(PI*RPY[1]/180);
  rotateX(PI*RPY[0]/180);
  
  float scl = 4;

  // This draws the main body box
  noFill();
  fill(1,0,0, alpha);
  stroke(0.8, alpha);
  box(scl*100, scl*40, scl*20);
  
  // This draws the nose Box
  pushMatrix();
  translate(scl*50,0,0);
  stroke(1,1,0,alpha);
  box(scl*10);
  popMatrix();
  
  // This draws the arms together with motor settings
  for (int idx = 0; idx < 4; idx++){
    float x = 1 - 2*(idx / 2);  
    float y = 1 - 2*(idx % 2);
    if (idx >= 2) {
      y = -y;
    }
    
    // This is the arm
    pushMatrix();
    translate(scl*x*50, scl*y*50, 0);
    rotateZ(PI/4 + PI*idx/2);
    float c = 0.2;
    if (idx < 2) c = 1; 
    stroke(c, alpha);
    box(scl*70, scl*10, scl*10);
    popMatrix();
    
    // This is the motor setting
    pushMatrix();
    rotateZ(PI/4 - PI*idx/2);
    translate(scl*120, 0, 10);
    colorMode(HSB, 1);
    stroke(1, mp[idx], 1, alpha);
    box(2,2,scl + scl*100*mp[idx]);
    colorMode(RGB, 1);
    popMatrix();
  }
  
  popMatrix();
}

void drawRef(float[] RPY, float alpha){
  // Draw a drone representation from RPY angles together with motor percentages
  pushMatrix();
  rotateZ(PI*RPY[2]/180);
  rotateY(PI*RPY[1]/180);
  rotateX(PI*RPY[0]/180);
  
  float scl = 4;

  // This draws the main body box
  noFill();
  fill(1,0,0, alpha);
  stroke(0.8, alpha);
  box(scl*100, scl*40, scl*20);
  
  // This draws the nose Box
  pushMatrix();
  translate(scl*50,0,0);
  stroke(1,1,0,alpha);
  box(scl*10);
  popMatrix();
  popMatrix();
}
