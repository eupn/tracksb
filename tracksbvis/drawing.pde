void drawCube(PVector accel) {  
  pushMatrix();
  translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 - 50, 0);
  scale(1, 1, 1);
  
  float[] axis = quat.toAxisAngle();
  rotate(axis[0], axis[1], -axis[3], axis[2]); // Axes Y and Z are swapped so Z is the vertical axis

  drawAxes(accel);
  
  popMatrix();
}


void drawAxes(PVector accel) {
  pushMatrix();
  scale(5,5,5);
  // Rotate skateboard to match axes
  rotateX(radians(90));
  rotateZ(-radians(90));
  //translate(0, 5, -8);
  shape(skate);
  popMatrix();

  // Axes Y and Z are swapped so Z is the vertical axis instead of Processing's Y

  // X, blue
  stroke(0, 0, 255);
  fill(0, 0, 255);
  line(-300, 0, 0, 300, 0, 0);
  
  // Y, red
  stroke(255, 0, 0);
  fill(255, 0, 0);
  line(0, 0, -300, 0, 0, 300); // Line spans across Processing's Z axis
  
  // Z, green
  stroke(0, 255, 0);
  fill(0, 255, 0);
  line(0, -300, 0, 0, 300, 0); // Line spans across Processing's Y axis
  
  fill(255, 255, 255);
  text("+x", -210, 0, 0);
  text("-x", 210, 0, 0);
  text("-z", 0, 210, 0);
  text("+z", 0, -210, 0);
  text("+y", 0, 0, 210);
  text("-y", 0, 0, -210);
  
  // Draw acceleration vector
  PVector scaled_accel = accel.mult(10);
  stroke(255, 0, 0);
  fill(255, 0, 0);
  line(accel.x, accel.y, accel.z, scaled_accel.x, scaled_accel.y, scaled_accel.z);
}
