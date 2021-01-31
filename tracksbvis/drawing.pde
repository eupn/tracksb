final int AXIS_LINE_LENGTH = 150;
final int ACCEL_VEC_SCALE = 50;

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
  strokeWeight(1);

  pushMatrix();
  scale(5,5,5);
  // Rotate skateboard to match axes
  rotateX(radians(90));
  rotateZ(-radians(90));
  shape(skate);
  popMatrix();

  // Axes Y and Z are swapped so Z is the vertical axis instead of Processing's Y

  // X, blue
  stroke(0, 0, 255);
  fill(0, 0, 255);
  line(-AXIS_LINE_LENGTH, 0, 0, AXIS_LINE_LENGTH, 0, 0);
  
  // Y, red
  stroke(255, 0, 0);
  fill(255, 0, 0);
  line(0, 0, -AXIS_LINE_LENGTH, 0, 0, AXIS_LINE_LENGTH); // Line spans across Processing's Z axis
  
  // Z, green
  stroke(0, 255, 0);
  fill(0, 255, 0);
  line(0, -AXIS_LINE_LENGTH, 0, 0, AXIS_LINE_LENGTH, 0); // Line spans across Processing's Y axis
  
  fill(255, 255, 255);
  text("+x", -AXIS_LINE_LENGTH, 0, 0);
  text("-x", AXIS_LINE_LENGTH, 0, 0);
  text("-z", 0, AXIS_LINE_LENGTH, 0);
  text("+z", 0, -AXIS_LINE_LENGTH, 0);
  text("+y", 0, 0, AXIS_LINE_LENGTH);
  text("-y", 0, 0, -AXIS_LINE_LENGTH);
  
  // Draw acceleration vector
  PVector scaled_accel = accel.mult(50);
  stroke(120, 120, 120);
  strokeWeight(4);
  fill(255, 0, 0);
  line(0, 0, 0, scaled_accel.x, scaled_accel.z, -scaled_accel.y);
}
