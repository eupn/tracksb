import processing.serial.*;
import toxi.geom.*;
import toxi.geom.mesh.*;
import toxi.math.waves.*;
import toxi.processing.*;
import grafica.*;

float [] q = new float [4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

PFont font;
final int VIEW_SIZE_X = 1000, VIEW_SIZE_Y = 800;

PShape skate;
BufferedReader reader;
String line = null;

String LOG_PATH = "../ble-quaternions-listener/motion_log.txt";

final int ACCEL_NUM_POINTS = 1000;
final int GYRO_NUM_POINTS = 1000;

final int ACCEL_PLOT_HEIGHT = 200;
final int ACCEL_PLOT_WIDTH = VIEW_SIZE_X / 2;
final int GYRO_PLOT_HEIGHT = 200;
final int GYRO_PLOT_WIDTH = VIEW_SIZE_X / 2;

final int PLOTS_X = 0;
final int PLOTS_Y = VIEW_SIZE_Y - ACCEL_PLOT_HEIGHT;

GPlot accel_plot;
GPointsArray accel_x_points = new GPointsArray();
GPointsArray accel_y_points = new GPointsArray();
GPointsArray accel_z_points = new GPointsArray();

GPlot gyro_plot;
GPointsArray gyro_x_points = new GPointsArray();
GPointsArray gyro_y_points = new GPointsArray();
GPointsArray gyro_z_points = new GPointsArray();

int time = 0;
int timer = 0;

void setup() 
{
  size(1000, 800, P3D);
  
  skate = loadShape("skateboard.obj");
  
  //myPort = new Serial(this, "/dev/tty.usbmodemTEST1", 115200);  
  font = loadFont("CourierNew36.vlw"); 
  delay(100);

  reader = createReader(LOG_PATH);
  
  accel_plot = new GPlot(this, PLOTS_X, PLOTS_Y, ACCEL_PLOT_WIDTH, ACCEL_PLOT_HEIGHT);
  accel_plot.setBgColor(0);
  accel_plot.setBoxBgColor(0);
  accel_plot.setBoxLineColor(color(255, 255, 255));
  accel_plot.setGridLineColor(color(255, 255, 255));
  accel_plot.setFontColor(color(255, 255, 255));
  accel_plot.setTitleText("Linear acceleration");
  accel_plot.getXAxis().setAxisLabelText("time");
  accel_plot.getYAxis().setAxisLabelText("m/s^2");
  accel_plot.activatePanning();
  accel_plot.setYLim(-10, 10);
  accel_plot.addLayer("x", new GPointsArray());
  accel_plot.getLayer("x").setPointColor(color(0, 0, 255));
  accel_plot.addLayer("y", new GPointsArray());
  accel_plot.getLayer("y").setPointColor(color(255, 0, 0));
  accel_plot.addLayer("z", new GPointsArray());
  accel_plot.getLayer("z").setPointColor(color(0, 255, 0));
  
  gyro_plot = new GPlot(this, PLOTS_X + ACCEL_PLOT_WIDTH + 30, PLOTS_Y, GYRO_PLOT_WIDTH, GYRO_PLOT_HEIGHT);
  gyro_plot.setBgColor(0);
  gyro_plot.setBoxBgColor(0);
  gyro_plot.setBoxLineColor(color(255, 255, 255));
  gyro_plot.setGridLineColor(color(255, 255, 255));
  gyro_plot.setFontColor(color(255, 255, 255));
  gyro_plot.setTitleText("Rotation");
  gyro_plot.getXAxis().setAxisLabelText("time");
  gyro_plot.getYAxis().setAxisLabelText("rad/s");
  gyro_plot.activatePanning();
  gyro_plot.setYLim(-10, 10);
  gyro_plot.addLayer("x", new GPointsArray());
  gyro_plot.getLayer("x").setPointColor(color(0, 0, 255));
  gyro_plot.addLayer("y", new GPointsArray());
  gyro_plot.getLayer("y").setPointColor(color(255, 0, 0));
  gyro_plot.addLayer("z", new GPointsArray());
  gyro_plot.getLayer("z").setPointColor(color(0, 255, 0));
}

void draw() {
  if (millis() - timer >= 1) {
    try {
      String currentLine = "";

      while ((currentLine = reader.readLine()) != null) {
          line = currentLine;
      }
    } catch (IOException e) {
      e.printStackTrace();
      line = null;
    }

    timer = millis();
  }
  
  PVector accel = new PVector(0, 0, 0);
  if (line != null) {
    line = trim(line);
    String items[] = split(line, ' ');

    if (items.length > 1) {
      float x = float(items[0]);
      float y = float(items[1]);
      float z = float(items[2]);
      float w = float(items[3]);
      
      float acc_x = float(items[4]);
      float acc_y = float(items[5]);
      float acc_z = float(items[6]);

      accel = new PVector(acc_x, acc_y, acc_z);

      float gyro_x = float(items[7]);
      float gyro_y = float(items[8]);
      float gyro_z = float(items[9]);

      quat.set(w, x, y, z);

      q[0] = w;
      q[1] = x;
      q[2] = y;
      q[3] = z;
      
      // Accel Plot

      accel_plot.setXLim(time - ACCEL_NUM_POINTS + 1, time + 1);
      accel_x_points.add(time, acc_x);
      if (accel_x_points.getNPoints() > ACCEL_NUM_POINTS) {
        accel_x_points.remove(0);
      }
      accel_plot.getLayer("x").setPoints(accel_x_points);
      
      accel_y_points.add(time, acc_y);
      if (accel_y_points.getNPoints() > ACCEL_NUM_POINTS) {
        accel_y_points.remove(0);
      }
      accel_plot.getLayer("y").setPoints(accel_y_points);

      accel_z_points.add(time, acc_z);
      if (accel_z_points.getNPoints() > ACCEL_NUM_POINTS) {
        accel_z_points.remove(0);
      }
      accel_plot.getLayer("z").setPoints(accel_z_points);
      
      // Gyro Plot
      gyro_plot.setXLim(time - GYRO_NUM_POINTS + 1, time + 1);
      gyro_x_points.add(time, gyro_x);
      if (gyro_x_points.getNPoints() > GYRO_NUM_POINTS) {
        gyro_x_points.remove(0);
      }
      gyro_plot.getLayer("x").setPoints(gyro_x_points);

      gyro_y_points.add(time, gyro_y);
      if (gyro_y_points.getNPoints() > GYRO_NUM_POINTS) {
        gyro_y_points.remove(0);
      }
      gyro_plot.getLayer("y").setPoints(gyro_y_points);
      
      gyro_z_points.add(time, gyro_z);
      if (gyro_z_points.getNPoints() > GYRO_NUM_POINTS) {
        gyro_z_points.remove(0);
      }
      gyro_plot.getLayer("z").setPoints(gyro_z_points);

      time += 5;
    }
  }

  background(0);

  drawCube(accel);
  
  accel_plot.beginDraw();
  accel_plot.drawBackground();
  accel_plot.drawBox();
  accel_plot.drawXAxis();
  accel_plot.drawYAxis();
  accel_plot.drawTopAxis();
  accel_plot.drawRightAxis();
  accel_plot.drawTitle();
  accel_plot.getMainLayer().drawPoints();
  accel_plot.getLayer("x").drawPoints();
  accel_plot.getLayer("y").drawPoints();
  accel_plot.getLayer("z").drawPoints();
  accel_plot.endDraw();
  
  gyro_plot.beginDraw();
  gyro_plot.drawBackground();
  gyro_plot.drawBox();
  gyro_plot.drawXAxis();
  gyro_plot.drawYAxis();
  gyro_plot.drawTopAxis();
  gyro_plot.drawRightAxis();
  gyro_plot.drawTitle();
  gyro_plot.getMainLayer().drawPoints();
  gyro_plot.getLayer("x").drawPoints();
  gyro_plot.getLayer("y").drawPoints();
  gyro_plot.getLayer("z").drawPoints();
  gyro_plot.endDraw();
}

void keyPressed() {

}
