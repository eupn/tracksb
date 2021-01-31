import java.io.FileWriter;
import java.io.*;
import java.util.*;
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

File recordingFile = null;
int recordingStartedMillis = 0;
FileWriter fw;
BufferedWriter bw;
PrintWriter pw;

File playingFile = null;
int playingStartedMillis = 0;

String[] recordingFiles = null;

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
  if (playingFile != null) {
    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }
  } else {
    try {
      String currentLine = "";

      while ((currentLine = reader.readLine()) != null) {
        line = currentLine;
      }
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }
  }

  PVector accel = new PVector(0, 0, 0);
  if (line != null) {
    line = trim(line);

    String items[] = split(line, ' ');

    if (items.length == 10) {
      if (recordingFile != null) {
        pw.write(line + '\n');
      }

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

  fill(color(255, 255, 255));
  stroke(color(255, 255, 255));
  if (recordingFile != null) {
    text("Recording to " + recordingFile.getName(), 0, textAscent());
  } else if (playingFile == null) {
    if (playingFile == null && recordingFiles != null) {
      text("Select recording to play:", 0, 10);
      for (int i = 0; i < recordingFiles.length; i++) {
        text(i + ". " + recordingFiles[i], 0, textAscent() * (i + 2));
      }
    } else {
      text("Press [R] to record, [P] to play recording", 0, textAscent());
    }
  } else if (playingFile != null) {
    if (line != null) {
      text("Playing " + playingFile.getName(), 0, textAscent());
    } else {
      text("Playback complete", 0, textAscent());
    }
  }

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
  if (key == 'r') {
    recordingFiles = null;
    playingFile = null;

    if (recordingFile != null) {
      try {
        pw.close();
        fw.close();
      } 
      catch (IOException e) {
        e.printStackTrace();
      }
      recordingFile = null;
      println("Recording stopped");
    } else {
      String timestamp = day() + "_" + month() + "_" + year() + "_" + hour() + "_" + minute() + "_" + second();
      String fileName = sketchPath("recording_" + timestamp + ".txt");
      println("Recording to " + fileName);
      recordingFile = new File(fileName);

      try {
        if (!recordingFile.exists()) {
          recordingFile.createNewFile();
        }

        fw = new FileWriter(recordingFile, true);
        bw = new BufferedWriter(fw);
        pw = new PrintWriter(bw);
        recordingStartedMillis = millis();
      } 
      catch (IOException e) {
        e.printStackTrace();
      }
    }
  } else if (key == 'p') {
    if (recordingFiles != null) {
      recordingFiles = null;
    } 
    if (playingFile != null) {
      playingFile = null;
      reader = createReader(LOG_PATH);
    } else {
      try { 
        recordingFiles = listRecordings().toArray(new String[0]);
      } 
      catch (IOException e) { 
        e.printStackTrace();
      }
    }
  } else if (key >= '0' && key <= '9') {
    if (recordingFiles != null) {
      int recordingIndex = key - '0';
      if (recordingIndex < recordingFiles.length) {
        String selectedRecording = recordingFiles[key - '0'];
        String playingFilePath = sketchPath(selectedRecording);
        playingFile = new File(playingFilePath);
        reader = createReader(playingFilePath);
        playingStartedMillis = millis();
      }
    }
  }
}

ArrayList<String> listRecordings() throws IOException {
  FilenameFilter fileNameFilter = new FilenameFilter() {
    @Override
      public boolean accept(File dir, String name) {
      if (name.startsWith("recording_") && name.lastIndexOf('.') > 0) {

        // get last index for '.' char
        int lastIndex = name.lastIndexOf('.');

        // get extension
        String str = name.substring(lastIndex);

        // match path name extension
        if (str.equals(".txt")) {
          return true;
        }
      }

      return false;
    }
  };

  final List<File> files = Arrays.asList(new File(sketchPath("")).listFiles(fileNameFilter));
  final Map<File, Long> constantLastModifiedTimes = new HashMap<File, Long>();
  for (final File f : files) {
    constantLastModifiedTimes.put(f, f.lastModified());
  }
  Collections.sort(files, new Comparator<File>() {
    @Override
      public int compare(final File f1, final File f2) {
      return constantLastModifiedTimes.get(f2).compareTo(constantLastModifiedTimes.get(f1));
    }
  }
  );
  ArrayList<String> filesList = new ArrayList();
  for (final File f : files) {
    filesList.add(f.getName());
  }
  return filesList;
}

String[] removeByIndex(String[] array, int index) {
  int index2 = array.length-1;
  String old = array[index];
  array[index] = array[index2];
  array[index2] = old;
  array = shorten(array);
  return array;
}
