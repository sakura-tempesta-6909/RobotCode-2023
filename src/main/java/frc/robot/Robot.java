package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.component.Arm;
import frc.robot.component.Component;
import frc.robot.component.Drive;
import frc.robot.component.Hand;
import frc.robot.component.Intake;
import frc.robot.phase.Autonomous;
import frc.robot.subClass.Const;
import frc.robot.subClass.ExternalSensors;
import frc.robot.subClass.Util;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;

public class Robot extends TimedRobot {

    ArrayList<Component> components;

    ExternalSensors externalSensors;

    PrintStream defaultConsole = System.out;
    ByteArrayOutputStream newConsole = new ByteArrayOutputStream();

    @Override
    public void robotInit() {
        System.setOut(new PrintStream(newConsole));
        Const.ConstInit();
        components = new ArrayList<>();
        components.add(new Drive());
        components.add(new Intake());
        components.add(new Hand());
        components.add(new Arm());

        externalSensors = new ExternalSensors();

        State.StateInit();
        Util.sendSystemOut(defaultConsole, newConsole);
        defaultConsole.print(newConsole);
        newConsole = new ByteArrayOutputStream();

        Thread visionThread = new Thread(() -> apriltagVisionThreadProc());
        visionThread.setDaemon(true);
        visionThread.start();
    }

    @Override
    public void robotPeriodic() {
        Util.sendSystemOut(defaultConsole, newConsole);
        defaultConsole.print(newConsole);
        newConsole = new ByteArrayOutputStream();
    }

    @Override
    public void autonomousInit() {
        for (Component component : components) {
            component.autonomousInit();
        }
        Autonomous.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        Autonomous.run();

        for (Component component : components) {
            component.applyState();
        }
    }

    @Override
    public void teleopInit() {
        State.mode = State.Modes.k_drive;

        for (Component component : components) {
            component.teleopInit();
        }
    }

    @Override
    public void teleopPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        State.mode.changeMode();

        State.mode.changeState();

        for (Component component : components) {
            component.applyState();
        }
        Util.allSendConsole();
    }

    @Override
    public void disabledInit() {
        for (Component component : components) {
            component.disabledInit();
        }
    }

    @Override
    public void disabledPeriodic() {
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }
        Util.allSendConsole();
    }

    @Override
    public void testInit() {
        State.mode = State.Modes.k_test;

        for (Component component : components) {
            component.testInit();
        }
    }

    @Override
    public void testPeriodic() {
        externalSensors.readExternalSensors();
        State.StateReset();
        for (Component component : components) {
            component.readSensors();
        }
        State.mode.changeState();

        for (Component component : components) {
            component.applyState();
        }
    }

    void apriltagVisionThreadProc() {
        AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
  
    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    Mat mat = new Mat();
    Mat grayMat = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();

    //
    Scalar outlineColor = new Scalar(0, 255, 0);
    Scalar xColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      tags.clear();
      for (AprilTagDetection detection : detections) {
        tags.add(detection.getId());

        double[] translation = detection.getHomography();

        System.out.println("Translation" + Arrays.toString(translation));

        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
          SmartDashboard.putNumber("CenterX", detection.getCenterX() - 80);
          SmartDashboard.putNumber("CenterY", detection.getCenterY() - 60);

          double thetaX = Math.toDegrees(Math.atan((detection.getCenterX() - 80) / Const.Calculation.FocalLengthX));
          double thetaY = Math.toDegrees(Math.atan((detection.getCenterY() - 60) / Const.Calculation.FocalLengthY));
          SmartDashboard.putNumber("AngleX", thetaX);
          SmartDashboard.putNumber("AngleY", thetaY);
        }

        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
      }

      SmartDashboard.putString("tag", tags.toString());
      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }
}
