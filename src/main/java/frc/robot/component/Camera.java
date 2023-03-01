package frc.robot.component;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.subClass.Const;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;

public class Camera implements Component {

    AprilTagDetector detector;
    UsbCamera camera;
    CvSink cvSink;
    CvSource outputStream;
    Mat mat;
    Mat grayMat;
    ArrayList<Integer> tags;
    Scalar outlineColor;
    Scalar xColor;
    AprilTagDetection[] detections;

    public Camera() {
        detector = new AprilTagDetector();
        detector.addFamily("tag16h5", 0);

        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);

        cvSink = CameraServer.getVideo();

        outputStream = CameraServer.putVideo("detect", 640, 480);

        mat = new Mat();
        grayMat = new Mat();
        tags = new ArrayList<>();

        outlineColor = new Scalar(0, 255, 0);
        xColor = new Scalar(0, 0, 255);

    }

    public void calculation() {
        //角度を求める
        State.cameraCenterWidth = SmartDashboard.getNumber("CenterX", 0);
        State.cameraCenterHeight = SmartDashboard.getNumber("CenterY", 0);
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan(State.cameraCenterWidth  / Const.Calculation.Camera.FocalLengthWeight));
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan(State.cameraCenterHeight / Const.Calculation.Camera.FocalLengthHeight));
        SmartDashboard.putNumber("AngleX", State.aprilTagAngleWidth);
        SmartDashboard.putNumber("AngleY", State.aprilTagAngleHeight);

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.aprilTagAngleHeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        State.cameraToTag = (Const.Calculation.Camera.GoalHeight - Const.Calculation.Camera.CameraLensHeight) / Math.tan(angleToGoalRadians);
        State.armToTag = State.cameraToTag - Const.Calculation.Camera.CameraToArm;
        SmartDashboard.putNumber("Distance", State.cameraToTag);


        //apriltagの方を向く
        if (State.aprilTagAngleWidth > 0) {
            State.cameraTrackingZRotation = State.aprilTagAngleWidth / -Const.Calculation.Camera.ThetaMaxWidth * Const.Speeds.MidDrive + -0.2;
            if (State.aprilTagAngleWidth < 9 && State.aprilTagAngleWidth > 3) {
                State.limelightTrackingZRotation = -Const.Speeds.MidDrive;
            }
        } else if (State.aprilTagAngleWidth < 0) {
            State.cameraTrackingZRotation = State.aprilTagAngleWidth / -Const.Calculation.Camera.ThetaMaxWidth * Const.Speeds.MidDrive + 0.2;
            if (State.aprilTagAngleWidth > -9 && State.aprilTagAngleWidth < -3) {
                State.limelightTrackingZRotation = Const.Speeds.MidDrive;
            }
        }
    }

    public void frontDrive() {
        if (State.aprilTagAngleWidth < 5) {
            State.cameraXSpeed  = State.aprilTagAngleHeight / -Const.Calculation.Camera.ThetaMaxHeight * Const.Speeds.MidDrive + -0.2;
        } else if (State.aprilTagAngleWidth >5) {
            State.cameraXSpeed = Const.Speeds.Neutral;
        }
    }

    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void readSensors() {
        calculation();
    }


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
