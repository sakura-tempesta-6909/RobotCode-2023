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

        Thread visionThread = new Thread(this::detection);
        visionThread.setDaemon(true);
        visionThread.start();

    }

    public void calculation(AprilTagDetection detection) {
        //角度を求める
        SmartDashboard.getNumber("CenterX", State.cameraCenterWidth);
        SmartDashboard.getNumber("CenterY", State.cameraCenterHeight);
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan((State.cameraCenterWidth - Const.Calculation.Camera.CameraCenterHeight) / Const.Calculation.Camera.FocalLengthHeight));
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan((-State.cameraCenterHeight - Const.Calculation.Camera.CameraCenterWidth) / Const.Calculation.Camera.FocalLengthWeight));
        SmartDashboard.putNumber("AngleX", State.aprilTagAngleHeight);
        SmartDashboard.putNumber("AngleY", State.aprilTagAngleWidth);

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.aprilTagAngleWidth;
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

    public void detection() {
        while (true) {
            //カメラからフレームを取得する
            if (cvSink.grabFrame(mat) == 0) {
                outputStream.notifyError(cvSink.getError());
                return;
            }
            //画像をグレースケールにする
            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            //AprilTagを検出する
            detections = detector.detect(grayMat);
            tags.clear();
            //AprilTagの数だけ繰り返す
            for (AprilTagDetection detection : detections) {
                double[] translation = detection.getHomography();

                System.out.println("Translation" + Arrays.toString(translation));

                for (var i = 0; i <= 3; i++) {
                    var j = (i + 1) % 4;
                    //i,jのXとYのコーナーの座標を取得
                    var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                    var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                    //検出したAprilTagを四角形で囲う
                    Imgproc.line(mat, pt1, pt2, outlineColor, 2);

                }

                calculation(detection);

                //検出したAprilTagの中心にクロスヘアを描画
                var cx = detection.getCenterX();
                var cy = detection.getCenterY();
                var ll = 10;
                Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
                Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
                Imgproc.putText(mat, Integer.toString(detection.getId()), new Point(cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
            }

            SmartDashboard.putString("tag", tags.toString());
            outputStream.putFrame(mat);
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

    }


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
