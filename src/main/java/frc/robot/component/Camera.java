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

    public void calculation(AprilTagDetection detection) {
        //角度を求める
        State.apriltagAngleHight = Math.toDegrees(Math.atan((detection.getCenterX() - Const.Calculation.Camera.CameraCenterHight) / Const.Calculation.Camera.FocalLengthHight));
        State.apriltagAngleWeight = Math.toDegrees(Math.atan((-detection.getCenterY() - Const.Calculation.Camera.CameraCenterWeight) / Const.Calculation.Camera.FocalLengthWeight));
        SmartDashboard.putNumber("AngleX", State.apriltagAngleHight);
        SmartDashboard.putNumber("AngleY", State.apriltagAngleWeight);

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.apriltagAngleWeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        State.distanceFromCameraToTagCentis = (Const.Calculation.Camera.GoalHightCentis - Const.Calculation.Camera.CameraLensHeightCentis) / Math.tan(angleToGoalRadians);
        SmartDashboard.putNumber("Distance", State.distanceFromCameraToTagCentis);
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
        //カメラからフレームを取得する
        if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            return;
        }
        //画像をグレースケールにする
        Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

        //Apriltagを検出する
        detections = detector.detect(grayMat);
        tags.clear();
        //Apriltagの数だけ繰り返す
        for (AprilTagDetection detection : detections) {
            double[] translation = detection.getHomography();

            System.out.println("Translation" + Arrays.toString(translation));

            for (var i = 0; i <= 3; i++) {
                var j = (i + 1) % 4;
                //i,jのXとYのコーナーの座標を取得
                var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                //検出したApriltagを四角形で囲う
                Imgproc.line(mat, pt1, pt2, outlineColor, 2);

            }

            calculation(detection);

            //検出したApriltagの中心にクロスヘアを描画
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


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
