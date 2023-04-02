package frc.robot.consts;

public class Const {
    public static final class Ports {
        public static final int DriveController = 0;
        public static final int OperateController = 1;
        public static final int Joystick = 2;
    }
    public static void ConstInit() {
        ArmConst.ArmConstInit();
        CameraConst.CameraConstInit();
        DriveConst.DriveConstInit();
        GrabGamePiecePhaseConst.GrabGamePiecePhaseConstInit();
        HandConst.HandConstInit();
        IntakeConst.IntakeConstInit();
        LimelightConst.LimelightConstInit();
        MQTTConst.MQTTConstInit();

    }
}
