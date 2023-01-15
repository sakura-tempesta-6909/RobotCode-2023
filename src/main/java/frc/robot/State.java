package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mode.Mode;
import frc.robot.SubClass.Const;

public class State {
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;

    public static double armTargetX;
    public static double armTargetZ;

    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);

        armTargetX = 0.0;
        armTargetZ = 0.0;
    }

    public static void StateReset() {
        driveState = DriveState.s_stopDrive;

        armTargetX = 0.0;
        armTargetZ = 0.0;
    }

    public enum DriveState {
        s_fastDrive,
        s_midDrive,
        s_slowDrive,
        s_stopDrive,

    }

}
