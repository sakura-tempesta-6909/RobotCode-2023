package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mode.Mode;
import frc.robot.SubClass.Const;

public class State {
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;
    public static IntakeState intakeState;
    public static HandState handState;
    public static ArmState armState;
    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        handState = HandState.s_releaseHand;
        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = IntakeState.s_stopConveyor;
        armState = ArmState.s_fixArmPosition;
    }

    public enum DriveState {
        s_fastDrive,
        s_midDrive,
        s_slowDrive,
        s_stopDrive,

    }

    public enum IntakeState {
        s_outtakeConveyor,
        s_intakeConveyor,
        s_stopConveyor,

    }

    public enum HandState {
        s_grabHand,
        s_releaseHand,
    }

    public enum ArmState {
        s_moveArmToSpecifiedPosition,
        s_moveArmMotor,
        s_fixArmPosition,
    }

}
