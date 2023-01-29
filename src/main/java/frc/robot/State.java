package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mode.ArmMode;
import frc.robot.Mode.DriveMode;
import frc.robot.Mode.Mode;
import frc.robot.SubClass.Const;

public class State {

    public static Modes mode;

    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;

    public static ArmState armState;

    public static double armTargetX;
    public static double armTargetZ;

    public static double armTheta1, armTheta2;

    public static double armAxisX;
    public static double armAxisZ;

    public static void StateInit() {
        mode = Modes.m_drive;

        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);

        armTargetX = 0.0;
        armTargetZ = 0.0;
        armTheta1 = 0.0;
        armTheta2 = 0.0;
        armAxisX = 0.0;
        armAxisZ = 0.0;

        StateReset();
    }

    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        armState = ArmState.s_rotationCtrl;
    }

    public enum DriveState {
        s_fastDrive,
        s_midDrive,
        s_slowDrive,
        s_stopDrive,

    }

    public enum ArmState {
        s_autoCtrl,
        s_axisCtrl,
        s_rotationCtrl,
    }

    public enum Modes {
        m_drive (new DriveMode()),
        m_arm (new ArmMode());

        private final Mode mode;

        Modes(Mode mode) {
            this.mode = mode;
        }

        public void changeMode() {
            this.mode.changeMode();
        }

        public void changeState() {
            this.mode.changeState();
        }
    }

}
