package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mode.Mode;

public class State {
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;

    public static double limelightTrackingZRotation;
    public static double limelightSeekingZRotation;
    public static double distanceFromLimelightToGoalCentis;
    public static double limelightXSpeed;


    public static void StateInit() {
        XboxController driveController = new XboxController(0);
        XboxController operateController = new XboxController(1);
        Mode.addController(driveController, operateController);
    }

    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
    }

    public enum DriveState {
        s_fastDrive,
        s_midDrive,
        s_slowDrive,
        s_stopDrive,
        // targetに照準を合わせる
        s_targetTracking,
        s_targetApproaching,
        s_targetSeeking,

        

    }

    public enum limelightState {
        
    }

}
