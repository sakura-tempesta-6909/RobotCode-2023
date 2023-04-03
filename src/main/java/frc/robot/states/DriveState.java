package frc.robot.states;

import frc.robot.consts.*;

public class DriveState {
    public static DriveStates driveState;
    /** PID時のターゲット[cm] 正が前向き */
    public static double targetMeter;
    /** 左右のモーターの位置[cm] 正が前向き */
    public static double rightMeter, leftMeter;
    /** arcadeDrive用の引数 */
    public static double xSpeed, zRotation;

    public static boolean resetPIDController, resetPosition;

    public static boolean isMotorBrake;

    public static boolean isAtTarget() {
        boolean isLeftMotorAtTarget = Math.abs(leftMeter - targetMeter) < DriveConst.PID.LossTolerance;
        boolean isRightMotorAtTarget = Math.abs(rightMeter - targetMeter) < DriveConst.PID.LossTolerance;
        return isRightMotorAtTarget && isLeftMotorAtTarget;
    }

    public enum DriveStates {
        /** ロボットの速度を速くする */
        s_fastDrive,
        /** ロボットの速度を中くらいにする */
        s_midDrive,
        /** ロボットの速度を遅くする */
        s_slowDrive,
        /** ロボットの速度を0にする */
        s_stopDrive,
        /** コーンのtargetに照準を合わせる */
        s_limelightTracking,
        /** キューブのtargetに照準を合わせる */
        s_aprilTagTracking,
        /** pidで動く */
        s_pidDrive,
    }

    public static void StatesInit() {
        targetMeter = 0.0;
        xSpeed = 0.0;
        zRotation = 0.0;
        rightMeter = 0.0;
        leftMeter = 0.0;
    }

    public static void StatesReset() {
        driveState = DriveStates.s_stopDrive;
        resetPosition = false;
        resetPIDController = false;
        isMotorBrake = false;
        ArmState.StatesReset();
        HandState.StateReset();
    }
}
