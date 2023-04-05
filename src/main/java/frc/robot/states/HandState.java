package frc.robot.states;

import frc.robot.consts.ArmConst;
import frc.robot.consts.HandConst;

public class HandState {
    public static GrabHandStates grabHandState;
    public static HandState.RotateStates rotateState;

    public enum RotateStates {
        /** 手首を回転させる */
        s_rightRotateHand,
        /** 手首を逆回転させる */
        s_leftRotateHand,
        /** 手首の回転を止める */
        s_stopHand,
        /** 手首を元の位置に戻す */
        s_turnHandBack,
        /** 手首を所定の位置に動かす */
        s_moveHandToSpecifiedAngle,
    }

    public enum GrabHandStates {
        /** 物体をつかむ */
        s_grabHand,
        /** 物体を離す */
        s_releaseHand,
    }

    /** 手首の回転した度数 */
    public static double actualHandAngle = 0.0;

    public static double targetAngle = 0.0;
    public static boolean isResetHandPID = false;

    public static boolean isAtTarget() {
        return Math.abs(actualHandAngle - targetAngle) < HandConst.PIDAngleTolerance;
    }

    public static void StateInit() {
    }

    public static void StateReset() {
        grabHandState = GrabHandStates.s_grabHand;
        rotateState = HandState.RotateStates.s_stopHand;
        isResetHandPID = false;
    }
}
