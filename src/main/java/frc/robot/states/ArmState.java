package frc.robot.states;

import frc.robot.subClass.Const;

public class ArmState {
    /** アームのモード */
    public static ArmStates armState;
    public static MoveLeftAndRightArmState moveLeftAndRightArmState;
    /** 計測した角度(actualRootAngle, actualJointAngle)から計算した長さの値[cm] */
    public static double actualHeight, actualDepth;
    /** ターゲットの高さ[cm] */
    public static double targetHeight;
    /** ターゲットの奥行き[cm] */
    public static double targetDepth;
    /**
     * 計測した角度の値[deg]
     * root - 根本の角度
     * joint - 関節部分の角度
     */
    public static double actualRootAngle, actualJointAngle;
    /**
     * ターゲットの座標から計算された角度の目標値[deg]
     * root - 根本の角度
     * joint - 関節部分の角度
     */
    public static double targetRootAngle, targetJointAngle;
    /** モーターの速度 */
    public static double jointSpeed;
    public static double rootSpeed;
    /** 根本のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
    public static double rootMotorFeedforward;
    /** 関節部分のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
    public static double jointMotorFeedforward;

    public static double targetMoveLeftAndRightAngle;
    public static boolean isMoveLeftAndRightEncoderReset;
    /**
     * アームがターゲット位置にいるかを判定
     * targetAngleとactualAngleの差がPIDAngleTolerance未満でtrue
     * @return jointMotorとrootMotorの両方がatSetpointかどうか
     * */
    public static boolean isAtTarget() {
        boolean isDepthAtSetpoint = Math.abs(targetDepth - actualDepth) < Const.Arm.PIDAngleTolerance;
        boolean isHeightMotorAtSetpoint = Math.abs(targetHeight - actualHeight) < Const.Arm.PIDAngleTolerance;
        return isHeightMotorAtSetpoint && isDepthAtSetpoint;
    }

    public static double moveLeftAndRightMotor;
    /** アームを左右に動かす時の位置 */
    public static double actualLeftAndRightAngle;

    /** PIDコントローラーをリセットする（Integralの値をリセットする） */
    public static boolean resetPidController;
    /** エンコーダーをリセット（その時点の位置を0と定める） */
    public static boolean resetEncoder;

    public static class TargetDepth {
        public static double TopCorn;
        public static double MiddleCorn;
        public static double BottomCorn;

        public static double TopCube;
        public static double MiddleCube;
        public static double BottomCube;

        public static double SubStation;
    }


    public enum ArmStates {
        /** アームを指定した場所に移動させる */
        s_moveArmToSpecifiedPosition,
        /** アームを微調整する */
        s_adjustArmPosition,
        /** アームの支点を動かす */
        s_moveArmMotor,
        /** アームをその場で固定する */
        s_fixArmPosition,
    }

    public enum MoveLeftAndRightArmState {
        /** アームを右に動かす */
        s_moveRightMotor,
        /** アームを左に動かす */
        s_moveLeftMotor,
        /** アームを固定する */
        s_fixLeftAndRightMotor,
        /** アームを真ん中に動かす */
        s_movetomiddle,
        s_limelightTracking,

    }

    public static void StatesInit() {
        //init armMode value
        targetHeight = 10000.0;
        targetDepth = 10000.0;

        actualHeight = 0.0;
        actualDepth = 0.0;

        targetRootAngle = 0.0;
        targetJointAngle = 0.0;

        actualRootAngle = 0.0;
        actualJointAngle = 0.0;

        jointSpeed = 0.0;
        rootSpeed = 0.0;

        rootMotorFeedforward = 0.0;
        jointMotorFeedforward = 0.0;

        moveLeftAndRightMotor = 0.0;
    }


    public static void StatesReset() {
        armState = ArmStates.s_fixArmPosition;
        moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        resetPidController = false;
        resetEncoder = false;
        isMoveLeftAndRightEncoderReset = false;

        // TODO どれくらい引くかを計測する
        TargetDepth.TopCorn = 101.0 ;
        TargetDepth.MiddleCorn = 58.0 + 20;
        TargetDepth.BottomCorn = 30.0 + 10;

        TargetDepth.TopCube = 101.0 -10;
        TargetDepth.MiddleCube = 58.0 + 20;
        TargetDepth.BottomCube = 30.0 + 10;

        TargetDepth.SubStation = 36 + 20;
    }
}