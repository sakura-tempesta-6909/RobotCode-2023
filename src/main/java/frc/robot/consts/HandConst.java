package frc.robot.consts;

public class HandConst {
    public static final class Ports {
        public static final int HandSolenoid = 1;
        public static final int HandRotationMotor = 6;
    }

    public static final class Speeds {
        public static final double HandRotationSpeed = 0.1;
    }
    /**
     * handのモーターののPIDのP
     */
    public static final double P_HANDR = 0.035;
    /**
     * handのモーターののPIDのI
     */
    public static final double I_HANDR = 0.00001;
    /**
     * handのモーターののPIDのD
     */
    public static final double D_HANDR = 0.0;
    /**
     * handのモーターのの積分値の最大値
     */
    public static final double IMax_HANDR = 0.015 / 0.0001;
    /**
     * アームを左右に動かす時のギア比
     */
    public static final double HandGearRatio = 12 * 40 / 24;

    public static final double PIDAngleTolerance = 5;

    public static void HandConstInit() {

    }
}
