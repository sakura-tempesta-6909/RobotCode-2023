package frc.robot.consts;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;

public class DriveConst {

    public static final class Ports {
        public static final int DriveRightFront = 8;
        public static final int DriveLeftFront = 10;
        public static final int DriveRightBack = 7;
        public static final int DriveLeftBack = 9;
    }

    public static final class Speeds {
        public static final double Neutral = 0;

        public static final double FastDrive = .9;
        public static final double MidDrive = 0.75;
        public static final double SlowDrive = 0.3;
    }

    public static final double TrapezoidalAccelerationX = 0.03;
    public static final double TrapezoidalAccelerationZ = 0.06;
    public static final double SkipLowSpeedThreshold = 0.7;

    // DrivePoint
    public static final double EncoderPointsPerRevolution = 4096;
    // タイヤの直径を求める 単位はメートル
    public static final double DriveWheelDiameter = Units.inchesToMeters(6.0);
    // タイヤの円周のを求める　単位はメートル
    public static final double DriveLengthPerWheelRevolution = DriveWheelDiameter * Math.PI;

    // 1m進むとどのくらいPointが増えるか
    public static final double DrivePointsPerDriveLength = EncoderPointsPerRevolution / DriveLengthPerWheelRevolution;

    public static final class PID {
        /**
         * 短い移動と判別するための閾値 [m]
         */
        public static final double ShortThreshold = 0.5;
        public static final double LossTolerance = 0.1;
        // DrivePoint
        public static final double EncoderPointsPerRevolution = 4096;
        // タイヤの直径を求める 単位はメートル
        public static final double DriveWheelDiameter = Units.inchesToMeters(6.0);
        // タイヤの円周のを求める　単位はメートル
        public static final double DriveLengthPerWheelRevolution = DriveWheelDiameter * Math.PI;

        // 1m進むとどのくらいPointが増えるか
        public static final double PointsPerLength = EncoderPointsPerRevolution / DriveLengthPerWheelRevolution;

        public static final int LongSlotIdx = 0;
        public static final int ShortSlotIdx = 1;
        public static final TalonSRXConfiguration DriveRight = new TalonSRXConfiguration();
        public static final TalonSRXConfiguration DriveLeft = new TalonSRXConfiguration();

        public static void init() {
            DriveRight.slot0.kP = 0.051;
            DriveRight.slot0.kI = 0.000006;
            DriveRight.slot0.kD = 0.00054;
            DriveRight.slot0.maxIntegralAccumulator = 1023 * 0.014 / DriveRight.slot0.kI;

            DriveLeft.slot0.kP = 0.048;
            DriveLeft.slot0.kI = 0.000009;
            DriveLeft.slot0.kD = 0.00054;
            DriveLeft.slot0.maxIntegralAccumulator = 1023 * 0.014 / DriveLeft.slot0.kI;

            DriveRight.slot1.kP = 0.2;
            DriveRight.slot1.kI = 0.004;
            DriveRight.slot1.kD = 0.000;
            DriveRight.slot1.maxIntegralAccumulator = 1023 * 0.1 / DriveRight.slot1.kI;

            DriveLeft.slot1.kP = 0.2;
            DriveLeft.slot1.kI = 0.0004;
            DriveLeft.slot1.kD = 0.000;
            DriveLeft.slot1.maxIntegralAccumulator = 1023 * 0.1 / DriveLeft.slot1.kI;
        }
    }
    public static void DriveConstInit() {

    }
}
