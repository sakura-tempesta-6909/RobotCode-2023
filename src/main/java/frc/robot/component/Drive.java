package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.State;
import frc.robot.subClass.Const;

public class Drive implements Component {
    private final WPI_TalonSRX driveRightFront, driveLeftFront;
    private DifferentialDrive differentialDrive;
    private final PIDController pidLimelightDrive;
    private final PIDController pidCameraDrive;
    private final PIDController pidDriveXspeed;
    private final PIDController pidDriveZRotation;


    public Drive() {
        driveRightFront = new WPI_TalonSRX(Const.Ports.DriveRightFront);
        driveLeftFront = new WPI_TalonSRX(Const.Ports.DriveLeftFront);
        VictorSPX driveRightBack = new VictorSPX(Const.Ports.DriveRightBack);
        VictorSPX driveLeftBack = new VictorSPX(Const.Ports.DriveLeftBack);

        driveRightBack.follow(driveRightFront);
        driveLeftBack.follow(driveLeftFront);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);

        Const.Drive.PID.init();
        driveRightFront.configAllSettings(Const.Drive.PID.DriveRight);
        driveLeftFront.configAllSettings(Const.Drive.PID.DriveLeft);

        driveRightFront.setInverted(true);
        driveRightBack.setInverted(true);
        driveLeftFront.setInverted(false);
        driveLeftBack.setInverted(false);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);
        pidLimelightDrive = new PIDController(Const.Calculation.Limelight.PID.LimelightDriveP, Const.Calculation.Limelight.PID.LimelightDriveI, Const.Calculation.Limelight.PID.LimelightDriveD);
        pidCameraDrive = new PIDController(Const.Calculation.Camera.PID.CameraDriveP, Const.Calculation.Camera.PID.CameraDriveI, Const.Calculation.Camera.PID.CameraDriveD);
        driveRightFront.setNeutralMode(NeutralMode.Brake);
        driveLeftFront.setNeutralMode(NeutralMode.Brake);
        driveRightBack.setNeutralMode(NeutralMode.Brake);
        driveLeftBack.setNeutralMode(NeutralMode.Brake);

        pidDriveXspeed = new PIDController(Const.Drive.PID.XSpeedP, Const.Drive.PID.XSpeedI, Const.Drive.PID.XSpeedD);
        pidDriveZRotation = new PIDController(Const.Drive.PID.ZRotationP, Const.Drive.PID.ZRotationI, Const.Drive.PID.ZRotationD);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        double pidXSpeed = pidDriveXspeed.calculate(xSpeed, 0);
        double pidZRotation = pidDriveZRotation.calculate(zRotation, 0);
        driveRightFront.set(pidXSpeed);
        driveLeftFront.set(pidZRotation);
        differentialDrive.arcadeDrive(State.Drive.xSpeed, State.Drive.zRotation);
        differentialDrive.feed();
    }

    public void pidControlTargetTracking() {
        double limelightTrackingZRotation = pidLimelightDrive.calculate(State.tx, 0);
        if (limelightTrackingZRotation > 0.7) {
            limelightTrackingZRotation = 0.7;
        } else if (limelightTrackingZRotation < -0.7) {
            limelightTrackingZRotation = -0.7;
        }
        arcadeDrive(State.limelightXSpeed * 0.7, -limelightTrackingZRotation);
    }

    public void pidControlApriltagTracking() {
        double cameraZRotation = pidCameraDrive.calculate(State.aprilTagAngleWidth, 0);
        if (cameraZRotation > 0.5) {
            cameraZRotation = 0.5;
        } else if (cameraZRotation < -0.5) {
            cameraZRotation = -0.5;
        }
        arcadeDrive(State.cameraXSpeed * Const.Speeds.MidDrive, cameraZRotation);
    }

    public double PointsToLength(double points) {
        return points / Const.Drive.PID.PointsPerLength;
    }

    public double LengthToPoints(double length) {
        return length * Const.Drive.PID.PointsPerLength;
    }

    /**
     * PIDでtargetLength分前後に動かす
     */
    private void pidDrive() {
        if (Math.abs(State.Drive.targetLength) < Const.Drive.PID.LengthThreshold) {
            driveRightFront.selectProfileSlot(Const.Drive.PID.ShortSlotIdx, 0);
            driveLeftFront.selectProfileSlot(Const.Drive.PID.ShortSlotIdx, 0);
        } else {
            driveRightFront.selectProfileSlot(Const.Drive.PID.LongSlotIdx, 0);
            driveLeftFront.selectProfileSlot(Const.Drive.PID.LongSlotIdx, 0);
        }
        driveRightFront.set(ControlMode.Position, LengthToPoints(State.Drive.targetLength));
        driveLeftFront.set(ControlMode.Position, LengthToPoints(State.Drive.targetLength));
    }

    /**
     * @return 右のモーターの進んだ距離を取得する[cm]
     */
    public double getRightLength() {
        return PointsToLength(driveRightFront.getSelectedSensorPosition());
    }

    /**
     * @return 左のモーターの進んだ距離を取得する[cm]
     */
    public double getLeftLength() {
        return PointsToLength(driveLeftFront.getSelectedSensorPosition());
    }

    private boolean isAtTarget() {
        boolean isLeftMotorAtTarget = Math.abs(State.Drive.leftLength - State.Drive.targetLength) < Const.Drive.PID.LossTolerance;
        boolean isRightMotorAtTarget = Math.abs(State.Drive.rightLength - State.Drive.targetLength) < Const.Drive.PID.LossTolerance;
        return isRightMotorAtTarget && isLeftMotorAtTarget;
    }


    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void readSensors() {
        State.Drive.rightLength = getRightLength();
        State.Drive.leftLength = getLeftLength();

        State.Drive.isAtTarget = isAtTarget();
    }

    @Override
    public void applyState() {
        if (State.pidLimelightReset) {
            pidLimelightDrive.reset();
        }
        if (State.Drive.resetPosition) {
            driveRightFront.setSelectedSensorPosition(0.0);
            driveLeftFront.setSelectedSensorPosition(0.0);
        }

        if (State.Drive.resetPIDController) {
            driveLeftFront.setIntegralAccumulator(0.0);
            driveRightFront.setIntegralAccumulator(0.0);
        }

        switch (State.Drive.state) {
            case s_fastDrive:
                arcadeDrive(Const.Speeds.FastDrive * State.Drive.xSpeed, Const.Speeds.FastDrive * State.Drive.zRotation);
                break;
            case s_midDrive:
                arcadeDrive(Const.Speeds.MidDrive * State.Drive.xSpeed, Const.Speeds.MidDrive * State.Drive.zRotation);
                break;
            case s_slowDrive:
                arcadeDrive(Const.Speeds.SlowDrive * State.Drive.xSpeed, Const.Speeds.SlowDrive * State.Drive.zRotation);
                break;
            case s_stopDrive:
                arcadeDrive(Const.Speeds.Neutral * State.Drive.xSpeed, Const.Speeds.Neutral * State.Drive.zRotation);
                break;
            case s_limelightTracking:
                pidControlTargetTracking();
                break;
            case s_aprilTagTracking:
                pidControlApriltagTracking();
                break;
            case s_pidDrive:
                pidDrive();
                break;
        }

    }
}

