package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.states.CameraState;
import frc.robot.states.DriveState;
import frc.robot.states.LimelightState;
import frc.robot.states.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Util;

public class Drive implements Component {
    private final WPI_TalonSRX driveRightFront, driveLeftFront;
    private final VictorSPX driveRightBack, driveLeftBack;
    private DifferentialDrive differentialDrive;
    private final PIDController pidLimelightDrive;
    private final PIDController pidCameraDrive;
    private double preXSpeed, preZRotation;


    public Drive() {
        driveRightFront = new WPI_TalonSRX(Const.Ports.DriveRightFront);
        driveLeftFront = new WPI_TalonSRX(Const.Ports.DriveLeftFront);
        driveRightBack = new VictorSPX(Const.Ports.DriveRightBack);
        driveLeftBack = new VictorSPX(Const.Ports.DriveLeftBack);

        driveRightBack.follow(driveRightFront);
        driveLeftBack.follow(driveLeftFront);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);

        Const.Drive.PID.init();
        driveRightFront.configAllSettings(Const.Drive.PID.DriveRight);
        driveLeftFront.configAllSettings(Const.Drive.PID.DriveLeft);

        driveLeftFront.setSensorPhase(true);
        driveRightFront.setSensorPhase(true);

        driveRightFront.setInverted(true);
        driveRightBack.setInverted(true);
        driveLeftFront.setInverted(false);
        driveLeftBack.setInverted(false);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);
        pidLimelightDrive = new PIDController(Const.Calculation.Limelight.PID.LimelightDriveP, Const.Calculation.Limelight.PID.LimelightDriveI, Const.Calculation.Limelight.PID.LimelightDriveD);
        pidCameraDrive = new PIDController(Const.Calculation.Camera.PID.CameraDriveP, Const.Calculation.Camera.PID.CameraDriveI, Const.Calculation.Camera.PID.CameraDriveD);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        if(Math.abs(preXSpeed) <  Const.Drive.SkipLowSpeedThreshold) {
            xSpeed = xSpeed >  Const.Drive.SkipLowSpeedThreshold ?  Const.Drive.SkipLowSpeedThreshold : xSpeed < - Const.Drive.SkipLowSpeedThreshold ? - Const.Drive.SkipLowSpeedThreshold : xSpeed;
        }
        if(Math.abs(preZRotation) <  Const.Drive.SkipLowSpeedThreshold) {
            zRotation = zRotation >  Const.Drive.SkipLowSpeedThreshold ?  Const.Drive.SkipLowSpeedThreshold : zRotation < - Const.Drive.SkipLowSpeedThreshold ? - Const.Drive.SkipLowSpeedThreshold : zRotation;
        }
        if (xSpeed - preXSpeed >= Const.Drive.TrapezoidalAccelerationX) {
            xSpeed = preXSpeed + Const.Drive.TrapezoidalAccelerationX;
        } else if (xSpeed - preXSpeed <= -Const.Drive.TrapezoidalAccelerationX) {
            xSpeed = preXSpeed - Const.Drive.TrapezoidalAccelerationX;
        }

         if (zRotation - preZRotation >= Const.Drive.TrapezoidalAccelerationZ) {
            zRotation = preZRotation + Const.Drive.TrapezoidalAccelerationZ;
        } else if (zRotation - preZRotation <= -Const.Drive.TrapezoidalAccelerationZ) {
            zRotation = preZRotation - Const.Drive.TrapezoidalAccelerationZ;
        }
        zRotation = Math.max(Math.min(zRotation, 0.7), -0.7);
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        differentialDrive.feed();

        preXSpeed = xSpeed;
        preZRotation = zRotation;
    }

    public void pidControlTargetTracking() {
        double limelightTrackingZRotation = 0;
        if(LimelightState.tv) {
        limelightTrackingZRotation = pidLimelightDrive.calculate(LimelightState.tx, 0);
        }
        if (limelightTrackingZRotation > 0.7) {
            limelightTrackingZRotation = 0.7;
        } else if (limelightTrackingZRotation < -0.7) {
            limelightTrackingZRotation = -0.7;
        }
        arcadeDrive(LimelightState.limelightXSpeed * 0.7, -limelightTrackingZRotation);
    }

    public void pidControlAprilTagTracking() {
        double cameraZRotation = pidCameraDrive.calculate(CameraState.aprilTagAngleWidth, 0);
        if (cameraZRotation > 0.5) {
            cameraZRotation = 0.5;
        } else if (cameraZRotation < -0.5) {
            cameraZRotation = -0.5;
        }
        arcadeDrive(CameraState.cameraXSpeed * Const.Speeds.MidDrive, cameraZRotation);
    }

    /**
     * PIDでtargetLength分前後に動かす
     */
    private void drivePosition() {
        if (Math.abs(DriveState.targetMeter) > Const.Drive.PID.ShortThreshold) {
            driveRightFront.selectProfileSlot(Const.Drive.PID.LongSlotIdx, 0);
            driveLeftFront.selectProfileSlot(Const.Drive.PID.LongSlotIdx, 0);
        } else {
            driveRightFront.selectProfileSlot(Const.Drive.PID.ShortSlotIdx, 0);
            driveLeftFront.selectProfileSlot(Const.Drive.PID.ShortSlotIdx, 0);
        }
        driveRightFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(DriveState.targetMeter));
        driveLeftFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(DriveState.targetMeter));
    }

    /**
     * @return 右のモーターの進んだ距離を取得する[cm]
     */
    public double getRightLength() {
        return Util.Calculate.driveEncoderPointsToMeter(driveRightFront.getSelectedSensorPosition());
    }

    /**
     * @return 左のモーターの進んだ距離を取得する[cm]
     */
    public double getLeftLength() {
        return Util.Calculate.driveEncoderPointsToMeter(driveLeftFront.getSelectedSensorPosition());
    }

    private boolean isAtTarget() {
        boolean isLeftMotorAtTarget = Math.abs(DriveState.leftMeter - DriveState.targetMeter) < Const.Drive.PID.LossTolerance;
        boolean isRightMotorAtTarget = Math.abs(DriveState.rightMeter - DriveState.targetMeter) < Const.Drive.PID.LossTolerance;
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
        DriveState.rightMeter = getRightLength();
        DriveState.leftMeter = getLeftLength();
    }

    @Override
    public void applyState() {
        if (LimelightState.pidLimelightReset) {
            pidLimelightDrive.reset();
        }
        if (DriveState.resetPosition) {
            driveRightFront.setSelectedSensorPosition(0.0);
            driveLeftFront.setSelectedSensorPosition(0.0);
        }

        if (DriveState.resetPIDController) {
            driveLeftFront.setIntegralAccumulator(0.0);
            driveRightFront.setIntegralAccumulator(0.0);
        }

        if (DriveState.isMotorBrake) {
            driveRightFront.setNeutralMode(NeutralMode.Brake);
            driveLeftFront.setNeutralMode(NeutralMode.Brake);
            driveRightBack.setNeutralMode(NeutralMode.Brake);
            driveLeftBack.setNeutralMode(NeutralMode.Brake);
        } else {
            driveRightFront.setNeutralMode(NeutralMode.Coast);
            driveLeftFront.setNeutralMode(NeutralMode.Coast);
            driveRightBack.setNeutralMode(NeutralMode.Coast);
            driveLeftBack.setNeutralMode(NeutralMode.Coast);
        }


        switch (DriveState.driveState) {
            case s_fastDrive:
                arcadeDrive(Const.Speeds.FastDrive * DriveState.xSpeed, Const.Speeds.FastDrive * DriveState.zRotation);
                break;
            case s_midDrive:
                arcadeDrive(Const.Speeds.MidDrive * DriveState.xSpeed, Const.Speeds.MidDrive * DriveState.zRotation);
                break;
            case s_slowDrive:
                arcadeDrive(Const.Speeds.SlowDrive * DriveState.xSpeed, Const.Speeds.SlowDrive * DriveState.zRotation);
                break;
            case s_stopDrive:
                arcadeDrive(Const.Speeds.Neutral * DriveState.xSpeed, Const.Speeds.Neutral * DriveState.zRotation);
                break;
            case s_limelightTracking:
                pidControlTargetTracking();
                break;
            case s_aprilTagTracking:
                pidControlAprilTagTracking();
                break;
            case s_pidDrive:
                drivePosition();
                break;
        }

    }
}