package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.States.State;
import frc.robot.consts.CameraConst;
import frc.robot.consts.DriveConst;
import frc.robot.consts.LimelightConst;
import frc.robot.subClass.Util;

public class Drive implements Component {
    private final WPI_TalonSRX driveRightFront, driveLeftFront;
    private final VictorSPX driveRightBack, driveLeftBack;
    private DifferentialDrive differentialDrive;
    private final PIDController pidLimelightDrive;
    private final PIDController pidCameraDrive;
    private double preXSpeed, preZRotation;


    public Drive() {
        driveRightFront = new WPI_TalonSRX(DriveConst.Ports.DriveRightFront);
        driveLeftFront = new WPI_TalonSRX(DriveConst.Ports.DriveLeftFront);
        driveRightBack = new VictorSPX(DriveConst.Ports.DriveRightBack);
        driveLeftBack = new VictorSPX(DriveConst.Ports.DriveLeftBack);

        driveRightBack.follow(driveRightFront);
        driveLeftBack.follow(driveLeftFront);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);

        DriveConst.PID.init();
        driveRightFront.configAllSettings(DriveConst.PID.DriveRight);
        driveLeftFront.configAllSettings(DriveConst.PID.DriveLeft);

        driveLeftFront.setSensorPhase(true);
        driveRightFront.setSensorPhase(true);

        driveRightFront.setInverted(true);
        driveRightBack.setInverted(true);
        driveLeftFront.setInverted(false);
        driveLeftBack.setInverted(false);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);
        pidLimelightDrive = new PIDController(LimelightConst.PID.LimelightDriveP, LimelightConst.PID.LimelightDriveI, LimelightConst.PID.LimelightDriveD);
        pidCameraDrive = new PIDController(CameraConst.PID.CameraDriveP, CameraConst.PID.CameraDriveI, CameraConst.PID.CameraDriveD);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        if(Math.abs(preXSpeed) <  DriveConst.SkipLowSpeedThreshold) {
            xSpeed = xSpeed >  DriveConst.SkipLowSpeedThreshold ?  DriveConst.SkipLowSpeedThreshold : xSpeed < - DriveConst.SkipLowSpeedThreshold ? - DriveConst.SkipLowSpeedThreshold : xSpeed;
        }
        if(Math.abs(preZRotation) <  DriveConst.SkipLowSpeedThreshold) {
            zRotation = zRotation >  DriveConst.SkipLowSpeedThreshold ?  DriveConst.SkipLowSpeedThreshold : zRotation < - DriveConst.SkipLowSpeedThreshold ? - DriveConst.SkipLowSpeedThreshold : zRotation;
        }
        if (xSpeed - preXSpeed >= DriveConst.TrapezoidalAccelerationX) {
            xSpeed = preXSpeed + DriveConst.TrapezoidalAccelerationX;
        } else if (xSpeed - preXSpeed <= -DriveConst.TrapezoidalAccelerationX) {
            xSpeed = preXSpeed - DriveConst.TrapezoidalAccelerationX;
        }

         if (zRotation - preZRotation >= DriveConst.TrapezoidalAccelerationZ) {
            zRotation = preZRotation + DriveConst.TrapezoidalAccelerationZ;
        } else if (zRotation - preZRotation <= -DriveConst.TrapezoidalAccelerationZ) {
            zRotation = preZRotation - DriveConst.TrapezoidalAccelerationZ;
        }
        zRotation = Math.max(Math.min(zRotation, 0.7), -0.7);
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        differentialDrive.feed();

        preXSpeed = xSpeed;
        preZRotation = zRotation;
    }

    public void pidControlTargetTracking() {
        double limelightTrackingZRotation = 0;
        if(State.tv) {
        limelightTrackingZRotation = pidLimelightDrive.calculate(State.tx, 0);
        }
        if (limelightTrackingZRotation > 0.7) {
            limelightTrackingZRotation = 0.7;
        } else if (limelightTrackingZRotation < -0.7) {
            limelightTrackingZRotation = -0.7;
        }
        arcadeDrive(State.limelightXSpeed * 0.7, -limelightTrackingZRotation);
    }

    public void pidControlAprilTagTracking() {
        double cameraZRotation = pidCameraDrive.calculate(State.aprilTagAngleWidth, 0);
        if (cameraZRotation > 0.5) {
            cameraZRotation = 0.5;
        } else if (cameraZRotation < -0.5) {
            cameraZRotation = -0.5;
        }
        arcadeDrive(State.cameraXSpeed * DriveConst.Speeds.MidDrive, cameraZRotation);
    }

    /**
     * PIDでtargetLength分前後に動かす
     */
    private void drivePosition() {
        if (Math.abs(State.Drive.targetMeter) > DriveConst.PID.ShortThreshold) {
            driveRightFront.selectProfileSlot(DriveConst.PID.LongSlotIdx, 0);
            driveLeftFront.selectProfileSlot(DriveConst.PID.LongSlotIdx, 0);
        } else {
            driveRightFront.selectProfileSlot(DriveConst.PID.ShortSlotIdx, 0);
            driveLeftFront.selectProfileSlot(DriveConst.PID.ShortSlotIdx, 0);
        }
        driveRightFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(State.Drive.targetMeter));
        driveLeftFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(State.Drive.targetMeter));
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
        boolean isLeftMotorAtTarget = Math.abs(State.Drive.leftMeter - State.Drive.targetMeter) < DriveConst.PID.LossTolerance;
        boolean isRightMotorAtTarget = Math.abs(State.Drive.rightMeter - State.Drive.targetMeter) < DriveConst.PID.LossTolerance;
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
        State.Drive.rightMeter = getRightLength();
        State.Drive.leftMeter = getLeftLength();
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

        if (State.Drive.isMotorBrake) {
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


        switch (State.Drive.state) {
            case s_fastDrive:
                arcadeDrive(DriveConst.Speeds.FastDrive * State.Drive.xSpeed, DriveConst.Speeds.FastDrive * State.Drive.zRotation);
                break;
            case s_midDrive:
                arcadeDrive(DriveConst.Speeds.MidDrive * State.Drive.xSpeed, DriveConst.Speeds.MidDrive * State.Drive.zRotation);
                break;
            case s_slowDrive:
                arcadeDrive(DriveConst.Speeds.SlowDrive * State.Drive.xSpeed, DriveConst.Speeds.SlowDrive * State.Drive.zRotation);
                break;
            case s_stopDrive:
                arcadeDrive(DriveConst.Speeds.Neutral * State.Drive.xSpeed, DriveConst.Speeds.Neutral * State.Drive.zRotation);
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