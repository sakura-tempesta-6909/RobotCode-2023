package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.CameraState;
import frc.robot.states.DriveState;
import frc.robot.states.LimelightState;
import frc.robot.consts.CameraConst;
import frc.robot.consts.DriveConst;
import frc.robot.consts.LimelightConst;
import frc.robot.subClass.Util;

public class Drive implements Component {
    private final WPI_TalonSRX driveRightFront, driveLeftFront;
    private final VictorSPX driveRightBack, driveLeftBack;
    private DifferentialDrive differentialDrive;
    private final PIDController pidLimelightDrive;
    private final PIDController pidCameraDrive, pidDriveLong, pidDriveMiddle ,pidDriveShort;
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

       
        pidDriveLong = new PIDController(0.5, 0.1, 0);
        pidDriveLong.setIntegratorRange(-0.2 / 0.1, 0.6 / 0.1);
        pidDriveMiddle = new PIDController(0.7,0.1,0);
        
    
        pidDriveShort = new PIDController(1.0,0.4,0);
        pidDriveShort.setIntegratorRange(-0.05/ 0.4, 1000000000 / 0.4);
        
    }

    public void trapezoidalAccelerationArcadeDrive(double xSpeed, double zRotation) {
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

    public void arcadeDrive(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        differentialDrive.feed();
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
        trapezoidalAccelerationArcadeDrive(LimelightState.limelightXSpeed * 0.7, -limelightTrackingZRotation);
    }

    public void pidControlAprilTagTracking() {
        double cameraZRotation = pidCameraDrive.calculate(CameraState.aprilTagAngleWidth, 0);
        if (cameraZRotation > 0.5) {
            cameraZRotation = 0.5;
        } else if (cameraZRotation < -0.5) {
            cameraZRotation = -0.5;
        }
        trapezoidalAccelerationArcadeDrive(CameraState.cameraXSpeed * DriveConst.Speeds.MidDrive, cameraZRotation);
    }

    /**
     * PIDでtargetLength分前後に動かす
     */
    private void drivePosition() {
        if (Math.abs(DriveState.targetMeter) > DriveConst.PID.MiddleThreshold) {
            trapezoidalAccelerationArcadeDrive(pidDriveLong.calculate((DriveState.rightMeter + DriveState.leftMeter) / 2, DriveState.targetMeter), 0);
        } else if (Math.abs(DriveState.targetMeter) > DriveConst.PID.ShortThreshold){
           trapezoidalAccelerationArcadeDrive(pidDriveMiddle.calculate((DriveState.rightMeter + DriveState.leftMeter) / 2, DriveState.targetMeter), 0);
        } else {
            trapezoidalAccelerationArcadeDrive(pidDriveShort.calculate((DriveState.rightMeter + DriveState.leftMeter) / 2, DriveState.targetMeter), 0);
        }
        // driveRightFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(DriveState.targetMeter));
        // driveLeftFront.set(ControlMode.Position, Util.Calculate.meterToDriveEncoderPoints(DriveState.targetMeter));
        
        
       
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
        SmartDashboard.putNumber("DriveRP", DriveConst.PID.DriveRight.slot0.kP);
        SmartDashboard.putNumber("DriveRI", DriveConst.PID.DriveRight.slot0.kI);
        SmartDashboard.putNumber("DriveRD", DriveConst.PID.DriveRight.slot0.kD);
        SmartDashboard.putNumber("DriveLP", DriveConst.PID.DriveLeft.slot0.kP);
        SmartDashboard.putNumber("DriveLI", DriveConst.PID.DriveLeft.slot0.kI);
        SmartDashboard.putNumber("DriveLD", DriveConst.PID.DriveLeft.slot0.kD);

        SmartDashboard.putNumber("DriveRP", 0);
        SmartDashboard.putNumber("DriveRI", 0);
        SmartDashboard.putNumber("DriveRD", 0);
        SmartDashboard.putNumber("DriveLP", 0);
        SmartDashboard.putNumber("DriveLI", 0);
        SmartDashboard.putNumber("DriveLD", 0);
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
            pidDriveLong.reset();
            pidDriveMiddle.reset();
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
                trapezoidalAccelerationArcadeDrive(DriveConst.Speeds.FastDrive * DriveState.xSpeed, DriveConst.Speeds.FastDrive * DriveState.zRotation);
                break;
            case s_midDrive:
                arcadeDrive(DriveConst.Speeds.MidDrive * DriveState.xSpeed, DriveConst.Speeds.MidDrive * DriveState.zRotation);
                break;
            case s_slowDrive:
                arcadeDrive(DriveConst.Speeds.SlowDrive * DriveState.xSpeed, DriveConst.Speeds.SlowDrive * DriveState.zRotation);
                break;
            case s_stopDrive:
                trapezoidalAccelerationArcadeDrive(DriveConst.Speeds.Neutral * DriveState.xSpeed, DriveConst.Speeds.Neutral * DriveState.zRotation);
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