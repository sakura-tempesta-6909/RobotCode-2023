package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.State;
import frc.robot.subClass.Const;

public class Drive implements Component {

    private WPI_TalonSRX driveRightFront, driveLeftFront;
    private VictorSPX driveRightBack, driveLeftBack;
    private DifferentialDrive differentialDrive;
    private PIDController pidLimelightDrive;

    public Drive() {
        driveRightFront = new WPI_TalonSRX(Const.Ports.DriveRightFront);
        driveLeftFront = new WPI_TalonSRX(Const.Ports.DriveLeftFront);
        driveRightBack = new VictorSPX(Const.Ports.DriveRightBack);
        driveLeftBack = new VictorSPX(Const.Ports.DriveLeftBack);

        driveRightBack.follow(driveRightFront);
        driveLeftBack.follow(driveLeftFront);
        driveRightFront.setInverted(true);
        driveRightBack.setInverted(true);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);
        pidLimelightDrive = new PIDController(Const.Pid.LimelightDriveP, Const.Pid.LimelightDriveI, Const.Pid.LimelightDriveD);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        differentialDrive.feed();
    }

    public void pidControlTargetTracking() {
        double limelightTrackingZRotation = pidLimelightDrive.calculate(State.tx, 0);
        if (limelightTrackingZRotation > 0.5) {
            limelightTrackingZRotation = 0.5;
        } else if (limelightTrackingZRotation < -0.5) {
            limelightTrackingZRotation = -0.5;
        }
        arcadeDrive(State.limelightXSpeed * Const.Speeds.MidDrive, limelightTrackingZRotation);
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
        // TODO Auto-generated method stub

    }

    @Override
    public void applyState() {
        switch (State.driveState) {
            case s_fastDrive:
                arcadeDrive(Const.Speeds.FastDrive * State.driveXSpeed, Const.Speeds.FastDrive * State.driveZRotation);
                break;
            case s_midDrive:
                arcadeDrive(Const.Speeds.MidDrive * State.driveXSpeed, Const.Speeds.MidDrive * State.driveZRotation);
                break;
            case s_slowDrive:
                arcadeDrive(Const.Speeds.SlowDrive * State.driveXSpeed, Const.Speeds.SlowDrive * State.driveZRotation);
                break;
            case s_stopDrive:
                arcadeDrive(Const.Speeds.Neutral * State.driveXSpeed, Const.Speeds.Neutral * State.driveZRotation);
                break;
            case s_targetTracking:
                pidControlTargetTracking();
                break;
            case s_apriltagTracking:
                arcadeDrive(Const.Speeds.Neutral * State.driveXSpeed, State.cameraTrackingZRotation);
                break;

        }

        if (State.pidLimelightReset) {
            pidLimelightDrive.setPID(0, 0, 0);
        }
    }
}

