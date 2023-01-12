package frc.robot.Component;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.State;


public class Drive implements Component{

    private WPI_TalonSRX driveRightFront, driveLeftFront;
    private VictorSPX driveRightBack, driveLeftBack;
    private DifferentialDrive differentialDrive;

    public Drive() {
        driveRightFront = new WPI_TalonSRX(0);
        driveLeftFront = new WPI_TalonSRX(1);
        driveRightBack = new VictorSPX(2);
        driveLeftBack = new VictorSPX(3);

        driveRightBack.follow(driveRightFront);
        driveLeftBack.follow(driveLeftFront);

        differentialDrive = new DifferentialDrive(driveLeftFront, driveRightFront);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        differentialDrive.feed();
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
        switch(State.driveState) {
            case s_fastDrive:
                arcadeDrive(0.8 * State.driveXSpeed, 0.8 * State.driveZRotation);
                break;
            case s_midDrive:
                arcadeDrive(0.5 * State.driveXSpeed, 0.5 * State.driveZRotation);
                break;
            case s_slowDrive:
                arcadeDrive(0.3 * State.driveXSpeed, 0.3 * State.driveZRotation); 
                break;
            case s_stopDrive:
                arcadeDrive(0 * State.driveXSpeed, 0 * State.driveZRotation);
                break;
        }
        
    }

}
