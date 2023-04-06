package frc.robot.mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.consts.DriveConst;
import frc.robot.states.DriveState;
import frc.robot.states.DriveState.DriveStates;

public class TestMode extends Mode{
    @Override
    public void changeMode() {}
    
    @Override
    public void changeState() {

        if(driveController.getXButton()){
            DriveState.driveState =DriveState.DriveStates.s_pidDrive;
            DriveState.targetMeter = 0.3;
        } else if (driveController.getYButton()) {
            DriveState.driveState = DriveStates.s_pidDrive;
            DriveState.targetMeter = -0.3;
        }



        if (driveController.getXButtonPressed()) {
            DriveState.resetPosition = true;
            DriveState.resetPIDController = true;
        } else if (driveController.getYButtonPressed()) {
            DriveState.resetPosition = true;
            DriveState.resetPIDController = true;
        }
        // SmartDashboard.putNumber("DriveRP", DriveConst.PID.DriveRight.slot0.kP);
        // SmartDashboard.putNumber("DriveRI", DriveConst.PID.DriveRight.slot0.kI);
        // SmartDashboard.putNumber("DriveRD", DriveConst.PID.DriveRight.slot0.kD);
        // SmartDashboard.putNumber("DriveLP", DriveConst.PID.DriveLeft.slot0.kP);
        // SmartDashboard.putNumber("DriveLI", DriveConst.PID.DriveLeft.slot0.kI);
        // SmartDashboard.putNumber("DriveLD", DriveConst.PID.DriveLeft.slot0.kD);

        // SmartDashboard.putNumber("DriveRP", 0);
        // SmartDashboard.putNumber("DriveRI", 0);
        // SmartDashboard.putNumber("DriveRD", 0);
        // SmartDashboard.putNumber("DriveLP", 0);
        // SmartDashboard.putNumber("DriveLI", 0);
        // SmartDashboard.putNumber("DriveLD", 0);


    }
}
