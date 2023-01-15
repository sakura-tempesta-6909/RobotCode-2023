package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Component.Component;
import frc.robot.Component.Drive;

public class Robot extends TimedRobot {
  ArrayList<Component> compornents;

  @Override
  public void robotInit() {
    compornents = new ArrayList<>();
    compornents.add(new Drive());

    State.StateInit();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    State.StateReset();

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    State.StateReset();
   
  }

  @Override
  public void disabledInit () {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    State.StateReset();
    
  }
}
