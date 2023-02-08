package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateRightCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredAngle; 

  public RotateRightCommand(SwerveSubsystem newSwerve, double newDesiredAngle) {
    swerve = newSwerve;
    desiredAngle = swerve.getAngle() + newDesiredAngle; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.rotateRight();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return (swerve.getAngle() >= desiredAngle-2) && (swerve.getAngle() <= desiredAngle+2); 
  }
}
