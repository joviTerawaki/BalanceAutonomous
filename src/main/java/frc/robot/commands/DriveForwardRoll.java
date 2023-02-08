package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForwardRoll extends CommandBase {
  private final SwerveSubsystem swerve;

  public DriveForwardRoll(SwerveSubsystem newSwerve) {
    swerve = newSwerve; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.driveBackward();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    //dummy value
    return swerve.getRoll() >= 8;
  }
}
