package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForward extends CommandBase{
    private SwerveSubsystem swerve;

    public DriveForward(SwerveSubsystem subs){
        swerve = subs;

        addRequirements(subs);
    }

    @Override
    public void initialize(){
        swerve.resetEnc();
    }

    //test 
    @Override
    public void execute(){
        swerve.driveForward();
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
