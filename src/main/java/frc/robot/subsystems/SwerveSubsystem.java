package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoValues;
import frc.robot.Constants.SwerveConsts;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private SwerveModule frontRight;

    private SwerveModuleState states;

    private AHRS navx;

    //CONSTRUCTOR
    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConsts.FL_turningMotorPort, SwerveConsts.FL_driveMotorPort, 
            SwerveConsts.FL_absoluteEncoderPort, SwerveConsts.FL_offset , false, true, true);
        
        backLeft = new SwerveModule(SwerveConsts.BL_turningMotorPort, SwerveConsts.BL_driveMotorPort, 
            SwerveConsts.BL_absoluteEncoderPort, SwerveConsts.BL_offset , false, true, true);

        backRight = new SwerveModule(SwerveConsts.BR_turningMotorPort, SwerveConsts.BR_driveMotorPort, 
            SwerveConsts.BR_absoluteEncoderPort, SwerveConsts.BR_offset , false, true, true);

        frontRight = new SwerveModule(SwerveConsts.FR_turningMotorPort, SwerveConsts.FR_driveMotorPort, 
            SwerveConsts.FR_absoluteEncoderPort, SwerveConsts.FR_offset , false, true, true);

        navx = new AHRS(SPI.Port.kMXP);;
    }

    //RESET METHODS
    public void resetNavx() {
        navx.zeroYaw();
    }

    public void resetEnc(){
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontRight.resetEncoders();
    }

    //GET METHODS 
    public double getEnc() {
        return frontLeft.getDrivePosition(); 
    }

    //0-360
    public double getYawAngle(){
        return ( /*navx.getYaw()*/ navx.getAngle() % 360 );
    }

    //number line 
    public double getAngle(){
        return navx.getAngle(); 
    }

    public double getRoll() {
        return navx.getRoll(); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYawAngle());
    }

    //modules 
    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
        frontRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConsts.maxSpeed_mps);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }

    // PERIODIC - runs repeatedly (like periodic from timed robot)
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Yaw", getYawAngle());
    }

    // Auto

    public void driveForward(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(AutoValues.driveTranslationSpeed, 0, 0));
        setModuleStates(moduleStates);
    }

    public void driveBackward(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(-AutoValues.driveTranslationSpeed, 0, 0));
        setModuleStates(moduleStates);
    }

    public void strafeLeft(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, -AutoValues.driveTranslationSpeed, 0));
        setModuleStates(moduleStates);
    }

    public void strafeRight(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, AutoValues.driveTranslationSpeed, 0));
        setModuleStates(moduleStates);
    }

    public void rotateLeft(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, -AutoValues.driveRotationSpeed));
        setModuleStates(moduleStates);
    }

    public void rotateRight(){
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, AutoValues.driveRotationSpeed));
        setModuleStates(moduleStates);
    }

    public void pidDrive(double y, double x, double z) {
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(new ChassisSpeeds(y, x, z)); 
        setModuleStates(moduleStates);
    }

}
