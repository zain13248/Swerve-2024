package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

 
    public SwerveModulePosition[] getModulePositions(){

        return( new SwerveModulePosition[]{
          frontLeft.getPosition(),
          frontRight.getPosition(), 
          backLeft.getPosition(),
          backRight.getPosition()});
    
      } 

      
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer;  
    
    public SwerveSubsystem() {
        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getOdometryAngle(), getModulePositions());
    try {
            zeroHeading();
    } catch (Exception e) {
            }
    }

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the swerve modules.
        (Measure<Voltage> volts) -> {
            frontLeft.setTurningLock(0);
            frontRight.setTurningLock(0);
            backLeft.setTurningLock(0);
            backRight.setTurningLock(0);
            
            frontLeft.setDriveVoltage(volts.in(Volts));
            frontRight.setDriveVoltage(volts.in(Volts));
            backLeft.setDriveVoltage(volts.in(Volts));
            backRight.setDriveVoltage(volts.in(Volts));
        },
        // Tell SysId how to record a frame of data for each swerve module.
        log -> {

            // Record a frame for the "front-left" swerve module.
            log.motor("front-left")
            .voltage(m_appliedVoltage.mut_replace(frontLeft.getDriveMotorOutput() * RobotController.getBatteryVoltage(), Volts))                    
            .linearPosition(m_distance.mut_replace(frontLeft.getDrivePosition1(), Meters))
            .linearVelocity(m_velocity.mut_replace(frontLeft.getDriveVelocity1(), MetersPerSecond));


            // Record a frame for the "front-right" swerve module.
            log.motor("front-right")
            .voltage(m_appliedVoltage.mut_replace(frontRight.getDriveMotorOutput() * RobotController.getBatteryVoltage(), Volts))                    
            .linearPosition(m_distance.mut_replace(frontRight.getDrivePosition1(), Meters))
            .linearVelocity(m_velocity.mut_replace(frontRight.getDriveVelocity1(), MetersPerSecond));

            // Record a frame for the "back-left" swerve module.
            log.motor("back-left")
            .voltage(m_appliedVoltage.mut_replace(backLeft.getDriveMotorOutput() * RobotController.getBatteryVoltage(), Volts))                    
            .linearPosition(m_distance.mut_replace(backLeft.getDrivePosition1(), Meters))
            .linearVelocity(m_velocity.mut_replace(backLeft.getDriveVelocity1(), MetersPerSecond));

            // Record a frame for the "back-right" swerve module.
            log.motor("back-right")
            .voltage(m_appliedVoltage.mut_replace(backRight.getDriveMotorOutput() * RobotController.getBatteryVoltage(), Volts))                    
            .linearPosition(m_distance.mut_replace(backRight.getDrivePosition1(), Meters))
            .linearVelocity(m_velocity.mut_replace(backRight.getDriveVelocity1(), MetersPerSecond));
        },
        this));



    
    


public Rotation2d getInitRotation2d(SwerveModule swrvMod) { 
    return Rotation2d.fromRadians(swrvMod.getAbsoluteEncoderRad()); }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
       }

       public Rotation2d getOdometryAngle(){
        return(Rotation2d.fromDegrees(gyro.getYaw()));
      }   
    
    @Override
     public void periodic() {
        // Used for Odometry purposes only, does not affect Teleop
    
        SwerveModulePosition lf = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getAbsoluteEncoderRad()));
        SwerveModulePosition rf = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getAbsoluteEncoderRad()));
        SwerveModulePosition lb = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getAbsoluteEncoderRad()));
        SwerveModulePosition rb = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getAbsoluteEncoderRad()));
    
        odometer.update(getRotation2d(),
                new SwerveModulePosition[]{
                        lf, rf, lb, rb
                });
    
        SmartDashboard.putNumber("Gyro:", getHeading());
        // SmartDashboard.putNumber("Left Front Swerve: ", frontLeft.getAbsoluteEncoderRotations());
        // SmartDashboard.putNumber("Right Front Swerve: ", frontRight.getAbsoluteEncoderRotations());
        // SmartDashboard.putNumber("Left Back Swerve: ", backLeft.getAbsoluteEncoderRotations());
        // SmartDashboard.putNumber("Right Back Swerve: ", backRight.getAbsoluteEncoderRotations());
    
        SmartDashboard.putNumber("Left Front Swerve Encoder: ", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Right Front Swerve Encoder: ", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Left Back Swerve Encoder: ", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Right Back Swerve Encoder: ", backRight.getAbsoluteEncoderRad());
    
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }    



    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void setWheelState( ){

    }
    
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }



}