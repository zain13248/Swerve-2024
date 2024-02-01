package frc.robot;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

    
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    SendableChooser<Command> ChooseAuto = new SendableChooser<>();



public RobotContainer() {

// Starts recording to data log
    // // Control the drive with split-stick arcade controls
    // swerveSubsystem.setDefaultCommand(new SwerveModule(swerveSubsystem,
    //         () -> driverJoystick.getLeftY(), () -> driverJoystick.getRightY()));
    // Set the default command for the swerve subsystem
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> driverJoystick.getRawAxis(OIConstants.kLogitechLeftXAxis), // Replace with correct axis index for Logitech
            () -> driverJoystick.getRawAxis(OIConstants.kLogitechLeftYAxis), // Replace with correct axis index for Logitech
            () -> driverJoystick.getRawAxis(OIConstants.kLogitechRightXAxis), // Replace with correct axis index for Logitech
            () -> !driverJoystick.getRawButton(OIConstants.kLogitechFieldOrientedButtonIdx) // Replace with correct button index for Logitech
    ));

    configureButtonBindings();

    ChooseAuto.addOption("3Auto", getAutonomousCommand());
    ChooseAuto.addOption("Option 2", getAutonomousCommand());

    Shuffleboard.getTab("Auto").add(ChooseAuto);   
}
    private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 1).onTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverJoystick, 2).onTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(driverJoystick, 3).onTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverJoystick, 4).onTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    new JoystickButton(driverJoystick, 5).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }
  

    public Command getAutonomousCommand() {
        return ChooseAuto.getSelected();

 }
}