// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.driverobot;
import frc.robot.subsystems.DriveSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final static driverobot c_driverobot = new driverobot();
  SendableChooser<String> autoChooser = new SendableChooser<String>();
  public static SendableChooser<Boolean> fieldoriented = new SendableChooser<Boolean>();
  public static SendableChooser<Boolean> ratelimitChooser = new SendableChooser<Boolean>();
  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
fieldoriented.setDefaultOption("field oriented", true);
fieldoriented.addOption("robot oriented", false);
SmartDashboard.putData("drive mode selector",fieldoriented);

ratelimitChooser.setDefaultOption("false", false);
ratelimitChooser.addOption("true", true);
SmartDashboard.putData("rate limit?", ratelimitChooser);

    configureButtonBindings();
File deploy = Filesystem.getDeployDirectory();
File pathfolder = new File(Path.of(deploy.getAbsolutePath(),"pathplanner","autos").toString());
File[] listOfFiles = pathfolder.listFiles();

for (int i = 0; i < listOfFiles.length; i++) {
  if (listOfFiles[i].isFile()) {
    System.out.println("path:" + listOfFiles[i].getName());
    autoChooser.addOption(listOfFiles[i].getName().replace(".auto", ""), listOfFiles[i].getName().replace(".auto", ""));
  }
}
SmartDashboard.putData("Autonomous",autoChooser);
//String pathplannerlocation = ;

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        c_driverobot);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 3)
    .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));
        /*new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.set0(),
            m_robotDrive));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
   

    // An example trajectory to follow.  All units in meters.
    /*PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath(autoChooser.getSelected(),  new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
           exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.setX());*/
    /*PathPlannerPath path = PathPlannerPath.fromPathFile("test-path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.buildAuto("test-path");*/
    return new PathPlannerAuto(autoChooser.getSelected());

  }
}
