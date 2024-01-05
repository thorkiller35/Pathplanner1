// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.armsub;
// import frc.robot.subsystems.basicautosub;
import frc.robot.subsystems.climbsub;
import frc.robot.subsystems.intakesub;
import frc.robot.commands.intakesetcmd;
import frc.robot.commands.intakesetcmdauto;
import frc.robot.commands.indicator.*;
import frc.robot.auton.*;
import frc.robot.commands.intakesetcmd;
import frc.robot.commands.armcmd;

import frc.robot.commands.climbcmd;
import frc.robot.subsystems.armsub;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.05;

	// choosers (for auton)
	

	private SendableChooser<String> autonOptionChooser = new SendableChooser<>();

	// sensors

	// motorized devices

	public final Drivetrain drivetrain = new Drivetrain();
	
	private final intakesub Ointake = new intakesub();

	private final armsub Oarmsub = new armsub();

	private final climbsub oclimbsub = new climbsub();
	


	// private final basicautosub oBasicautosub = new basicautosub();

	// pneumatic devices

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	private final Indicator indicator = new Indicator(null);

	// The driver's controller
	CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.GAMEPAD);
	CommandXboxController copilot = new CommandXboxController(Ports.USB.co_pad);
	// private final Joystick joystick1 = new Joystick(2);
	
	// Joystick driverGamepad = new Joystick(Ports.USB.RIGHT);

	private final SendableChooser<Command> autoChooser1;	

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {


		NamedCommands.registerCommand("Shoot High", Commands.print("Passed marker 1"));
		NamedCommands.registerCommand("Score + Pickup", Commands.print("Passed marker 2"));
		NamedCommands.registerCommand("Pickup", new intakesetcmd(Ointake, 0.37));
	
		// choosers (for auton)
		
		
	
		SmartDashboard.putData("Auton options", autonOptionChooser);		


		// Configure the button bindings
		configureButtonBindings();
		autoChooser1 = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    
		initializeAutonomousChooser();


		// Configure default commands
		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
                    -MathUtil.applyDeadband(driverGamepad.getLeftY(), GAMEPAD_AXIS_THRESHOLD),
                    -MathUtil.applyDeadband(driverGamepad.getLeftX(), GAMEPAD_AXIS_THRESHOLD),
                    MathUtil.applyDeadband(driverGamepad.getRightX(), GAMEPAD_AXIS_THRESHOLD),
                    // -MathUtil.applyDeadband(driverGamepad.getY(), GAMEPAD_AXIS_THRESHOLD),
					// -MathUtil.applyDeadband(driverGamepad.getX(), GAMEPAD_AXIS_THRESHOLD),
					// -MathUtil.applyDeadband(driverGamepad.getZ(), GAMEPAD_AXIS_THRESHOLD),
					true, true),
				drivetrain));

		indicator.setDefaultCommand(new IndicatorScrollRainbow(indicator)); // temp`
		

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {


		driverGamepad.x()
			.whileTrue(new RunCommand(
				() -> drivetrain.setX(),
				drivetrain));

		driverGamepad.y()
			.onTrue(new InstantCommand(
				() -> drivetrain.resetEncoders(),
				drivetrain).ignoringDisable(true));

		driverGamepad.a()
			.onTrue(new InstantCommand(
				() -> drivetrain.zeroHeading(),
				drivetrain).ignoringDisable(true));   
		// copilot.leftTrigger()
		// .onTrue(new InstantCommand(
		// 	() -> Ointake.set_intake(true)
		// ))  ;
		copilot.leftTrigger()
		.whileTrue(new intakesetcmd(Ointake,0.17));
		//Out take


		//Intake2	
		copilot.y()
		.whileTrue(new intakesetcmd(Ointake, 0.88));
		//mid
		copilot.b()
		.whileTrue(new intakesetcmd(Ointake, 0.37));


		copilot.rightTrigger()
		.whileTrue(new intakesetcmd(Ointake,-0.2));
	

		// copilot.b().onTrue(new InstantCommand(() -> Ointake.set_out(true)));
		
		copilot.x().onTrue(new armcmd(Oarmsub, 19.857034));
		// Intake

		// copilot.a()
		// .onTrue((new armcmd(Oarmsub,13.142808)));
		// Low
		copilot.a()
		.onTrue(new armcmd(Oarmsub, 0));


		driverGamepad.leftTrigger()
		.whileTrue(new climbcmd(oclimbsub, 0.5));
		driverGamepad.rightTrigger()
		.whileTrue(new climbcmd(oclimbsub, -0.5));

		
		
		// new JoystickButton(joystick1,1)
		// .whileActiveOnce(new armcmd(Oarmsub, 50));

  
		
	// 	SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   2.0
    // ));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   0
    // ));

	// SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
	// 	Pose2d currentPose = drivetrain.getPose();
		
	// 	// The rotation component in these poses represents the direction of travel
	// 	Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
	// 	Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
  
	// 	List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
	// 	PathPlannerPath path = new PathPlannerPath(
	// 	  bezierPoints, 
	// 	  new PathConstraints(
	// 		4.0, 4.0, 
	// 		Units.degreesToRadians(360), Units.degreesToRadians(540)
	// 	  ),  
	// 	  new GoalEndState(0.0, currentPose.getRotation())
	// 	);
  
	// 	AutoBuilder.followPathWithEvents(path).schedule();
	  

	}
	private void initializeAutonomousChooser() {
        // Example autonomous commands - replace with actual commands
        autoChooser1.setDefaultOption("Do Nothing", new SequentialCommandGroup());
        autoChooser1.addOption("Autonomous Path 1", new PathPlannerAuto("example"));
        autoChooser1.addOption("Autonomous Path 2", new PathPlannerAuto("pathName2"));
		
        // Add more autonomous options here

        // Put the chooser on the SmartDashboard
		
        SmartDashboard.putData("Auto Mode", autoChooser1);
    }
    
	// public static Command buildAuto(String pathName1, String pathName2, Command actionCommand) {
    // PathPlannerTrajectory trajectory1 = PathPlanner.loadPath(pathName1, maxVelocity, maxAcceleration);
    // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath(pathName2, maxVelocity, maxAcceleration);

    // SwerveControllerCommand path1Command = createSwerveControllerCommand(trajectory1);
    // SwerveControllerCommand path2Command = createSwerveControllerCommand(trajectory2);

    // return new SequentialCommandGroup(
    //     path1Command,
    //     actionCommand,
    //     path2Command
    // );


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// buildAuto("Score", "Score + Pickup", getAutonomousCommand())
		// return autoChooser1.getSelected();
		return autoChooser1.getSelected();
		// return Commands.sequence(
		// 	,
		// 	Commands.waitSeconds(0.1),
			
		// 	Commands.race(
		// 		Commands.waitSeconds(2),
		// 		Commands.run(
		// 			() -> drivetrain.drive(1, 0, 0, false, false),
		// 			drivetrain
		// 		)
		// 	)
		// );
	}
	

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public Trajectory createExampleTrajectory(TrajectoryConfig config) {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			config);

		return exampleTrajectory;
	}
	
	public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose());

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}

	public Field2d getField()
	{
		return field;
	}

	public Drivetrain getDrivetrain()
	{
		return drivetrain;
	}
}
