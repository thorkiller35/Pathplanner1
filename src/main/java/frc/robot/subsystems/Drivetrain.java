// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the function of the drivetrain.
 */
public class Drivetrain extends SubsystemBase {
	private static Drivetrain instance;
	public static synchronized Drivetrain getInstance() {
		if (instance == null) instance = new Drivetrain();
		return instance;
	  }
	

	public static final double mod1 = Units.degreesToRadians(329.853515625); // adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double mod3 = Units.degreesToRadians(326.513671875-180); // adjust as needed so that virtual (turn) position of wheel is zero when straight //326.07421875
	public static final double mod0 = Units.degreesToRadians(177.01171875); // adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double mod2 = Units.degreesToRadians(305.15625-180); // adjust as needed so that virtual (turn) position of wheel is zero when straight -82.70507813  97.29492187
	

	//public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = Units.degreesToRadians(324.755859375); // adjust as needed so that virtual (turn) position of wheel is zero when straight
	//public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = Units.degreesToRadians(141.6796875); // adjust as needed so that virtual (turn) position of wheel is zero when straight //326.07421875
	//public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = Units.degreesToRadians(166.81640625); // adjust as needed so that virtual (turn) position of wheel is zero when straight
	//public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = Units.degreesToRadians(100.0625); // adjust as needed so that virtual (turn) position of wheel is zero when straight -82.70507813  97.29492187

	public static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed

	public static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	public static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in

	// Create SwerveModules
	private final SwerveModule m_frontLeft = new SwerveModule(
		Ports.CAN.FRONT_LEFT_DRIVING,
		Ports.CAN.FRONT_LEFT_TURNING,
		Ports.Analog.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_frontRight = new SwerveModule(
		Ports.CAN.FRONT_RIGHT_DRIVING,
		Ports.CAN.FRONT_RIGHT_TURNING,
		Ports.Analog.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_rearLeft = new SwerveModule(
		Ports.CAN.REAR_LEFT_DRIVING,
		Ports.CAN.REAR_LEFT_TURNING,
		Ports.Analog.REAR_LEFT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_rearRight = new SwerveModule(
		Ports.CAN.REAR_RIGHT_DRIVING,
		Ports.CAN.REAR_RIGHT_TURNING,
		Ports.Analog.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER);

	// The gyro sensor
	private final AHRS m_gyro = new AHRS(Port.kMXP); 

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.Swerve.flModuleOffset, 
      Constants.Swerve.frModuleOffset, 
      Constants.Swerve.blModuleOffset, 
      Constants.Swerve.brModuleOffset
    );
	

	// Odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
		kinematics,
		Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
		new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_frontLeft.getPosition(),
			m_frontRight.getPosition()
		});
	

	/** Creates a new Drivetrain. */
	public Drivetrain() {
		m_frontLeft.calibrateVirtualPosition(mod1); // set virtual position for absolute encoder
		m_frontRight.calibrateVirtualPosition(mod3);
		m_rearLeft.calibrateVirtualPosition(mod0);
		m_rearRight.calibrateVirtualPosition(mod2);
		

		m_frontLeft.resetEncoders(); // resets relative encoders
		m_frontRight.resetEncoders();
		m_rearLeft.resetEncoders();
		m_rearRight.resetEncoders();

		zeroHeading(); // resets gyro

		// sets initial pose arbitrarily
		// Note: the field coordinate system (or global coordinate system) is an absolute coordinate system where a point on the field is designated as the origin.
		// Positive theta is in the counter-clockwise direction, and the positive x-axis points away from your alliance’s driver station wall,
		// and the positive y-axis is perpendicular and to the left of the positive x-axis.
		Translation2d initialTranslation = new Translation2d(Units.inchesToMeters(FIELD_LENGTH_INCHES/2),Units.inchesToMeters(FIELD_WIDTH_INCHES/2)); // mid field
		Rotation2d initialRotation = new Rotation2d(); 
		Pose2d initialPose = new Pose2d(initialTranslation,initialRotation);
		resetOdometry(initialPose);



	AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.Swerve.pathFollowerConfig, 
      this
    );
		
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
			Rotation2d.fromDegrees( m_gyro.getAngle()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_frontLeft.getPosition(),
				m_frontRight.getPosition()
			});
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(
			Rotation2d.fromDegrees( m_gyro.getAngle()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_frontLeft.getPosition(),
				m_frontRight.getPosition()
			},
			pose);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
		
		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}
			

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
			else if (angleDif > 0.85*Math.PI) {
				if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				}
				else {
					m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			}
			else {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;
			
			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

		var swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees( m_gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);

		;
		
		

	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
	
		SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
		setModuleStates(targetStates);
	  }

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0 and seeds the turn encoders using the absolute encoders. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_rearLeft.getPosition(),
			m_rearRight.getPosition()
		};
	}

	public void resetPose(final Pose2d pose2d) {
		m_gyro.setAngleAdjustment(pose2d.getRotation().getDegrees());
		m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose2d);
	}

	public ChassisSpeeds getSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	  }

	public SwerveModuleState[] getModuleStates() {
    	return new SwerveModuleState[]{
			m_frontLeft.getState(),
			m_frontRight.getState(),
			m_rearLeft.getState(),
			m_rearRight.getState()

		};
	  }	


	

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees( m_gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public SwerveModule getFrontLeftModule()
	{
		return m_frontLeft;
	}

	public SwerveModule getFrontRightModule()
	{
		return m_frontRight;
	}

	public SwerveModule getRearLeftModule()
	{
		return m_rearLeft;
	}

	public SwerveModule getRearRightModule()
	{
		return m_rearRight;
	}

	public AHRS getImu()
	{
		return m_gyro;
	}

	
	/**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location Pose to make the current pose at.
     */
   
    

   
}
