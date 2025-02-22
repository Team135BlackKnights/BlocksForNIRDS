// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.DrivetrainC;

import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import frc.robot.subsystems.drive.Mecanum.MecanumIO;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSparkBase;
import frc.robot.subsystems.drive.Mecanum.MecanumIOTalonFX;
import frc.robot.subsystems.drive.FastSwerve.ModuleIO;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIOTalonFX;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.utils.RunTest;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	private final SendableChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			yButtonDrive = new JoystickButton(driveController, 4), //used for Aim/Drive to pose
			bButtonDrive = new JoystickButton(driveController, 2),
			aButtonDrive = new JoystickButton(driveController, 1),
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	public static int currentTest = 0;
		@AutoLogOutput(key = "RobotState/currentPath")
	public static String currentPath = "";
	public static Field2d field = new Field2d();
	public enum GamePieceState {
		NO_GAME_PIECE, HAS_NOTE, ABORT
	}
	public static GamePieceState currentGamePieceStatus = GamePieceState.NO_GAME_PIECE;
	public static boolean userDrive = true;
	public static Command currentAuto;

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands. y * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		//Publish the current mode of the robot (to check in  Advantage Scope)
		Logger.recordOutput("robotMode", Constants.currentMode);

		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
		List<Pair<String, Command>> autoCommands = new ArrayList<>();
		switch (Constants.currentMode) {
		case REAL:
			switch (DriveConstants.driveType) {
			case SWERVE:
				switch (DriveConstants.robotMotorController) {
				case CTRE_ON_RIO:
				case CTRE_ON_CANIVORE:
					switch (DriveConstants.gyroType) {
					case NAVX:
						drivetrainS = new Swerve(new GyroIONavX(),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
						break;
					case PIGEON:
						drivetrainS = new Swerve(new GyroIOPigeon2(),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
						break;
					default:
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case NAVX:
						drivetrainS = new Swerve(new GyroIONavX(),
								new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
								new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
						break;
					case PIGEON:
						drivetrainS = new Swerve(new GyroIOPigeon2(),
								new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
								new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
					default:
						break;
					}
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				switch (DriveConstants.robotMotorController) {
				case CTRE_ON_RIO:
				case CTRE_ON_CANIVORE:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Tank(
								new TankIOTalonFX(new GyroIOPigeon2()));
						break;
					case NAVX:
						drivetrainS = new Tank(new TankIOTalonFX(new GyroIONavX()));
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Tank(
								new TankIOSparkBase(new GyroIOPigeon2()));
						break;
					case NAVX:
						drivetrainS = new Tank(new TankIOSparkBase(new GyroIONavX()));
						break;
					}
					break;
				}
				break;
			case MECANUM:
				switch (DriveConstants.robotMotorController) {
				case CTRE_ON_RIO:
				case CTRE_ON_CANIVORE:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Mecanum(
								new MecanumIOTalonFX(new GyroIOPigeon2()));
						break;
					case NAVX:
						drivetrainS = new Mecanum(
								new MecanumIOTalonFX(new GyroIONavX()));
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Mecanum(
								new MecanumIOSparkBase(new GyroIOPigeon2()));
						break;
					case NAVX:
						drivetrainS = new Mecanum(
								new MecanumIOSparkBase(new GyroIONavX()));
						break;
					}
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			//Placeholder values
			default:
				throw new IllegalArgumentException(
						"Unknown implementation type, please check DriveConstants.java!");
			}
			autoCommands.addAll(Arrays.asList(
					//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
			//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
			//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
			));
			break;
		default:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIO() {},
						new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				drivetrainS = new Tank(new TankIO() {});
				break;
			case MECANUM:
				drivetrainS = new Mecanum(new MecanumIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
			}
			autoCommands.addAll(Arrays.asList(
					//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
			//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
			//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
			));
		}
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		Pathfinding.setPathfinder(new LocalADStarAK());
		NamedCommands.registerCommands(autoCommands);
		PathfindingCommand.warmupCommand().schedule();
		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}
		PathfindingCommand.warmupCommand()
				.andThen(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0))
				.finallyDo(() -> RobotContainer.field.getObject("target pose")
						.setPose(new Pose2d(-50, -50, new Rotation2d())))
				.schedule();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(auto -> {
			try {
				currentAuto = auto;
				field.getObject("path")
						.setPoses(PathFinder.parseAutoToPose2dList(auto.getName()));
			}
			catch (Exception e) {
				System.err.println("NO FOUND PATH FOR DESIRED AUTO!!");
				field.getObject("path").setPoses(
						new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
								new Pose2d(-50.2, -50, new Rotation2d())
				});
			}
		});
		// Configure the trigger bindings
		configureBindings();
		addNTCommands();
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		// Some condition that should decide if we want to override rotation
		return angleOverrider;
	}

	private void configureBindings() {
		xButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.onTrue(new InstantCommand(() -> drivetrainS.zeroHeading()));
		yButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, true));
		bButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, true));
		aButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, false));
		xButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, false));
		//Example Drive To 2024 Amp Pose, Bind to what you need.
		yButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0));
		//swerve DRIVE tests
		//When user hits right bumper, go to next test, or wrap back to starting test for SysID.
		rightBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == Constants.SysIdRoutines.values().length - 1) {
				currentTest = 0;
				System.out.println("looping");
			} else {
				currentTest++;
			}
		}));
		//When user hits left bumper, go to next test, or wrap back to starting test for SysID.
		leftBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == 0) {
				currentTest = Constants.SysIdRoutines.values().length - 1;
				System.out.println("looping");
			} else {
				currentTest--;
			}
		}));
		//When using CTRE, be sure to hit Start so that the motors are logged via CTRE (For SysId)
		selectButtonTest.onTrue(Commands.runOnce(SignalLogger::stop));
		startButtonTest.onTrue(Commands.runOnce(SignalLogger::start));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	/**
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {
		return new double[] { Math.min(drivetrainS.getCurrent(), 200)
		};
	}

	private static void addNTCommands() {
		SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
	}

	/**
	 * RUN EACH system's test command. Does NOT run any checks on vision.
	 * 
	 * @return a command with all of them in a sequence.
	 */
	public static Command allSystemsCheck() {
		return Commands.sequence(drivetrainS.getRunnableSystemCheckCommand());
	}

	public static HashMap<String, Double> combineMaps(
			List<HashMap<String, Double>> maps) {
		HashMap<String, Double> combinedMap = new HashMap<>();
		// Iterate over the list of maps
		for (HashMap<String, Double> map : maps) {
			combinedMap.putAll(map);
		}
		return combinedMap;
	}

	public static HashMap<String, Double> getAllTemps() {
		// List of HashMaps
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps());
		// Combine all maps
		HashMap<String, Double> combinedMap = combineMaps(maps);
		return combinedMap;
	}


	public static Collection<ParentDevice> getOrchestraDevices() {
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		return devices;
	}

	public static Subsystem[] getAllSubsystems() {
		Subsystem[] subsystems = new Subsystem[1];
		subsystems[0] = drivetrainS;
		return subsystems;
	}
}