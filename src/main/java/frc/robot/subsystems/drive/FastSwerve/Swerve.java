package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.GeometryConstants.ReefscapeGeometryScoring.BranchScoreLocation;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Setpoints.SwerveSetpointGenerator;
import frc.robot.subsystems.drive.FastSwerve.Setpoints.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.EqualsUtil;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.DriveConstants.SwerveModuleType;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCanivore;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.FieldConstants;

import java.util.*;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Swerve extends SubsystemChecker implements DrivetrainS {
	private static final LoggableTunedNumber coastWaitTime = new LoggableTunedNumber(
			"Drive/CoastWaitTimeSeconds", 0.5);
	private static final LoggableTunedNumber coastMetersPerSecThreshold = new LoggableTunedNumber(
			"Drive/CoastMetersPerSecThreshold", 0.25);
	private static final LoggableTunedNumber txTyObservationStaleSecs = new LoggableTunedNumber(
			"Odometry/TxTyObservationStaleSeconds", 0.125);
	private static final LoggableTunedNumber minDistanceTagPoseBlend = new LoggableTunedNumber(
			"Odometry/MinDistanceTagPoseBlend", Units.inchesToMeters(12.0));
	private static final LoggableTunedNumber maxDistanceTagPoseBlend = new LoggableTunedNumber(
			"Odometry/MaxDistanceTagPoseBlend", Units.inchesToMeters(60.0));

	public enum DriveMode {
		/** Driving with input from driver joysticks. (Default) */
		TELEOP,
		/** Driving based on a trajectory. */
		TRAJECTORY,
		/** Mathematical derivation of wheel radius, to account for ALL error */
		WHEEL_RADIUS_CHARACTERIZATION,
		/** Characterization of the drive motors kS/kV. */
		MODULE_CHARACTERIZATION
	}

	public enum CoastRequest {
		AUTOMATIC, ALWAYS_BRAKE, ALWAYS_COAST
	}

	private final OdometryThreadInputsAutoLogged odometryTimestampInputs;
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4];
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			DriveConstants.kModuleTranslations);
	// Store previous positions and time for filtering odometry data
	private SwerveModulePosition[] lastPositions = null;
	private double lastTime = 0.0;
	/** Active drive mode. */
	private DriveMode currentDriveMode = DriveMode.TELEOP;
	private boolean modulesOrienting = false;
	private final Timer lastMovementTimer = new Timer();
	@AutoLogOutput(key = "Drive/BrakeModeEnabled")
	private boolean brakeModeEnabled = false;
	@AutoLogOutput(key = "Drive/CoastRequest")
	private CoastRequest coastRequest = CoastRequest.AUTOMATIC;
	private boolean lastEnabled = false;
	private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
	private static final double poseBufferSizeSeconds = 2.0;
	private Pose2d odometryPose = new Pose2d();
	private Pose2d estimatedPose = new Pose2d();
	private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
			new ChassisSpeeds(),
			new SwerveModuleState[] { new SwerveModuleState(),
					new SwerveModuleState(), new SwerveModuleState(),
					new SwerveModuleState()
			},
			new boolean[] { false, false, false, false
			});
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
			.createBuffer(poseBufferSizeSeconds);

	public record OdometryObservation(SwerveModulePosition[] wheelPositions,
			Rotation2d gyroAngle, double timestamp) {
	}

	private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

	public record ModuleLimits(double maxDriveVelocity,
			double maxDriveAcceleration, double maxSteeringVelocity) {
	}

	public record VisionObservation(Pose2d visionPose, double timestamp,
			Matrix<N3, N1> stdDevs) {
	}

	public record TxTyObservation(
			int tagId, double tx, double ty, Pose3d robotToCam, double distance, double timestamp) {
	}

	public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {
	}

	private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] { new SwerveModulePosition(),
			new SwerveModulePosition(), new SwerveModulePosition(),
			new SwerveModulePosition() };

	private Rotation2d lastGyroAngle = new Rotation2d();
	private Twist2d robotVelocity = new Twist2d();
	private final SwerveSetpointGenerator setpointGenerator;
	private boolean collisionDetected;
	boolean[] isSkidding = new boolean[] { false, false, false, false
	};
	private final OdometryThread odometryThread;
	private double[] pathPlannerNM = new double[4];
	private double characterizationVelocity = 0.0;
	private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();
	private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
	static {
		for (int i = 1; i <= FieldConstants.aprilTagOffsets.length; i++) {
		  tagPoses2d.put(
			  i,
			  VisionConstants.kTagLayout
				  .getTagPose(i)
				  .map(Pose3d::toPose2d)
				  .orElse(new Pose2d()));
		}
	  }
	public Swerve(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl,
			ModuleIO br) {
		this.gyroIO = gyroIO;
		modules[0] = new Module(fl, 0);
		modules[1] = new Module(fr, 1);
		modules[2] = new Module(bl, 2);
		modules[3] = new Module(br, 3);
		lastMovementTimer.start();
		setBrakeMode(true);
		for (int i = 0; i < 3; ++i) {
			qStdDevs.set(i, 0, Math.pow(
					DriveConstants.TrainConstants.odometryStateStdDevs.get(i, 0),
					2));
		}
		setpointGenerator = new SwerveSetpointGenerator(kinematics,
				DriveConstants.kModuleTranslations);
		AutoBuilder.configure(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setPathplannerChassisSpeeds,
				DriveConstants.mainController,
				DriveConstants.mainConfig,
				() -> Robot.isRed, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});

		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		// SwerveDrive view
		SmartDashboard.putData("Swerve Drive", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

				builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

				builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

				builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

				builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
			}
		});
		setBrakeMode(true);
		registerSelfCheckHardware();
		this.odometryThread = OdometryThread.createInstance();
		this.odometryTimestampInputs = new OdometryThreadInputsAutoLogged();
		this.odometryThread.start();
	}

	/** Add odometry observation */
	public void addOdometryObservation(OdometryObservation observation) {
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions,
				observation.wheelPositions());
		lastWheelPositions = observation.wheelPositions();
		// Check gyro connected
		if (observation.gyroAngle != null) {
			// Update dtheta for twist if gyro connected
			twist = new Twist2d(twist.dx, twist.dy,
					observation.gyroAngle().minus(lastGyroAngle).getRadians());
			lastGyroAngle = observation.gyroAngle();
		} else {
			twist = new Twist2d(twist.dx, twist.dy, twist.dtheta);
		}
		// Add twist to odometry pose
		odometryPose = odometryPose.exp(twist);
		// Add pose to buffer at timestamp
		poseBuffer.addSample(observation.timestamp(), odometryPose);
		// Calculate diff from last odometry pose and add onto pose estimate
		estimatedPose = estimatedPose.exp(twist);
	}

	public void addVisionObservation(VisionObservation observation) {
		// If measurement is old enough to be outside the pose buffer's timespan, skip.
		try {
			if (poseBuffer.getInternalBuffer().lastKey()
					- poseBufferSizeSeconds > observation.timestamp()) {
				System.out.println("OUTSIDE BUFFER");
				return;
			}
		} catch (NoSuchElementException ex) {
			System.err.println("NO ELEMENT!");
			return;
		}
		// Get odometry based pose at timestamp
		var sample = poseBuffer.getSample(observation.timestamp());
		if (sample.isEmpty()) {
			// exit if not there
			return;
		}
		// sample --> odometryPose transform and backwards of that
		var sampleToOdometryTransform = new Transform2d(sample.get(),
				odometryPose);
		var odometryToSampleTransform = new Transform2d(odometryPose,
				sample.get());
		// get old estimate by applying odometryToSample Transform
		Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
		// Calculate 3 x 3 vision matrix
		var r = new double[3];
		for (int i = 0; i < 3; ++i) {
			r[i] = observation.stdDevs().get(i, 0)
					* observation.stdDevs().get(i, 0);
		}
		// Solve for closed form Kalman gain for continuous Kalman filter with A = 0
		// and C = I. See wpimath/algorithms.md.
		Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < 3; ++row) {
			double stdDev = qStdDevs.get(row, 0);
			if (stdDev == 0.0) {
				visionK.set(row, row, 0.0);
			} else {
				visionK.set(row, row,
						stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
			}
		}
		// difference between estimate and vision pose
		Transform2d transform = new Transform2d(estimateAtTime,
				observation.visionPose());
		// scale transform by visionK
		var kTimesTransform = visionK.times(VecBuilder.fill(transform.getX(),
				transform.getY(), transform.getRotation().getRadians()));
		Transform2d scaledTransform = new Transform2d(kTimesTransform.get(0, 0),
				kTimesTransform.get(1, 0),
				Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
		// Recalculate current estimate by applying scaled transform to old estimate
		// then replaying odometry data
		estimatedPose = estimateAtTime.plus(scaledTransform)
				.plus(sampleToOdometryTransform);
	}

	public void addTxTyObservation(TxTyObservation observation) {
		// Skip if current data for tag is newer
		if (txTyPoses.containsKey(observation.tagId())
				&& txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
			return;
		}

		// Get rotation at timestamp
		var sample = poseBuffer.getSample(observation.timestamp());
		if (sample.isEmpty()) {
			// exit if not there
			return;
		}
		Rotation2d robotRotation = estimatedPose.transformBy(new Transform2d(odometryPose, sample.get())).getRotation();

		// Use 3D distance and tag angles to find robot pose
		Translation2d camToTagTranslation = new Pose3d(Translation3d.kZero, new Rotation3d(0, -observation.ty, -observation.tx))
				.transformBy(
						new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
				.getTranslation()
				.rotateBy(new Rotation3d(0, observation.robotToCam.getRotation().getY(), 0))
				.toTranslation2d();
		Rotation2d camToTagRotation = robotRotation.plus(
			observation.robotToCam.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
		var tagPose2d = tagPoses2d.get(observation.tagId());
		if (tagPose2d == null)
			return;
		Translation2d fieldToCameraTranslation = new Pose2d(tagPose2d.getTranslation(),
				camToTagRotation.plus(Rotation2d.kPi))
				.transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
				.getTranslation();
		Pose2d robotPose = new Pose2d(
				fieldToCameraTranslation, robotRotation.plus(observation.robotToCam.toPose2d().getRotation()))
				.transformBy(new Transform2d(observation.robotToCam.toPose2d(), Pose2d.kZero));
		// Use gyro angle at time for robot rotation
		robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

		// Add transform to current odometry based pose for latency correction
		txTyPoses.put(
				observation.tagId(),
				new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
	}


	public void addVelocityData(Twist2d robotVelocity) {
		this.robotVelocity = robotVelocity;
	}

	/** Get 2d pose estimate of robot if not stale. */
	public Optional<Pose2d> getTxTyPose(int tagId) {
		if (!txTyPoses.containsKey(tagId)) {
			DriverStation.reportError("No tag with id: " + tagId, true);
			return Optional.empty();
		}
		var data = txTyPoses.get(tagId);
		// Check if stale
		if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
			return Optional.empty();
		}
		// Get odometry based pose at timestamp
		var sample = poseBuffer.getSample(data.timestamp());
		// Latency compensate
		Optional<Pose2d> tagPose = sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
		if (tagPose.isPresent()) {
			Logger.recordOutput("Odometry/TxTyPoseTag"+tagId, tagPose.get());
		}
		return tagPose;
	}
/**
   * Get estimated pose using txty data given tagId on reef and aligned pose on reef. Used for algae
   * intaking and coral scoring.
   */
  public Pose2d getReefPose(BranchScoreLocation branch, Pose2d finalPose) {
    final boolean isRed = Robot.isRed; //don't repeat this call to another class
	if (branch == BranchScoreLocation.BRANCH_ONE || branch == BranchScoreLocation.BRANCH_THREE || branch == BranchScoreLocation.BRANCH_FIVE || branch == BranchScoreLocation.BRANCH_SEVEN || branch == BranchScoreLocation.BRANCH_NINE || branch == BranchScoreLocation.BRANCH_ELEVEN){
	return estimatedPose;
	}
    var tagPose =
        getTxTyPose(
            switch (branch) {
			case BRANCH_ONE -> isRed ? 6 : 19;
			case BRANCH_TWELVE -> isRed ? 7 : 18;
			case BRANCH_TWO -> isRed ? 6 : 19;
			case BRANCH_THREE -> isRed ? 11 : 20;
			case BRANCH_FOUR -> isRed ? 11 : 20;
			case BRANCH_FIVE -> isRed ? 10 : 21;
			case BRANCH_SIX -> isRed ? 10 : 21;
			case BRANCH_SEVEN -> isRed ? 9 : 22;
			case BRANCH_EIGHT -> isRed ? 9 : 22;
			case BRANCH_NINE -> isRed ? 8 : 17;
			case BRANCH_TEN -> isRed ? 8 : 17;
			case BRANCH_ELEVEN -> isRed ? 7 : 18;
            });
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return estimatedPose;
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (getEstimatedPose().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return getEstimatedPose().interpolate(tagPose.get(), 1.0 - t);
  }
	@Override
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	public boolean[] calculateSkidding() {
		SwerveModuleState[] moduleStates = getModuleStates();
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
		// Step 1: Create a measured ChassisSpeeds object with solely the rotation
		// component
		ChassisSpeeds rotationOnlySpeeds = new ChassisSpeeds(0.0, 0.0,
				currentChassisSpeeds.omegaRadiansPerSecond + .05);
		double[] xComponentList = new double[4];
		double[] yComponentList = new double[4];
		// Step 2: Convert it into module states with kinematics
		SwerveModuleState[] rotationalStates = kinematics
				.toSwerveModuleStates(rotationOnlySpeeds);
		// Step 3: Subtract the rotational states from the module states to get the
		// translational vectors and calculate the magnitudes.
		// These should all be the same direction and magnitude if there is no skid.
		for (int i = 0; i < moduleStates.length; i++) {
			double deltaX = moduleStates[i].speedMetersPerSecond
					* Math.cos(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.cos(rotationalStates[i].angle.getRadians());
			double deltaY = moduleStates[i].speedMetersPerSecond
					* Math.sin(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.sin(rotationalStates[i].angle.getRadians());
			xComponentList[i] = deltaX;
			yComponentList[i] = deltaY;
		}
		// Step 4: Compare all of the translation vectors. If they aren't the same, skid
		// is present.
		Arrays.sort(xComponentList);
		Arrays.sort(yComponentList);
		double deltaMedianX = (xComponentList[1] + xComponentList[2]) / 2;
		double deltaMedianY = (yComponentList[1] + yComponentList[2]) / 2;
		boolean[] areModulesSkidding = new boolean[4];
		for (int i = 0; i < 4; i++) {
			double deltaX = xComponentList[i];
			double deltaY = yComponentList[i];
			if (Math.abs(deltaX - deltaMedianX) > DriveConstants.SKID_THRESHOLD
					|| Math.abs(
							deltaY - deltaMedianY) > DriveConstants.SKID_THRESHOLD) {
				areModulesSkidding[i] = true;
			} else {
				areModulesSkidding[i] = false;
			}
		}
		Logger.recordOutput("Drive/Skids", areModulesSkidding);
		return areModulesSkidding;
	}

	/**
	 * Reset estimated pose and odometry pose to pose <br>
	 * Clear pose buffer
	 */
	public void resetPose(Pose2d initialPose) {
		estimatedPose = initialPose;
		odometryPose = initialPose;
		poseBuffer.clear();
	}

	ModuleLimits currentModuleLimits = DriveConstants.moduleLimitsLow; // implement limiting based off what you
	// need

	@AutoLogOutput(key = "RobotState/FieldVelocity")
	@Override
	public Twist2d getFieldVelocity() {
		Translation2d linearFieldVelocity = new Translation2d(robotVelocity.dx,
				robotVelocity.dy).rotateBy(estimatedPose.getRotation());
		return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(),
				robotVelocity.dtheta);
	}

	/**
	 * Get the current pose of the robot with front being whatever front has been
	 * set to
	 * 
	 * @return a VISUAL ONLY output of the robot pose
	 * @see {@link #getPose() getPose} for the geometrically accurate pose
	 */
	@AutoLogOutput(key = "RobotState/EstimatedPose")
	public Pose2d getEstimatedPose() {
		return estimatedPose.plus(new Transform2d(new Translation2d(),
				DriveConstants.TrainConstants.robotOffsetAngleDirection));
	}

	@Override
	public ModuleLimits getModuleLimits() {
		return currentModuleLimits;
	}

	public void periodic() {
		// Check if modules are skidding
		// Update & process inputs
		odometryThread.lockOdometry();
		long inputTime = System.currentTimeMillis();
		odometryThread.updateInputs(odometryTimestampInputs);
		Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
		// Read inputs from gyro
		gyroIO.updateInputs(gyroInputs);
		gyroInputs.yawPosition = gyroInputs.yawPosition
				.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		for (Rotation2d value : gyroInputs.odometryYawPositions) {
			value.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		}
		Logger.processInputs("Drive/Gyro", gyroInputs);
		// Read inputs from modules
		Arrays.stream(modules).forEach(Module::updateInputs);
		odometryThread.unlockOdometry();
		Logger.recordOutput("SystemStatus/Periodic/DriveInputsMS",
				(System.currentTimeMillis() - inputTime));
		long systemTime = System.currentTimeMillis();
		// for each, see if we're disconnected
		for (Module module : modules) {
			if (!module.isDriveConnected()) {
				addFault("Drive Motor Disconnect on " + module.name, false, true);
			}
			if (!module.isTurnConnected()) {
				addFault("Turn Motor Disconnect on " + module.name, false, true);
			}
		}
		isSkidding = calculateSkidding();
		// Calculate the min odometry position updates across all modules
		int minOdometryUpdates = IntStream
				.of(odometryTimestampInputs.measurementTimeStamps.length,
						Arrays.stream(modules)
								.mapToInt(module -> module.getModulePositions().length)
								.min().orElse(0))
				.min().orElse(0);
		if (gyroInputs.connected) {
			minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length,
					minOdometryUpdates);
		}
		// Pass odometry data to robot state
		for (int i = 0; i < minOdometryUpdates; i++) {
			int odometryIndex = i;
			Rotation2d yaw = gyroInputs.connected
					? gyroInputs.odometryYawPositions[i]
					: null;
			// Get all four swerve module positions at that odometry update
			// and store in SwerveDriveWheelPositions object
			SwerveModulePosition[] wheelPositions = Arrays.stream(modules)
					.map(module -> module.getModulePositions()[odometryIndex])
					.toArray(SwerveModulePosition[]::new);
			// Filtering based on delta wheel positions
			boolean includeMeasurement = true;
			if (lastPositions != null) {
				double dt = odometryTimestampInputs.measurementTimeStamps[i] - lastTime;
				for (int j = 0; j < modules.length; j++) {
					double velocity = (wheelPositions[j].distanceMeters
							- lastPositions[j].distanceMeters) / dt;
					double omega = wheelPositions[j].angle
							.minus(lastPositions[j].angle).getRadians() / dt;
					// Check if delta is too large
					if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity()
							* 5.0
							|| Math.abs(velocity) > currentModuleLimits
									.maxDriveVelocity() * 5.0) {
						includeMeasurement = false;
						break;
					}
				}
			}
			// If delta isn't too large we can include the measurement.
			if (includeMeasurement) {
				lastPositions = wheelPositions;
				addOdometryObservation(new OdometryObservation(wheelPositions, yaw,
						odometryTimestampInputs.measurementTimeStamps[i]));
				lastTime = odometryTimestampInputs.measurementTimeStamps[i];
			}
		}
		// Update current velocities use gyro when possible
		ChassisSpeeds robotRelativeVelocity = getChassisSpeeds();
		robotRelativeVelocity.omegaRadiansPerSecond = gyroInputs.connected
				? gyroInputs.yawVelocityRadPerSec
				: robotRelativeVelocity.omegaRadiansPerSecond;
		addVelocityData(GeomUtil.toTwist2d(robotRelativeVelocity));
		// Update brake mode
		// Reset movement timer if moved
		if (Arrays.stream(modules)
				.anyMatch(module -> Math.abs(
						module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold
								.get())) {
			lastMovementTimer.reset();
		}
		if (DriverStation.isEnabled() && !lastEnabled) {
			coastRequest = CoastRequest.AUTOMATIC;
		}
		lastEnabled = DriverStation.isEnabled();
		// debug error
		switch (coastRequest) {
			case AUTOMATIC -> {
				if (DriverStation.isEnabled()) {
					setBrakeMode(true);
				} else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
					setBrakeMode(false);
				}
			}
			case ALWAYS_BRAKE -> {
				setBrakeMode(true);
			}
			case ALWAYS_COAST -> {
				setBrakeMode(false);
			}
		}
		switch (currentDriveMode) {
			case MODULE_CHARACTERIZATION -> {
				for (int i = 0; i < 4; i++) {
					modules[i].runCharacterization(0, characterizationVelocity);
				}
				break;
			}
			case WHEEL_RADIUS_CHARACTERIZATION -> {
				desiredSpeeds = new ChassisSpeeds(0, 0, characterizationVelocity);
				break;
			}
			default -> {
				break;
			}
		}
		// Shift
		if (DriveConstants.TrainConstants.RPMMatch.get() > getAverageRPM() && modules[0].inLowGear()) {
			Arrays.stream(modules).forEach(module -> module.shift(false));
		} else if (DriveConstants.TrainConstants.RPMMatch.get() < getAverageRPM() && !modules[0].inLowGear()) {
			Arrays.stream(modules).forEach(module -> module.shift(true));
		}
		if (DriveConstants.swerveModuleType == SwerveModuleType.SHIFTING_THIFTYSWERVE) {
			if (modules[0].inLowGear()) {
				// set max speed / acceleration for low gear
				currentModuleLimits = DriveConstants.moduleLimitsLow;
				DriveConstants.kMaxTurningSpeedRadPerSec = currentModuleLimits.maxSteeringVelocity;
				DriveConstants.kMaxSpeedMetersPerSecond = currentModuleLimits.maxDriveVelocity;
				DriveConstants.maxTranslationalAcceleration.initDefault(currentModuleLimits.maxDriveAcceleration);
				// don't change our rotational accel
				DriveConstants.pathConstraints = new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.maxTranslationalAcceleration.get(),
						DriveConstants.kMaxTurningSpeedRadPerSec,
						DriveConstants.maxRotationalAcceleration.get());
			} else {
				currentModuleLimits = DriveConstants.moduleLimitsHigh;
				DriveConstants.kMaxTurningSpeedRadPerSec = currentModuleLimits.maxSteeringVelocity;
				DriveConstants.kMaxSpeedMetersPerSecond = currentModuleLimits.maxDriveVelocity;
				DriveConstants.maxTranslationalAcceleration.initDefault(currentModuleLimits.maxDriveAcceleration);
				// don't change our rotational accel
				DriveConstants.pathConstraints = new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.maxTranslationalAcceleration.get(),
						DriveConstants.kMaxTurningSpeedRadPerSec,
						DriveConstants.maxRotationalAcceleration.get());
			}
		}
		// Run modules
		if (!modulesOrienting && currentDriveMode != DriveMode.MODULE_CHARACTERIZATION) {
			// Run robot at desiredSpeeds
			// Generate feasible next setpoint
			SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
			SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];
			currentSetpoint = setpointGenerator.generateSetpoint(
					currentModuleLimits, currentSetpoint, desiredSpeeds, .02);
			for (int i = 0; i < modules.length; i++) {
				// Optimize setpoints
				optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];
				if (currentDriveMode == DriveMode.TRAJECTORY) {
					optimizedSetpointTorques[i] = new SwerveModuleState(
							pathPlannerNM[i] * (currentSetpoint.flipped()[i] ? -1 : 1),
							optimizedSetpointStates[i].angle);
				} else {
					optimizedSetpointTorques[i] = new SwerveModuleState(0.0,
							optimizedSetpointStates[i].angle);
				}

				modules[i].runSetpoint(optimizedSetpointStates[i],
						optimizedSetpointTorques[i]);
			}
			Logger.recordOutput("Drive/SwerveStates/Setpoints",
					optimizedSetpointStates);
			Logger.recordOutput("Drive/SwerveStates/Torques",
					optimizedSetpointTorques);
		}
		Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
		Logger.recordOutput("Drive/SetpointSpeeds",
				currentSetpoint.chassisSpeeds());
		Logger.recordOutput("Drive/DriveMode", currentDriveMode);
		collisionDetected = collisionDetected();
		DrivetrainS.super.periodic();
		Logger.recordOutput("SystemStatus/Periodic/DriveProcessMS", (systemTime - System.currentTimeMillis()));
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		currentDriveMode = DriveMode.TELEOP;
		desiredSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
		// desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);
		for (int i = 0; i < 4; i++) {
			pathPlannerNM[i] = 0;
		}
	}

	@Override
	public void setPathplannerChassisSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
		currentDriveMode = DriveMode.TRAJECTORY;
		desiredSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
		double[] robotRelativeForcesXNewtons = feedforwards.robotRelativeForcesXNewtons();
		double[] robotRelativeForcesYNewtons = feedforwards.robotRelativeForcesYNewtons();
		// calculate angles at that chassis speed
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < 4; i++) {
			// Get the angle of the wheel in radians
			Rotation2d moduleAngleRadians = states[i].angle;
			Vector<N2> wheelDirection = VecBuilder.fill(moduleAngleRadians.getCos(), moduleAngleRadians.getSin());

			// Project the forces onto the module's direction of motion
			Vector<N2> moduleForce = new Translation2d(robotRelativeForcesXNewtons[i], robotRelativeForcesYNewtons[i])
					.rotateBy(Rotation2d.fromRadians(getRotation2d().getRadians()).unaryMinus())
					.toVector();

			double wheelTorqueNm = moduleForce.dot(wheelDirection)
					* (DriveConstants.TrainConstants.kWheelDiameter.get() / 2);

			// Calculate feedforward torque in Newton-meters
			pathPlannerNM[i] = wheelTorqueNm / DriveConstants.TrainConstants.kDriveMotorGearRatioLow; // TODO: Do we
																										// need to
																										// divide by
																										// gear ratio?
		}
		Logger.recordOutput("Swerve/xForces", feedforwards.robotRelativeForcesXNewtons());
		Logger.recordOutput("Swerve/yForces", feedforwards.robotRelativeForcesYNewtons());
	}

	/**
	 * Set brake mode to {@code enabled} doesn't change brake mode if already
	 * set.
	 */
	private void setBrakeMode(boolean enabled) {
		if (brakeModeEnabled != enabled) {
			Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
		}
		brakeModeEnabled = enabled;
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of
	 * the modules.
	 */
	@AutoLogOutput(key = "Drive/SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		return Arrays.stream(modules).map(Module::getState)
				.toArray(SwerveModuleState[]::new);
	}

	/**
	 * Returns the measured speeds of the robot in the robot's frame of
	 * reference.
	 */
	@AutoLogOutput(key = "Drive/MeasuredSpeeds")
	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp", modules[0].getDriveMotorTemp());
		tempMap.put("FLTurnTemp", modules[0].getTurnMotorTemp());
		tempMap.put("FRDriveTemp", modules[1].getDriveMotorTemp());
		tempMap.put("FRTurnTemp", modules[1].getTurnMotorTemp());
		tempMap.put("BLDriveTemp", modules[2].getDriveMotorTemp());
		tempMap.put("BLTurnTemp", modules[2].getTurnMotorTemp());
		tempMap.put("BRDriveTemp", modules[3].getDriveMotorTemp());
		tempMap.put("BRTurnTemp", modules[3].getTurnMotorTemp());
		return tempMap;
	}

	private void registerSelfCheckHardware() {
		// super.registerAllHardware(gyroIO.getSelfCheckingHardware());
		super.registerAllHardware(modules[0].getSelfCheckingHardware());
		super.registerAllHardware(modules[1].getSelfCheckingHardware());
		super.registerAllHardware(modules[2].getSelfCheckingHardware());
		super.registerAllHardware(modules[3].getSelfCheckingHardware());
		if (DriveConstants.robotMotorController == MotorVendor.CTRE_ON_CANIVORE) {
			super.registerAllHardware(List.of(new SelfCheckingCanivore(DriveConstants.canBusName)));
		}
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		for (Module module : modules) {
			List<SelfChecking> moduleHardware = module.getSelfCheckingHardware();
			for (SelfChecking motor : moduleHardware) {
				if (motor.getHardware() instanceof TalonFX) {
					orchestra.add((TalonFX) motor.getHardware());
				}
			}
		}
		return orchestra;
	}

	public boolean[] isSkidding() {
		return isSkidding;
	}

	@Override
	public double[] getWheelRadiusCharacterizationPosition() {
		return Arrays.stream(modules).mapToDouble(Module::getPositionRads)
				.toArray();
	}

	@Override
	@AutoLogOutput(key = "RobotState/Velocity")
	public double getCharacterizationVelocity() {
		double driveVelocityAverage = 0.0;
		for (var module : modules) {
			driveVelocityAverage += module.getCharacterizationVelocity();
		}
		return driveVelocityAverage / 4.0;
	}

	public double getAverageRPM() {
		double driveRPM = 0.0;
		for (var module : modules) {
			driveRPM += module.getDriveMotorRPM();
		}
		return driveRPM / 4.0;
	}

	@Override
	public void runWheelRadiusCharacterization(double velocity) {
		currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
		characterizationVelocity = velocity;
	}

	@Override
	public void runCharacterization(double input) {
		currentDriveMode = DriveMode.MODULE_CHARACTERIZATION;
		characterizationVelocity = input;
	}

	@Override
	public void endCharacterization() {
		currentDriveMode = DriveMode.TELEOP;
	}

	@Override
	public double getCurrent() {
		return modules[0].getCurrent() + modules[1].getCurrent()
				+ modules[2].getCurrent() + modules[3].getCurrent();
	}

	@Override
	public SystemStatus getTrueSystemStatus() {
		return getSystemStatus();
	}

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> setChassisSpeeds(new ChassisSpeeds(1, 0, 0)))
						.withTimeout(1.0), runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								// Retrieve the corresponding REVSwerveModule from the hashmap
								// Get the name of the current ModulePosition
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < .8
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.2) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow (wanted 1) for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								// angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 0) >= 10
										&& Math.abs(Math.abs(angle) - 180) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1, 0)))
								.withTimeout(1.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < .8
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.2) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow (wanted 1) for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								// angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 90) >= 10
										&& Math.abs(Math.abs(angle) - 270) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -2)))
								.withTimeout(2.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								double angle = moduleState.angle.getDegrees();

								String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
								double[][] validAngles = {
										{ 45, 225, -45, -225 }, // Front Left
										{ 135, 315, -135, -315 }, // Front Right
										{ 135, 315, -135, -315 }, // Back Left
										{ 45, 225, -45, -225 } // Back Right
								};

								boolean valid = false;
								for (double validAngle : validAngles[i]) {
									if (isWithinTolerance(angle, validAngle, 10)) {
										valid = true;
										break;
									}
								}

								if (!valid) {
									addFault(
											"[System Check] Turn angle off for " + moduleNames[i] + " "
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 2)))
								.withTimeout(2.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								double angle = moduleState.angle.getDegrees();

								String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
								double[][] validAngles = {
										{ 45, 225, -45, -225 }, // Front Left
										{ 135, 315, -135, -315 }, // Front Right
										{ 135, 315, -135, -315 }, // Back Left
										{ 45, 225, -45, -225 } // Back Right
								};

								boolean valid = false;
								for (double validAngle : validAngles[i]) {
									if (isWithinTolerance(angle, validAngle, 10)) {
										valid = true;
										break;
									}
								}

								if (!valid) {
									addFault(
											"[System Check] Turn angle off for " + moduleNames[i] + " "
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	private boolean isWithinTolerance(double value, double target, double tolerance) {
		return Math.abs(value - target) < tolerance;
	}

	@Override
	public void zeroHeading() {
		gyroIO.reset();
	}

	@Override
	public boolean isConnected() {
		return gyroInputs.connected;
	}

	private boolean collisionDetected() {
		return gyroInputs.collisionDetected;
	}

	@Override
	public boolean isCollisionDetected() {
		return collisionDetected;
	}

	/**
	 * Returns command that orients all modules to {@code orientation}, ending when
	 * the modules have
	 * rotated.
	 */
	public Command orientModules(Rotation2d orientation) {
		return orientModules(new Rotation2d[] { orientation, orientation, orientation, orientation });
	}

	/**
	 * Returns command that orients all modules to {@code orientations[]}, ending
	 * when the modules
	 * have rotated.
	 */
	@Override
	public Command orientModules(Rotation2d[] orientations) {
		return run(() -> {
			SwerveModuleState[] states = new SwerveModuleState[4];
			for (int i = 0; i < orientations.length; i++) {
				modules[i].runSetpoint(
						new SwerveModuleState(0.0, orientations[i]),
						new SwerveModuleState(0.0, new Rotation2d()));
				states[i] = new SwerveModuleState(0.0, modules[i].getAngle());
			}
			currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states, new boolean[4]);
		})
				.until(
						() -> Arrays.stream(modules)
								.allMatch(
										module -> EqualsUtil.epsilonEquals(
												module.getAngle().getDegrees(),
												module.getSetpointState().angle.getDegrees(),
												2.0)))
				.beforeStarting(() -> modulesOrienting = true)
				.finallyDo(() -> modulesOrienting = false)
				.withName("Orient Modules");
	}

	public static Rotation2d[] getXOrientations() {
		return Arrays.stream(DriveConstants.kModuleTranslations)
				.map(Translation2d::getAngle)
				.toArray(Rotation2d[]::new);
	}

	public static Rotation2d[] getCircleOrientations() {
		return Arrays.stream(DriveConstants.kModuleTranslations)
				.map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
				.toArray(Rotation2d[]::new);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		addVisionObservation(new VisionObservation(pose, timestamp, estStdDevs));
	}

	/**
	 * Get the current pose of the robot with front being the GEOMETRY front
	 * 
	 * @return an INTERNAL ONLY output of the robot pose (use this for any
	 *         driving/turning calculations)
	 * @see {@link #getEstimatedPose() getEstimatedPose} for the visually
	 *      accurate pose
	 */
	@Override
	public Pose2d getPose() {
		return estimatedPose;
	}

	@Override
	public void stopModules() {
		setChassisSpeeds(new ChassisSpeeds());
	}

	@Override
	public Rotation2d getRotation2d() {
		return getPose().getRotation();
	}

	@Override
	public void setDriveCurrentLimit(int amps) {
		Arrays.stream(modules).forEach(module -> module.setCurrentLimit(amps));
	}

	@Override
	public void setCurrentLimit(int amps) {
		setDriveCurrentLimit(amps);
	}
}
