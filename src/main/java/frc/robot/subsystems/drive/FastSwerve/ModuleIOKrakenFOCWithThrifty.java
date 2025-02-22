package frc.robot.subsystems.drive.FastSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class ModuleIOKrakenFOCWithThrifty implements ModuleIO {
	// Hardware
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final AnalogInput turnAbsoluteEncoder;
	private final Rotation2d absoluteEncoderOffset;
	private final String driveName;
	private final String turnName;
	// Status Signals
	private final StatusSignal<Angle> drivePosition;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveAppliedVolts;
	private final StatusSignal<Current> driveSupplyCurrent;
	private final StatusSignal<Current> driveTorqueCurrent;
	private final StatusSignal<Temperature> driveTemp;
	private final StatusSignal<Angle> turnPosition;
	private final StatusSignal<AngularVelocity> turnVelocity;
	private final StatusSignal<Voltage> turnAppliedVolts;
	private final StatusSignal<Current> turnSupplyCurrent;
	private final StatusSignal<Current> turnTorqueCurrent;
	private final StatusSignal<Temperature> turnTemp;
	// Odometry Queues
	private final Queue<Double> drivePositionQueue;
	private final Queue<Double> turnPositionQueue;
	// Controller Configs
	private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
	private static final Executor brakeModeExecutor = Executors
			.newFixedThreadPool(8);
	// Control
	private final VoltageOut voltageControl = new VoltageOut(0);
	private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
	private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(
			0);
	private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0.0);
	private final NeutralOut neutralControl = new NeutralOut();
	private final boolean isTurnMotorInverted;
	private final boolean isDriveMotorInverted;
	private final boolean isTurnSensorInverted;

	/**
	 * @apiNote CANCoder offsets SHOULD be set to zero in code due to how the
	 *          user manual works
	 */
	@SuppressWarnings("unused")
	public ModuleIOKrakenFOCWithThrifty(int index) {
		// Init controllers and encoders from config constants
		switch (index) {
			case 0:
				if (DriveConstants.canBusName == "") {
					driveTalon = new TalonFX(DriveConstants.kFrontLeftDrivePort);
					turnTalon = new TalonFX(DriveConstants.kFrontLeftTurningPort);

				} else {
					driveTalon = new TalonFX(DriveConstants.kFrontLeftDrivePort, DriveConstants.canBusName);
					turnTalon = new TalonFX(DriveConstants.kFrontLeftTurningPort, DriveConstants.canBusName);
				}
				turnAbsoluteEncoder = new AnalogInput(DriveConstants.kFrontLeftAbsEncoderPort);
				driveName = "FrontLeftDrive";
				turnName = "FrontLeftTurn";
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kFrontLeftAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kFrontLeftDriveReversed;
				isTurnMotorInverted = DriveConstants.kFrontLeftTurningReversed;
				isTurnSensorInverted = DriveConstants.kFrontLeftAbsEncoderReversed;
				break;
			case 1:
				if (DriveConstants.canBusName == "") {
					driveTalon = new TalonFX(DriveConstants.kFrontRightDrivePort);
					turnTalon = new TalonFX(DriveConstants.kFrontRightTurningPort);
				} else {
					driveTalon = new TalonFX(DriveConstants.kFrontRightDrivePort, DriveConstants.canBusName);
					turnTalon = new TalonFX(DriveConstants.kFrontRightTurningPort, DriveConstants.canBusName);
				}
				turnAbsoluteEncoder = new AnalogInput(
						DriveConstants.kFrontRightAbsEncoderPort);
				driveName = "FrontRightDrive";
				turnName = "FrontRightTurn";
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kFrontRightAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kFrontRightDriveReversed;
				isTurnMotorInverted = DriveConstants.kFrontRightTurningReversed;
				isTurnSensorInverted = DriveConstants.kFrontRightAbsEncoderReversed;
				break;
			case 2:
				if (DriveConstants.canBusName == "") {
					driveTalon = new TalonFX(DriveConstants.kBackLeftDrivePort);
					turnTalon = new TalonFX(DriveConstants.kBackLeftTurningPort);
				} else {
					driveTalon = new TalonFX(DriveConstants.kBackLeftDrivePort, DriveConstants.canBusName);
					turnTalon = new TalonFX(DriveConstants.kBackLeftTurningPort, DriveConstants.canBusName);
				}
				turnAbsoluteEncoder = new AnalogInput(
						DriveConstants.kBackLeftAbsEncoderPort);
				driveName = "BackLeftDrive";
				turnName = "BackLeftTurn";
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kBackLeftAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kBackLeftDriveReversed;
				isTurnMotorInverted = DriveConstants.kBackLeftTurningReversed;
				isTurnSensorInverted = DriveConstants.kBackLeftAbsEncoderReversed;
				break;
			case 3:
				if (DriveConstants.canBusName == "") {
					driveTalon = new TalonFX(DriveConstants.kBackRightDrivePort);
					turnTalon = new TalonFX(DriveConstants.kBackRightTurningPort);
				} else {
					driveTalon = new TalonFX(DriveConstants.kBackRightDrivePort, DriveConstants.canBusName);
					turnTalon = new TalonFX(DriveConstants.kBackRightTurningPort, DriveConstants.canBusName);
				}
				driveName = "BackRightDrive";
				turnName = "BackRightTurn";
				turnAbsoluteEncoder = new AnalogInput(
						DriveConstants.kBackRightAbsEncoderPort);
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kBackRightAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kBackRightDriveReversed;
				isTurnMotorInverted = DriveConstants.kBackRightTurningReversed;
				isTurnSensorInverted = DriveConstants.kBackRightAbsEncoderReversed;
				break;
			default:
				throw new RuntimeException("Invalid module index");
		}
		// Config Motors
		driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.kMaxDriveCurrent;
		driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.kMaxDriveCurrent;
		driveTalonConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kMaxDriveCurrent;
		driveTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
		driveTalonConfig.MotorOutput.Inverted = isDriveMotorInverted
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		turnTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.kMaxTurnCurrent;
		turnTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.kMaxTurnCurrent;
		turnTalonConfig.MotorOutput.Inverted = isTurnMotorInverted
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		// Conversions affect getPosition()/setPosition() and getVelocity()
		driveTalonConfig.Feedback.SensorToMechanismRatio = DriveConstants.TrainConstants.kTurningMotorGearRatio;
		turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
		turnTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0
				/ DriveConstants.TrainConstants.kTurningMotorGearRatio;
		turnTalonConfig.MotionMagic.MotionMagicAcceleration = turnTalonConfig.MotionMagic.MotionMagicCruiseVelocity
				/ DriveConstants.overallTurningMotorConstantContainer.getKa();
		turnTalonConfig.MotionMagic.MotionMagicExpo_kV = DriveConstants.overallTurningMotorConstantContainer.getKv()
				* DriveConstants.TrainConstants.kTurningMotorGearRatio;
		turnTalonConfig.MotionMagic.MotionMagicExpo_kA = DriveConstants.overallTurningMotorConstantContainer.getKa();
		// Apply configs
		for (int i = 0; i < 4; i++) {
			boolean error = driveTalon.getConfigurator().apply(driveTalonConfig,
					0.1) == StatusCode.OK;
			error = error && (turnTalon.getConfigurator().apply(turnTalonConfig,
					0.1) == StatusCode.OK);
			if (!error)
				break;
		}
		// 250hz signals
		drivePosition = driveTalon.getPosition();
		turnPosition = turnTalon.getPosition();
		BaseStatusSignal.setUpdateFrequencyForAll(250, drivePosition,
				turnPosition);
		drivePositionQueue = OdometryThread
				.registerSignalInput(driveTalon.getPosition());
		turnPositionQueue = OdometryThread
				.registerInput(() -> {
					double absolutePositionPercent = turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
					if (isTurnSensorInverted) {
						absolutePositionPercent = 1 - absolutePositionPercent;
					}
					return new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
							.minus(absoluteEncoderOffset).getRadians();
				});
		// Get signals and set update rate
		// 100hz signals
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveSupplyCurrent = driveTalon.getSupplyCurrent();
		driveTorqueCurrent = driveTalon.getTorqueCurrent();
		driveTemp = driveTalon.getDeviceTemp();

		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnSupplyCurrent = turnTalon.getSupplyCurrent();
		turnTorqueCurrent = turnTalon.getTorqueCurrent();
		turnTemp = turnTalon.getDeviceTemp();
		BaseStatusSignal.setUpdateFrequencyForAll(100.0, driveVelocity,
				driveAppliedVolts, driveSupplyCurrent, driveTorqueCurrent,
				turnVelocity, turnAppliedVolts, turnSupplyCurrent,
				turnTorqueCurrent);
		// Reset turn position to absolute encoder position
		// get absolute position from analog encoder
		double absolutePositionPercent = turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
		if (isTurnSensorInverted) {
			absolutePositionPercent = 1 - absolutePositionPercent;
		}
		turnTalon.setPosition(
				Rotation2d.fromRotations(absolutePositionPercent * 2.0 * Math.PI)
						.minus(absoluteEncoderOffset).getRotations(),
				1.0);
		// Optimize bus utilization
		driveTalon.optimizeBusUtilization(0, 1.0);
		turnTalon.optimizeBusUtilization(0, 1.0);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.hasCurrentControl = true;
		inputs.driveMotorConnected = BaseStatusSignal
				.refreshAll(drivePosition, driveVelocity, driveAppliedVolts,
						driveSupplyCurrent, driveTorqueCurrent, driveTemp)
				.isOK();
		inputs.turnMotorConnected = BaseStatusSignal.refreshAll(turnPosition,
				turnVelocity, turnAppliedVolts, turnSupplyCurrent,
				turnTorqueCurrent, turnTemp).isOK();
		inputs.drivePositionRads = Units
				.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveVelocityRadsPerSec = Units
				.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
		inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
		inputs.driveMotorTemp = driveTemp.getValueAsDouble();
		// get absolute position from analog encoder
		double absolutePositionPercent = turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
		if (isTurnSensorInverted) {
			absolutePositionPercent = 1 - absolutePositionPercent;
		}
		inputs.turnAbsolutePosition = new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
				.minus(absoluteEncoderOffset);
		inputs.turnPosition = Rotation2d
				.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnVelocityRadsPerSec = Units
				.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
		inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
		inputs.turnMotorTemp = turnTemp.getValueAsDouble();
		inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
				.mapToDouble(signalValue -> Units.rotationsToRadians(signalValue)
						* (DriveConstants.TrainConstants.kWheelDiameter.get() / 2))
				.toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream()
				.map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void runDriveVolts(double volts) {
		driveTalon.setControl(voltageControl.withOutput(volts));
	}

	@Override
	public void runTurnVolts(double volts) {
		turnTalon.setControl(voltageControl.withOutput(volts));
	}

	@Override
	public void runCharacterization(double input) {
		driveTalon.setControl(currentControl.withOutput(input));
	}

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		driveTalon.setControl(velocityTorqueCurrentFOC
				.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
				.withFeedForward(feedForward).withOverrideCoastDurNeutral(false));
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		turnTalon.setControl(
				positionControl.withPosition(Units.radiansToRotations(angleRads)).withOverrideCoastDurNeutral(false));
	}

	@Override
	public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
		driveTalonConfig.Slot0.kP = kP;
		driveTalonConfig.Slot0.kI = kI;
		driveTalonConfig.Slot0.kD = kD;
		driveTalonConfig.Slot0.kS = kS;
		driveTalonConfig.Slot0.kV = kV;
		driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD, double kS, double kV, double deadbandAmps) {
		turnTalonConfig.Slot0.kP = kP;
		turnTalonConfig.Slot0.kI = kI;
		turnTalonConfig.Slot0.kD = kD;
		turnTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
		turnTalonConfig.Slot0.kS = kS;
		turnTalonConfig.Slot0.kV = kV;
		turnTalonConfig.TorqueCurrent.TorqueNeutralDeadband = deadbandAmps;
		turnTalon.getConfigurator().apply(turnTalonConfig, 0.01);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		brakeModeExecutor.execute(() -> {
			synchronized (driveTalonConfig) {
				driveTalonConfig.MotorOutput.NeutralMode = enable
						? NeutralModeValue.Brake
						: NeutralModeValue.Coast;
				driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
			}
		});
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		brakeModeExecutor.execute(() -> {
			synchronized (turnTalonConfig) {
				turnTalonConfig.MotorOutput.NeutralMode = enable
						? NeutralModeValue.Brake
						: NeutralModeValue.Coast;
				turnTalon.getConfigurator().apply(turnTalonConfig, 0.25);
			}
		});
	}

	@Override
	public void setCurrentLimit(int amps) {
		brakeModeExecutor.execute(() -> {
			synchronized (driveTalonConfig) {
				driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = amps;
				driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -amps;
				driveTalon.getConfigurator().apply(driveTalonConfig, .25);
			}
		});
	}

	@Override
	public void stop() {
		driveTalon.setControl(neutralControl);
		turnTalon.setControl(neutralControl);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX(driveName, driveTalon));
		hardware.add(new SelfCheckingTalonFX(turnName, turnTalon));
		return hardware;
	}
}
