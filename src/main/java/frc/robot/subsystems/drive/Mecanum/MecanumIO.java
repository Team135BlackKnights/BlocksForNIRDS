// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface MecanumIO {
	@AutoLog
	public static class MecanumIOInputs {
		public double leftFrontPositionRad = 0.0;
		public double leftFrontVelocityRadPerSec = 0.0;
		public double leftFrontAppliedVolts = 0.0;
		public double leftBackPositionRad = 0.0;
		public double leftBackVelocityRadPerSec = 0.0;
		public double leftBackAppliedVolts = 0.0;
		public double[] leftCurrentAmps = new double[] {};
		public double frontLeftDriveTemp = 0.0;
		public double backLeftDriveTemp = 0.0;
		public double rightFrontPositionRad = 0.0;
		public double rightFrontVelocityRadPerSec = 0.0;
		public double rightFrontAppliedVolts = 0.0;
		public double rightBackPositionRad = 0.0;
		public double rightBackVelocityRadPerSec = 0.0;
		public double rightBackAppliedVolts = 0.0;
		public double[] rightCurrentAmps = new double[] {};
		public double frontRightDriveTemp = 0.0;
		public double backRightDriveTemp = 0.0;
		public Rotation2d gyroYaw = new Rotation2d();
		public boolean gyroConnected = true;
		public boolean collisionDetected = false;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(MecanumIOInputs inputs) {}

	/** Run open loop at the specified voltage. */
	public default void setVoltage(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {}

	/** Resets the Gyro */
	public default void reset() {}

	/** Updates sim in SIM MODE */
	public default void updateSim(double dtSeconds) {}

	/** Update the motor controllers to a specificed max amperage */
	public default void setCurrentLimit(int amps) {}

	/** Run closed loop at the specified velocity. */
	public default void setVelocity(double frontLeftRadPerSec,
			double frontRightRadPerSec, double backLeftRadPerSec,
			double backRightRadPerSec, double frontLeftFFVolts,
			double frontRightFFVolts, double backLeftFFVolts,
			double backRightFFVolts) {}

}
