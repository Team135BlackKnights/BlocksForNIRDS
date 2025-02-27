package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.drive.Position;
import frc.robot.utils.maths.TimeUtil;

import java.util.List;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.util.DriveFeedforwards;

import java.util.HashMap;

public interface DrivetrainS extends Subsystem {
	/**
	 * Make the chassis go at a set speed
	 * 
	 * @param speeds the speed to set it to
	 */
	public static Field2d robotField = new Field2d();

	void setChassisSpeeds(ChassisSpeeds speeds);
	/**
	 * Swerve Only. Set the angles of the modules
	 */
	default Command orientModules(Rotation2d[] facings){
		throw new UnsupportedOperationException("Unimplemented method 'orientModules'");
	}
	/**
	 * @return the ChassisSpeeds of the drivetrain
	 */
	ChassisSpeeds getChassisSpeeds();

	/**
	 * Reset the drivetrain's odometry to a particular pose
	 * 
	 * @param pose the pose to set it to
	 */
	void resetPose(Pose2d pose);

	/**
	 * Apply a particular vision measurement to the drivetrain
	 * 
	 * @param pose       the pose returned by the vision estimate
	 * @param timestamp  the timestamp of the pose
	 * @param estStdDevs the estimated std dev (pose's difference from the mean
	 *                   in x, y, and theta)
	 */
	void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs);

	/**
	 * @return the pose of the robot
	 */
	Pose2d getPose();

	/**
	 * @apiNote This method is used to get the pose of the robot in the simulation
	 *          for SWERVE ONLY.
	 */
	default SwerveDriveKinematics getKinematics() {
		return null;
	}

	/**
	 * Stops the drivetrain
	 */
	void stopModules();

	/**
	 * @return the angle of the drivetrain as a rotation2d (in degrees)
	 */
	Rotation2d getRotation2d();

	/** Returns the current yaw velocity (Z rotation) in radians per second. */
	public default double getYawVelocity() {
		return 0;
	}

	/**
	 * Returns the measured X, Y, and theta field velocities in meters per sec.
	 * The components of the twist are velocities and NOT changes in position.
	 */
	public Twist2d getFieldVelocity();
	/**
	 * Reset the heading of the drivetrain
	 */
	void zeroHeading();

	/**
	 * @return if the gyro drivetrain is connected
	 */
	boolean isConnected();

	/**
	 * Checks gyros/accelerometers to check if a sudden movement has occured.
	 * 
	 * @return if a collision is detected
	 */
	boolean isCollisionDetected();

	/** Update the motor controllers to a specificed max amperage */
	public void setDriveCurrentLimit(int amps);

	HashMap<String, Double> getTemps();



	/**
	 * Create a position wrapper which contains the positions, and the
	 * timestamps.
	 * 
	 * @param <T>       The type of position, MechanumWheelPositions or
	 *                  SwerveModulePositions[] or tank's.
	 * @param positions with both a timestamp and position.
	 * @return
	 */
	default <T> Position<T> getPositionsWithTimestamp(T positions) {
		double timestamp = TimeUtil.getLogTimeSeconds();
		return new Position<>(positions, timestamp);
	}

	default double getCurrent() {
		return 0;
	}

	 void runWheelRadiusCharacterization(double velocity);
	 void runCharacterization(double velocity);
	 void endCharacterization();
	double getCharacterizationVelocity();
	void setPathplannerChassisSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards);

	 double[] getWheelRadiusCharacterizationPosition();

	default boolean[] isSkidding() {
		return new boolean[] { false, false, false, false
		};
	}
	default ModuleLimits getModuleLimits() {
		return null;
	}
	@Override
	default void periodic() {
		robotField.setRobotPose(getPose());

		SmartDashboard.putData(robotField);
	}
	default void setCurrentLimits(int amps){
		
	}
}
