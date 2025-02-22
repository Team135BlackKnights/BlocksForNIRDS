// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.utils.drive.DriveConstants;
import java.util.Queue;

/**
 * IO implementation for navX
 * 
 * @apiNote Needs to be tested
 */
public class GyroIONavX implements GyroIO {
	private final AHRS navX = new AHRS(NavXComType.kUSB1);
	private final Queue<Double> yawPositionQueue;
	private double last_world_linear_accel_x, last_world_linear_accel_y,
			current_angle_position, last_angle_position = 0;

	public GyroIONavX() {
		yawPositionQueue =  OdometryThread.registerInput(navX::getAngle);
	}

	@Override
	public void reset() { navX.reset(); }

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navX.isConnected();
		current_angle_position = navX.getAngle();
		inputs.yawPosition = Rotation2d.fromDegrees(current_angle_position);
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(
				(current_angle_position - last_angle_position) / 250);
		last_angle_position = current_angle_position;
		inputs.odometryYawPositions = yawPositionQueue.stream()
				.map((Double value) -> Rotation2d.fromDegrees(value))
				.toArray(Rotation2d[]::new);
		yawPositionQueue.clear();
		double curr_world_linear_accel_x = navX.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = navX.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			inputs.collisionDetected = true;
		}else{
			inputs.collisionDetected = false;
		}
	}
}
