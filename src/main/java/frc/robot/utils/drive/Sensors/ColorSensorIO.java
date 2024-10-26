package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface ColorSensorIO {
		@AutoLog
	
	public static class ColorSensorIOInputs{
		public String colorOutput;
		public double proximityCentimeters;
	}

	public default void updateInputs(ColorSensorIOInputs inputs) {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
	
}

