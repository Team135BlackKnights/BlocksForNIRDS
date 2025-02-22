package frc.robot.utils.drive.Sensors;



import org.littletonrobotics.junction.AutoLog;



public interface ColorSensorIO {
		@AutoLog
	
	public static class ColorSensorIOInputs{
		public String colorOutput;
		public double proximityCentimeters;
	}

	public default void updateInputs(ColorSensorIOInputs inputs) {}


	
}

