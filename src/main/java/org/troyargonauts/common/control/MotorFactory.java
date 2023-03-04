package org.troyargonauts.common.control;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import org.troyargonauts.common.control.motorcontrol.LazySparkMax;
import org.troyargonauts.common.control.motorcontrol.LazyTalonFX;
import org.troyargonauts.common.control.motorcontrol.LazyTalonSRX;

public final class MotorFactory {

	private MotorFactory() {}

	// Spark Configurations
	public static class SparkConfiguration {
		public boolean BURN_FACTORY_DEFAULT_FLASH;
		public CANSparkMax.IdleMode IDLE_MODE;
		public boolean INVERTED;

		public int STATUS_FRAME_0_RATE_MS;
		public int STATUS_FRAME_1_RATE_MS;
		public int STATUS_FRAME_2_RATE_MS;

		public double OPEN_LOOP_RAMP_RATE;
		public double CLOSED_LOOP_RAMP_RATE;

		public boolean ENABLE_VOLTAGE_COMPENSATION;
		public double NOMINAL_VOLTAGE;
	}

	public static final SparkConfiguration DEFAULT_SPARK_CONFIG = new SparkConfiguration() {
		{
			BURN_FACTORY_DEFAULT_FLASH = false;
			IDLE_MODE = CANSparkMax.IdleMode.kCoast;
			INVERTED = false;
			STATUS_FRAME_0_RATE_MS = 10;
			STATUS_FRAME_1_RATE_MS = 1000;
			STATUS_FRAME_2_RATE_MS = 1000;
			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
			ENABLE_VOLTAGE_COMPENSATION = false;
			NOMINAL_VOLTAGE = 12.0;
		}
	};

	private static final SparkConfiguration SLAVE_SPARK_CONFIG = new SparkConfiguration() {
		{
			BURN_FACTORY_DEFAULT_FLASH = false;
			IDLE_MODE = CANSparkMax.IdleMode.kCoast;
			INVERTED = false;
			STATUS_FRAME_0_RATE_MS = 1000;
			STATUS_FRAME_1_RATE_MS = 1000;
			STATUS_FRAME_2_RATE_MS = 1000;
			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
			ENABLE_VOLTAGE_COMPENSATION = false;
			NOMINAL_VOLTAGE = 12.0;
		}
	};

	public static LazySparkMax createDefaultSparkMax(int port) {
		return createSparkMax(port, DEFAULT_SPARK_CONFIG);
	}

	private static void handleCANError(final int id, final REVLibError error, final String message) {
		if (error != REVLibError.kOk) {
			DriverStation.reportError("Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
		}
	}

	public static LazySparkMax createSparkMax(final int port, final SparkConfiguration config) {
		final LazySparkMax sparkMax = new LazySparkMax(port, CANSparkMax.MotorType.kBrushless);

		handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
		handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
		handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

		handleCANError(port, sparkMax.setIdleMode(config.IDLE_MODE), "set neutral");
		sparkMax.setInverted(config.INVERTED);
		handleCANError(port, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
		handleCANError(port, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

		if (config.ENABLE_VOLTAGE_COMPENSATION) {
			handleCANError(port, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
		} else {
			handleCANError(port, sparkMax.disableVoltageCompensation(), "voltage compensation");
		}

		return sparkMax;
	}

	public static class TalonConfiguration {
		public NeutralMode NEUTRAL_MODE;
		// factory default
		public double NEUTRAL_DEADBAND;

		public boolean ENABLE_CURRENT_LIMIT;
		public boolean ENABLE_SOFT_LIMIT;
		public boolean ENABLE_LIMIT_SWITCH;
		public int FORWARD_SOFT_LIMIT;
		public int REVERSE_SOFT_LIMIT;

		public boolean INVERTED;
		public boolean SENSOR_PHASE;

		public int CONTROL_FRAME_PERIOD_MS;
		public int MOTION_CONTROL_FRAME_PERIOD_MS;
		public int GENERAL_STATUS_FRAME_RATE_MS;
		public int FEEDBACK_STATUS_FRAME_RATE_MS;
		public int QUAD_ENCODER_STATUS_FRAME_RATE_MS;
		public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS;
		public int PULSE_WIDTH_STATUS_FRAME_RATE_MS;

		public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD;
		public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

		public double OPEN_LOOP_RAMP_RATE;
		public double CLOSED_LOOP_RAMP_RATE;
	}

	private static final TalonConfiguration DEFAULT_TALON_CONFIG = new TalonConfiguration() {
		{
			NEUTRAL_MODE = NeutralMode.Coast;
			NEUTRAL_DEADBAND = 0.04;

			ENABLE_CURRENT_LIMIT = false;
			ENABLE_SOFT_LIMIT = false;
			ENABLE_LIMIT_SWITCH = false;
			FORWARD_SOFT_LIMIT = 0;
			REVERSE_SOFT_LIMIT = 0;

			INVERTED = false;
			SENSOR_PHASE = false;

			CONTROL_FRAME_PERIOD_MS = 10;
			MOTION_CONTROL_FRAME_PERIOD_MS = 100;
			GENERAL_STATUS_FRAME_RATE_MS = 10;
			FEEDBACK_STATUS_FRAME_RATE_MS = 20;
			QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
			ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
			PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

			VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
			VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
		}
	};


	private static final TalonConfiguration DRIVE_TALON = new TalonConfiguration() {
		{
			NEUTRAL_MODE = NeutralMode.Brake;
			NEUTRAL_DEADBAND = 0.04;

			ENABLE_CURRENT_LIMIT = false;
			ENABLE_SOFT_LIMIT = false;
			ENABLE_LIMIT_SWITCH = false;
			FORWARD_SOFT_LIMIT = 0;
			REVERSE_SOFT_LIMIT = 0;

			INVERTED = false;
			SENSOR_PHASE = false;

			CONTROL_FRAME_PERIOD_MS = 10;
			MOTION_CONTROL_FRAME_PERIOD_MS = 100;
			GENERAL_STATUS_FRAME_RATE_MS = 10;
			FEEDBACK_STATUS_FRAME_RATE_MS = 20;
			QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
			ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
			PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

			VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_25Ms;
			VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 8;

			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
		}
	};

	public static final TalonConfiguration HIGH_PERFORMANCE_TALON_CONFIG = new TalonConfiguration() {
		{
			NEUTRAL_MODE = NeutralMode.Coast;
			NEUTRAL_DEADBAND = 0.02;

			ENABLE_CURRENT_LIMIT = false;
			ENABLE_SOFT_LIMIT = false;
			ENABLE_LIMIT_SWITCH = false;
			FORWARD_SOFT_LIMIT = 0;
			REVERSE_SOFT_LIMIT = 0;

			INVERTED = false;
			SENSOR_PHASE = false;

			CONTROL_FRAME_PERIOD_MS = 5;
			MOTION_CONTROL_FRAME_PERIOD_MS = 100;
			GENERAL_STATUS_FRAME_RATE_MS = 5;
			FEEDBACK_STATUS_FRAME_RATE_MS = 10;
			QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
			ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
			PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

			VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_10Ms;
			VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;

			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
		}
	};

	private static final TalonConfiguration SLAVE_TALON_CONFIG = new TalonConfiguration() {
		{
			NEUTRAL_MODE = NeutralMode.Coast;
			NEUTRAL_DEADBAND = 0.04;

			ENABLE_CURRENT_LIMIT = false;
			ENABLE_SOFT_LIMIT = false;
			ENABLE_LIMIT_SWITCH = false;
			FORWARD_SOFT_LIMIT = 0;
			REVERSE_SOFT_LIMIT = 0;

			INVERTED = false;
			SENSOR_PHASE = false;

			CONTROL_FRAME_PERIOD_MS = 10;
			MOTION_CONTROL_FRAME_PERIOD_MS = 100;
			GENERAL_STATUS_FRAME_RATE_MS = 255;
			FEEDBACK_STATUS_FRAME_RATE_MS = 255;
			QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
			ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
			PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

			VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
			VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

			OPEN_LOOP_RAMP_RATE = 0.0;
			CLOSED_LOOP_RAMP_RATE = 0.0;
		}
	};





	public static LazyTalonFX createDefaultTalonFX(int canID, double rampRate) {
		LazyTalonFX motor = new LazyTalonFX(canID);

		motor.configFactoryDefault();
		motor.setSensorPhase(false);
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
		motor.configFeedbackNotContinuous(false, 4);
		motor.configOpenloopRamp(rampRate);
		motor.configClosedloopRamp(rampRate);
		motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		motor.setNeutralMode(NeutralMode.Coast);

		return motor;
	}

	public static LazyTalonSRX createDefaultTalonSRX(int canID, double rampRate) {
		LazyTalonSRX motor = new LazyTalonSRX(canID);

		motor.configFactoryDefault();
		motor.setSensorPhase(false);
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
		motor.configFeedbackNotContinuous(false, 4);
		motor.configOpenloopRamp(rampRate);
		motor.configClosedloopRamp(rampRate);
		motor.setNeutralMode(NeutralMode.Coast);

		return motor;
	}


}

