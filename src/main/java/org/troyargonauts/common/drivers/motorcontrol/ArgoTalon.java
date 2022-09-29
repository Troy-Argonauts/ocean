package org.troyargonauts.common.drivers.motorcontrol;

// Copyright 2019 FRC Team 3476 Code Orange

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Sends only new commands to the Talon to reduce CAN usage.
 */
public class ArgoTalon extends WPI_TalonFX {

	private double prevValue = 0;
	private final ControlMode prevControlMode = ControlMode.Disabled;

	public ArgoTalon(int deviceNumber) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(ControlMode mode, double outputValue) {
		//return;

		if (outputValue != prevValue || mode != prevControlMode) {
			super.set(mode, outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}
}
