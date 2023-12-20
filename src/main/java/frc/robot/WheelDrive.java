package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;

public class WheelDrive {
    private TalonSRX angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController speedPIDController;
    private RelativeEncoder speedEncoder;



    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new TalonSRX(angleMotor);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

        // I think the 42 here is the encoder parameter but not positive
        speedEncoder = this.speedMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        speedEncoder.setPosition(0);
        
        speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setFeedbackDevice(speedEncoder);
        speedPIDController.setOutputRange(-1, 1);

        
        this.angleMotor.setNeutralMode(NeutralMode.Brake);
        this.angleMotor.configClosedloopRamp(0);
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        this.angleMotor.configClosedLoopPeakOutput(0, 1);
        speedPIDController.setOutputRange(-1, 1);
    }

    public void drive(double speed, double angle) {
        System.out.println("Angle: " + angle);
        angleMotor.set(ControlMode.Position, (angle / 360.0) * 1024);
        speedPIDController.setReference(speed, ControlType.kVelocity);
    }
}
