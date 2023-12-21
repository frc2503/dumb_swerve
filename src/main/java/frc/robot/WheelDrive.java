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
        
        this.angleMotor.setNeutralMode(NeutralMode.Brake);
        this.angleMotor.configClosedloopRamp(0);
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        speedEncoder = this.speedMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        speedEncoder.setPosition(0);
        
        speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setFeedbackDevice(speedEncoder);


        this.angleMotor.configClosedLoopPeakOutput(0, 1);
        speedPIDController.setOutputRange(-1, 1);

        updatePIDValues(Constants.driveFeedForward, Constants.driveProportional, Constants.driveIntegral, Constants.driveDerivative, Constants.steerFeedForward, Constants.steerProportional, Constants.steerIntegral, Constants.steerDerivative);
    }

    public void drive(double speed, double angle) {
        angleMotor.set(ControlMode.Position, (angle / 360.0) * 1024);
        speedMotor.set(speed);
        //TODO the PIDController here will smooth out acceleration and deceleration. Just need to figure out how
        //speedPIDController.setReference(speed, ControlType.kVelocity);
    }

    public void updatePIDValues(double DFF, double DP, double DI, double DD, double SFF, double SP, double SI, double SD) {
        speedPIDController.setFF(DFF);
        speedPIDController.setP(DP);
        speedPIDController.setI(DI);
        speedPIDController.setD(DD);
        angleMotor.config_kF(0, SFF);
        angleMotor.config_kP(0, SP);
        angleMotor.config_kI(0, SI);
        angleMotor.config_kD(0, SD);
      }
}
