package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class Elevator extends SubsystemBase {

    private final CANSparkMax elevatorMotorLeft; // making the left the lead motor
    private final CANSparkMax elevatorMotorRight; // the right motor is the follower

    private PIDController pidController;

    private double currentPosition;

    /**
     * Initialize Elevator motor and the built in encoder. There are no cancoders on
     * the elevator
     */
    public Elevator() {
        // initialize motors
        // the right motor will spin clockwise and the left motor will go counter
        // clockwise
        elevatorMotorLeft = new CANSparkMax(Constants.Elevator.motorLeftId, MotorType.kBrushless);

        elevatorMotorRight = new CANSparkMax(Constants.Elevator.motorRightId, MotorType.kBrushless);

        elevatorMotorLeft.restoreFactoryDefaults();
        elevatorMotorRight.restoreFactoryDefaults();

        elevatorMotorRight.follow(elevatorMotorLeft, true);

        elevatorMotorLeft.setSmartCurrentLimit(Constants.Elevator.currentLimit); 
        elevatorMotorRight.setSmartCurrentLimit(Constants.Elevator.currentLimit); 
        // elevatorRightController = new CANCoder(Constants.Elevator.canConderRightId);

        // The motors will follow each other
        // The right motor will follow whatever the applied output on the
        // left motor is so only need to adjust output for the left motor

        // initialize pidContoller
        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI,
                Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.25);

        setPosition(Position.STANDBY.getElev());
    }

    public void resetEncoder() {
        elevatorMotorLeft.getEncoder().setPosition(0);
        elevatorMotorRight.getEncoder().setPosition(0);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        if(position > 35) {
            position = 35;
        }
        else if(position < 0.1) {
            position = 0.1;
        }
        currentPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public void move(double voltage) {
        elevatorMotorLeft.setVoltage(voltage);
    }

    public boolean reachedSetpoint(double distance) {
        return pidController.getPositionTolerance() >= Math.abs(currentPosition - distance);
    }

    public double getEncoderPosition() {
        return (elevatorMotorLeft.getEncoder().getPosition() + elevatorMotorRight.getEncoder().getPosition()) / 2;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Goal Position", currentPosition);

        move(
                MathUtil.clamp(
                        pidController.calculate(getEncoderPosition(), currentPosition),
                        -Constants.Elevator.maxMotorVoltage,
                        Constants.Elevator.maxMotorVoltage));
    }
}