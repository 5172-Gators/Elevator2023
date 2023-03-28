package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Intake.EjectSpeed;
import frc.robot.subsystems.Intake;

public class TimedIntake extends CommandBase {

    private double time;
    private final Intake s_Intake;
    private Timer timer;
    private EjectSpeed eject;
    private GamePiece gamePiece;
    private Direction direction;

    public enum Direction {
        INTAKE,
        OUTTAKE
    }

    public TimedIntake(Intake s_Intake, double time, GamePiece gamePiece, EjectSpeed eject,
            Direction direction) {
        this.time = time;
        this.eject = eject;
        this.s_Intake = s_Intake;
        this.gamePiece = gamePiece;
        this.timer = new Timer();
        this.direction = direction;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        switch (gamePiece) {
            case CONE:
                if (direction == Direction.INTAKE) {
                    s_Intake.setMotor(-eject.speed);
                } else {
                    s_Intake.setMotor(eject.speed);
                }
                break;
            case CUBE:
                if (direction == Direction.INTAKE) {
                    s_Intake.setMotor(Constants.Intake.cubeOuttakeSpeed);
                } else {
                    s_Intake.setMotor(-Constants.Intake.cubeOuttakeSpeed);
                }
                break;

        }

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();

        s_Intake.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
