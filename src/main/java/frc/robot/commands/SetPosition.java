package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetPosition extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Position position;
    private Timer timer;
    private Supplier<GamePiece> gamePiece;

    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position, Supplier<GamePiece> gamePiece) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.position = position;
        this.gamePiece = gamePiece;
        this.timer = new Timer();

        addRequirements(s_Wrist, s_Elevator);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        switch (position) {
            case HIGH:
                if (gamePiece.get() == GamePiece.CONE) {
                    s_Wrist.setPosition(Position.CONEHIGH.getWrist());
                    s_Elevator.setPosition(Position.CONEHIGH.getElev());
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    s_Wrist.setPosition(Position.CUBEHIGH.getWrist());
                    s_Elevator.setPosition(Position.CUBEHIGH.getElev());
                }
                break;

            case MID:
                if (gamePiece.get() == GamePiece.CONE) {
                    s_Wrist.setPosition(Position.CONEMID.getWrist());
                    s_Elevator.setPosition(Position.CONEMID.getElev());
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    s_Wrist.setPosition(Position.CUBEMID.getWrist());
                    s_Elevator.setPosition(Position.CUBEMID.getElev());
                }
                break;

            default:
                s_Wrist.setPosition(position.getWrist());
                s_Elevator.setPosition(position.getElev());
                break;

        }
        /*
         * if(sequential) {
         * addCommands(setElevatorPose, setWristPose);
         * }
         * else {
         * addCommands(new ParallelCommandGroup(setElevatorPose, setWristPose));
         * }
         */
    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (s_Wrist.atSetpoint() && s_Elevator.atSetpoint()) || timer.get() > .75;
    }

}
