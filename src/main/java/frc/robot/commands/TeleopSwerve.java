package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.VectorTools.util.SlewRateLimiter;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDs.LEDs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private LEDs s_LEDs;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftBumper;
    private BooleanSupplier rightBumper;
    private BooleanSupplier gridLineUp;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    // private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private PIDController translationController;
    private PIDController rotationController;

    private LEDMode previousMode;

    private enum Speed {
        FAST,
        NORMAL,
        SLOW
    }

    /**
     * The constructor initializes the class variables.
     * 
     * @param s_Swerve
     * @param translationSup
     * @param strafeSup
     * @param rotationSup
     * @param autoCenter
     * @param robotCentricSup
     */
    public TeleopSwerve(
            Swerve s_Swerve,
            LEDs s_LEDs,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier leftBumper,
            BooleanSupplier rightBumper,
            BooleanSupplier gridLineUp) {
        this.s_Swerve = s_Swerve;
        this.s_LEDs = s_LEDs;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.gridLineUp = gridLineUp;

        this.previousMode = s_LEDs.getLEDMode();
    }

    @Override
    public void initialize() {
        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);

        translationController = new PIDController(Constants.Autonomous.kPGridLineUp,
                Constants.Autonomous.kIGridLineUp,
                0);
        translationController.setTolerance(Constants.Autonomous.gridLineUpTolerance);

        rotationController = new PIDController(Constants.Autonomous.kPThetaGridLineUp, 0, 0);
        rotationController.setTolerance(Constants.Autonomous.thetaGridLineUpTolerance);
        rotationController.enableContinuousInput(0, 360);

    }

    /**
     * TODO
     */

    @Override
    public void execute() {
        /* Set Speeds based on button input */
        Speed speed = leftBumper.getAsBoolean() ? Speed.SLOW
                : rightBumper.getAsBoolean()
                        ? Speed.FAST
                        : Speed.NORMAL;

        double speedLimit = Constants.Swerve.speedLimit;
        double angularSpeedLimit = Constants.Swerve.angularVelocityLimit;

        switch (speed) {
            case FAST:
                translationLimiter.setRateLimit(Constants.Swerve.fastAccelerationLimit);
                strafeLimiter.setRateLimit(Constants.Swerve.fastAccelerationLimit);

                speedLimit = Constants.Swerve.fastSpeedLimit;
                angularSpeedLimit = Constants.Swerve.fastAngularVelocityLimit;
                break;
            case SLOW:
                translationLimiter.setRateLimit(Constants.Swerve.accelerationLimit);
                strafeLimiter.setRateLimit(Constants.Swerve.accelerationLimit);

                speedLimit = Constants.Swerve.slowSpeedLimit;
                angularSpeedLimit = Constants.Swerve.slowAngularVelocityLimit;
                break;
            default:
                translationLimiter.setRateLimit(Constants.Swerve.accelerationLimit);
                strafeLimiter.setRateLimit(Constants.Swerve.accelerationLimit);
                break;
        }

        if (s_LEDs.getLEDMode() != LEDMode.GREENFLASH &&
                s_LEDs.getLEDMode() != LEDMode.REDFLASH &&
                s_LEDs.getLEDMode() != previousMode) {
            previousMode = s_LEDs.getLEDMode();
        }

        /* Get Values, Deadband */
        double translationVal;
        double rotationVal;
        double strafeVal = strafeLimiter
                .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(),
                        Constants.Swerve.stickDeadband));

        if (gridLineUp.getAsBoolean()) {

            translationVal = MathUtil.clamp(
                    translationController.calculate(s_Swerve.getPose().getX(),
                            Constants.Autonomous.gridLineUpPosition),
                    -1,
                    1);

            rotationVal = MathUtil.clamp(
                    rotationController.calculate(s_Swerve.getYaw().getDegrees(),
                            Constants.Autonomous.gridLineUpAngle),
                    -1,
                    1);

            if (translationController.atSetpoint())
                translationVal = 0;

            if (translationController.atSetpoint() && rotationController.atSetpoint()) {
                s_LEDs.setLEDMode(LEDMode.GREENFLASH);
            } else {
                s_LEDs.setLEDMode(LEDMode.REDFLASH);
            }

        } else {
            if (s_LEDs.getLEDMode() != previousMode)
                s_LEDs.setLEDMode(previousMode);

            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(),
                    Constants.Swerve.stickDeadband);
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(),
                    Constants.Swerve.stickDeadband);
        }

        s_Swerve.drive(
                new Translation2d(translationLimiter.calculate(translationVal), strafeVal)
                        .times(speedLimit),
                rotationVal * (angularSpeedLimit),
                !robotCentricSup.getAsBoolean(),
                false);

    }
}
