package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private Pigeon2 gyro;

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private SwerveModule[] mSwerveMods;

    private ChassisSpeeds chassisSpeeds;

    private final Vision pcw;

    private Field2d field;

    public Swerve(Vision pcw) {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(),
                new Pose2d());

        this.pcw = pcw;

        field = new Field2d();
        chassisSpeeds = new ChassisSpeeds();
        SmartDashboard.putData(field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.fastSpeedLimit);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.fastSpeedLimit);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setModuleRotation(Rotation2d rotation) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(new SwerveModuleState(0, rotation), false);
        }
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public void resetOdometry(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
        setGyro(pose.getRotation());
    }

    public void resetToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void setGyro(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public PathPoint getPoint() {
        return PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Vision getCamera() {
        return pcw;
    }

    @Override
    public void periodic() {
        swervePoseEstimator.update(getYaw(), getPositions());

        Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(getPose());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            swervePoseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }

        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Pigeon2 Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Pigeon2 Pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("Pigeon2 Roll", getRoll().getDegrees());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
