package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.VectorTools.util.HSV;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

    public static final class Swerve {
        public static final double stickDeadband = 0.1;

        public static final int pigeonID = 6;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.12 / 1.0); // 6.12:1
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 30;

        public static final double pitchSetPoint = 0.0;

        public static final double drivePitchKP = 0.015;
        public static final double drivePitchKI = 0.0001;
        public static final double drivePitchKD = 0.000000000000001;
        public static final double drivePitchKFF = 0.000000000000001;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.005;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.16059;
        public static final double driveKV = 2.4135;
        public static final double driveKA = 0.41807;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double speedLimit = 3.0; // Base Meters Per Second Speed
        public static final double slowSpeedLimit = 0.75; // Slow Meters Per Second Speed
        public static final double fastSpeedLimit = 4.5; // Fast Meters Per Second Speed

        public static final double accelerationLimit = 3.0;
        public static final double fastAccelerationLimit = 5.0;

        public static final double angularVelocityLimit = 5;
        public static final double slowAngularVelocityLimit = 1.25;
        public static final double fastAngularVelocityLimit = 6;

        /* Swerve Limiting Values */
        public static final double autoCenterLimit = .3;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-121.8164);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.09);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52.734);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(60.82);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class Elevator {
        public static final int motorLeftId = 50;
        public static final int motorRightId = 51;

        public static final double elevatorKP = 1.5;
        public static final double elevatorKI = .2;
        public static final double elevatorKD = .05;

        public static final double maxMotorVoltage = 5;

        public static final int currentLimit = 30;

    }

    public static final class Wrist {
        public static final int wristMotorId = 61;
        public static final double maxMotorVoltage = 10.0;

        public enum PIDFFmode {
            WEIGHTED(
                    Wrist.weightedP,
                    Wrist.weightedI,
                    Wrist.weightedD,
                    Wrist.weightedS,
                    Wrist.weightedV,
                    Wrist.weightedA,
                    Wrist.weightedG),
            UNWEIGHTED(
                    Wrist.unweightedP,
                    Wrist.unweightedI,
                    Wrist.unweightedD,
                    Wrist.unweightedS,
                    Wrist.unweightedV,
                    Wrist.unweightedA,
                    Wrist.unweightedG);

            public final double kP;
            public final double kI;
            public final double kD;
            public final double kS;
            public final double kV;
            public final double kA;
            public final double kG;

            private PIDFFmode(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.kS = kS;
                this.kV = kV;
                this.kA = kA;
                this.kG = kG;
            }

        }

        public static double weightedP = 2.8;
        public static double weightedI = 0.0;
        public static double weightedD = 0.2;

        public static double weightedS = 0.4361;
        public static double weightedV = 0.79036;
        public static double weightedA = 0.0;
        public static double weightedG = 0.86416;

        public static double unweightedP = 2.2;
        public static double unweightedI = 0.0;
        public static double unweightedD = 0.2;

        public static double unweightedS = 0.11237;
        public static double unweightedV = 0.56387;
        public static double unweightedA = 0.041488;
        public static double unweightedG = 0.76416;

        public static final double motorGearRatio = 1 / 32.0;
        public static final double absoluteEncoderOffset = 5.412927;

        public static final int currentLimit = 40;
    }

    public static final class Intake {
        public static final int motorId = 60;

        public static final int pdpChannel = 2; // update number later

        public static final double stoppedRPMThreshold = .01;

        public static final double coneIntakeSpeed = 8;
        public static final double cubeIntakeSpeed = 4;

        public static final double coneOuttakeSpeed = 4;
        public static final double coneShootSpeed = 12;
        public static final double cubeOuttakeSpeed = 7;

        public static final int currentLimit = 30;

        public enum EjectSpeed {
            FAST(12),
            NORMAL(7);

            public final double speed;

            EjectSpeed(double speed) {
                this.speed = speed;
            }
        }
    }

    public enum Position {

        HIGH(0, 0),
        CONEHIGH(.067, 35),
        CUBEHIGH(1.55, 35),
        MID(0, 0),
        CONEMID(5.81731, 35),
        CUBEMID(1.427, 16.5),
        LOW(.5236, .25),
        STANDBY(1.1765, .25),
        CUBEINTAKE(.1, 0.3),
        STANDINGCONEINTAKE(5.106, 14.380),
        TIPPEDCONEINTAKE(5.572, 1.333),
        HUMANPLAYERINTAKE(.8763, 1.5);

        private double wristPos;
        private double elevatorPos;

        private Position(double wrist, double elev) {
            this.wristPos = wrist;
            this.elevatorPos = elev;
        }

        public double getWrist() {
            return wristPos;
        }

        public double getElev() {
            return elevatorPos;
        }
    }

    public enum GamePiece {
        CUBE(1),
        CONE(-1);

        private double direction;

        private GamePiece(double value) {
            direction = value;
        }

        public double getDirection() {
            return direction;
        }
    }

    public enum SEGMENT { // Numbers in order of segment from left to right (driver station POV)
        CONE_1(0), CONE_2(31.8), CONE_3(35.2), CONE_4(-1), CONE_5(-1), CONE_6(-1),
        CUBE_1(0), CUBE_2(20.6), CUBE_3(35.2), HUMANPLAYER(-1);

        // intake Ground Cube: 0
        // intake Cone Upright: 12
        // intake Cone Tipped: 0

        // intake Cone Single HP: 8.9
        // intake Cube Single HP: 11.8

        private double level;

        private SEGMENT(double level) {
            this.level = level;
        }

        public static SEGMENT getSegment(int level, boolean cone) {
            if (cone) {
                switch (level) {
                    case 1:
                        return CONE_1;
                    case 2:
                        return CONE_2;
                    case 3:
                        return CONE_3;
                }
            } else {
                switch (level) {
                    case 1:
                        return CUBE_1;
                    case 2:
                        return CUBE_2;
                    case 3:
                        return CUBE_3;
                }
            }
            return null;
        }

        public double getValue() {
            return level;
        }

    }

    public static final class Autonomous {
        public static final PathConstraints constraints = new PathConstraints(1, 1);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constants for line up */
        public static final double kPGridLineUp = 0.35;
        public static final double kIGridLineUp = 0.0;
        public static final double gridLineUpTolerance = 0.1;

        public static final double kPThetaGridLineUp = 0.025;
        public static final double thetaGridLineUpTolerance = 2.0;

        public static final double gridLineUpPosition = 1.98;
        public static final double gridLineUpAngle = 180.0;
    }

    public static final class PhotonVision {
        public static final String photonVisionName = "OV5647";
        public static final Transform3d robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.1505), Units.inchesToMeters(9.666),
                        Units.inchesToMeters(31.185)),
                new Rotation3d(
                        0, Units.degreesToRadians(15),
                        Units.degreesToRadians(5)));
    }

    public static final class LEDs {
        public static final int id = 9;
        public static final int length = 18;

        public static final LEDMode defaultMode = LEDMode.VECTORWAVE;

        public static final class Flash {
            public static final double speed = 5;
        }

        public static final class VectorWave {
            public static final HSV hsv = HSV.googleColorPickerHSV(8, 100, 100);

            public static final int pauseBetween = 10;
            public static final int length = 30;
            public static final double spread = 4;
            public static final double speed = .25;
        }

        public static final class OrangeDot {
            public static final HSV hsv = HSV.googleColorPickerHSV(20, 99, 100);

            public static final int pauseBetween = 10;
            public static final int length = 0;
            public static final double spread = 2;
            public static final double speed = .5;
        }

        public enum LEDMode {
            ORANGEDOT("Orange Dot"),
            VECTORWAVE("Vector Wave"),
            RAINBOW("Rainbow"),
            PURPLEFLASH("Purple Flash"),
            YELLOWFLASH("Yellow Flash"),
            GREENFLASH("Green Flash"),
            REDFLASH("Red Flash");

            private String name;

            private LEDMode(String name) {
                this.name = name;
            }

            @Override
            public String toString() {
                return name;
            }
        }
    }
}