package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.SwerveConstants;
import frc.robot.lib.subsystems.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;
    private final PIDController turnPid = new PIDController(0, 0, 0);
    private double x = 0;
    private double y = 0;
    private double rot = 0;

    private final StructArrayPublisher<SwerveModuleState> modulePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Advantage_Config/RobotModule", SwerveModuleState.struct).publish();
    private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Advantage_Config/Robot", Pose2d.struct).publish();

    public SwerveSubsystem() {
        super("Swerve");
        this.frontLeft = new SwerveModule(
            2, 1, 9,
            true, true, true,
            "frontLeft"
        );
        this.frontRight = new SwerveModule(
            6, 5, 10, 
            false, true, true,
            "frontRight"
        );
        this.backLeft = new SwerveModule(
            4, 3, 11,
            true, true, true,
            "backLeft"
        );
        this.backRight = new SwerveModule(
            7, 8, 12,
            false, true, true,
            "backRight"
        );
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.odometry = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition()
        );
        this.wait(1000);
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
        // this.publisher.set(new Pose3d(this.getPose()));
    }
    
    public void resetGyro() {
        this.gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param field         Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    public void simDrive(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
        
        this.x += speeds.vxMetersPerSecond * 0.015;
        this.y += speeds.vyMetersPerSecond * 0.015;
        this.rot += speeds.omegaRadiansPerSecond * 0.06;
        this.publisher.set(new Pose2d(this.x, this.y, new Rotation2d(this.rot)));
        this.modulePublisher.set(states); /// Swerve Sim
    }

    public void turnDrive(double xSpeed, double ySpeed, double angle) {
        double speed = this.turnPid.calculate(this.gyro.getAngle(), angle);
        this.driveSwerve(xSpeed, ySpeed, speed, true);
    }

    public void chassisDrive(ChassisSpeeds relativeSpeed) {
        ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(relativeSpeed, 0.02);
        if (Robot.isSimulation()) this.simDrive(targetSpeed.vxMetersPerSecond, targetSpeed.vyMetersPerSecond, targetSpeed.omegaRadiansPerSecond);
        SwerveModuleState state[] = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(targetSpeed);
        this.setModuleState(state);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    /**
     * Returns the current state of the swerve.
     *
     * @return The current state of the swerve.
     */
    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    /**
     * Returns the current position of the swerve.
     *
     * @return The current position of the swerve.
     */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

     /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        if (Robot.isSimulation()) {
            this.publisher.set(pose);
            this.x = pose.getX();
            this.y = pose.getY();
            this.rot = pose.getRotation().getDegrees();
        }
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePosition(), pose);
    }

    /**
     * Returns the chassis speed of the swerve
     * 
     * @return The chassis speed of the swerve
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    // Stop swerve.
    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
    }
}