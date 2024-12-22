package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
	public static final double TRACK_WIDTH = Units.inchesToMeters(19.25);
	public static final double TRACK_LENGTH = Units.inchesToMeters(19.25);
	public static final double WHEEL_RADIUS = 0.0508;

	public static final double MAX_SPEED_METERS_PER_SECOND = 2.0;
	public static final double MAX_ACCELERATION = 9.0;
	public static final double MAX_ANGULAR_DEGREES_PER_SECOND = 90.0;
	public static final double MAX_ANGULAR_ACCELERATION = 9.0;
	public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
	public static final int MAX_VOLTAGE = 20;

	public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI / 60;
	public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI;

	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
		new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
		new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2,TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)
	);

	public static final double DEAD_BAND = 0.05;
	public static final boolean gyroField = true;
}
