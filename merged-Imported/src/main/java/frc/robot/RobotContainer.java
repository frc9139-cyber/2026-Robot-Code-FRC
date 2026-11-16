package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AutoAimToTag;
import frc.robot.commands.ShootWhenReadyAuto;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

public class RobotContainer {

    private static final double DRIVE_JOYSTICK_DEADBAND = 0.08;

    private final Drive drive = new Drive();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> allianceChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<>();

    public RobotContainer() {
        // Limelight setup
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.setStreamMode_Standard("limelight");
        HttpCamera limelightFeed = new HttpCamera("Limelight", "http://10.91.39.11");

    CameraServer.addCamera(limelightFeed);

    Shuffleboard.getTab("Drive")
        .add("Limelight Video", limelightFeed)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withSize(4, 3) // Adjust size as needed
        .withPosition(0, 0);

    


        // Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Named commands for auto
        NamedCommands.registerCommand("shooting", new ShootWhenReadyAuto(shooter, 2700, 5.0));
        NamedCommands.registerCommand("toggle intake", intake.toggleIntake());
        NamedCommands.registerCommand("intake", intake.intake().withTimeout(3));

        // Alliance chooser
        allianceChooser.setDefaultOption("Red", "Red");
        allianceChooser.addOption("Blue", "Blue");
        SmartDashboard.putData("Alliance Color", allianceChooser);

        // Drive mode chooser
        fieldOrientedChooser.setDefaultOption("Field Oriented", false);
        fieldOrientedChooser.addOption("Robot Oriented", true);
        SmartDashboard.putData("Drive Mode", fieldOrientedChooser);

        configureBindings();
    }

    private void configureBindings() {

        // Default drive command
        drive.setDefaultCommand(
            new RunCommand(() -> {
                var controls = new Translation2d(
                    shapeDriveAxis(m_driverController.getLeftY()) * MAX_LINEAR_SPEED_TELEOP,
                    shapeDriveAxis(m_driverController.getLeftX()) * MAX_LINEAR_SPEED_TELEOP);
                var thetaInput = shapeDriveAxis(m_driverController.getRightX()) * MAX_ANGULAR_SPEED * 0.5;
                boolean fieldOriented = fieldOrientedChooser.getSelected();
                drive.drive(controls.getX(), controls.getY(), thetaInput, fieldOriented);
            }, drive)
        );
        
    
        // Start: Reset heading
        var resetHeading = Commands.runOnce(drive::zeroPose, drive);
        resetHeading.runsWhenDisabled();
        m_driverController.start().onTrue(resetHeading);

        // Left Trigger: Auto-align with vision
        m_driverController.leftTrigger().whileTrue(new AutoAimToTag(drive, m_driverController));

        // Right Trigger: Autoshoot (spin + auto-fire when ready)
        m_operatorController.rightTrigger().whileTrue(
            Commands.runEnd(
                () -> shooter.setShooterSpeed(CLOSE_RANGE_RPM, true, true),
                () -> shooter.stopAll(),
                shooter
            )
        );

        // Left Trigger: Shooter spinup only (no feeder)
        m_operatorController.leftTrigger().whileTrue(
            Commands.runEnd(
                () -> shooter.setShooterSpeed(CLOSE_RANGE_RPM, false, false),
                () -> shooter.stopAll(),
                shooter
            )
        );

        // Right Bumper: Feeder forward (manual)
        m_operatorController.rightBumper().whileTrue(
            Commands.runEnd(
                () -> shooter.testFeederDirect(),
                () -> shooter.stopFeeder(),
                shooter
            )  
        );

        // Left Bumper: Feeder reverse
        m_operatorController.leftBumper().whileTrue(
            Commands.runEnd(
                () -> shooter.setOuttake(),
                () -> shooter.stopFeeder(),
                shooter
            )
        );

        // A: Intake
        m_operatorController.a().whileTrue(intake.intake());

        // B: Outtake
        m_operatorController.b().whileTrue(intake.outtake());

        // X: Intake arm down
        m_operatorController.x().onTrue(intake.intakeDown());
    
        // Y: Intake arm up
        m_operatorController.y().onTrue(intake.intakeUp());
    }

    // Blend linear and cubic response for precise starts without a sluggish midrange.
    private static double shapeDriveAxis(double input) {
        double adjusted = MathUtil.applyDeadband(input, DRIVE_JOYSTICK_DEADBAND);
        return 0.35 * adjusted + 0.65 * adjusted * adjusted * adjusted;
    }

    public static Translation2d getControls(CommandXboxController controller) {
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red;

        double forward = controller.getLeftY();
        double strafe = controller.getLeftX();

        forward = Math.copySign(forward * forward, forward);
        strafe = Math.copySign(strafe * strafe, strafe);

        if (isRed) {
            forward = forward;
        }

        forward *= MAX_LINEAR_SPEED_TELEOP;
        strafe *= MAX_LINEAR_SPEED_TELEOP;

        return new Translation2d(forward, strafe);
    }

    public static boolean isRed() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red;
    }

    public static double getDistanceToTarget() {
        if (!LimelightHelpers.getTV("limelight")) {
            return -1.0;
        }

        double ty = LimelightHelpers.getTY("limelight");
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        if (Math.abs(angleToTargetRad) < 0.01) {
            return 999.0;
        }

        double distanceMeters = (HUB_TAG_HEIGHT_M - LIMELIGHT_HEIGHT_M)
                / Math.tan(angleToTargetRad);

        return Math.max(distanceMeters, 0.5);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kCoast))
                .beforeStarting(Commands.waitSeconds(3.0))
                .ignoringDisable(true)
        );
    }

    public void enabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kBrake))
        );
    }
}