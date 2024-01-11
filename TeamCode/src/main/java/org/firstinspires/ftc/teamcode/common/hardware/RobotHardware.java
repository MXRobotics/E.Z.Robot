package org.firstinspires.ftc.teamcode.common.hardware;

// ftclib imports
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.SensorColor;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;

// ftc sdk imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// internal imports
import org.firstinspires.ftc.teamcode.common.util.wrappers.Encoder;
//import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

// telemetry and dashboard
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;

// imu
import com.qualcomm.hardware.bosch.BNO055IMU;
// vision
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import javax.annotation.concurrent.GuardedBy;

public class RobotHardware {

    // dt
    public Motor leftFrontMotor;
    public Motor leftBackMotor;
    public Motor rightFrontMotor;
    public Motor rightBackMotor;

    public Encoder leftFrontEncoder;
    public Encoder leftBackEncoder;
    public Encoder rightFrontEncoder;
    public Encoder rightBackEncoder;


    // intake
    public DcMotorEx intakeMotor;
    public Encoder intakeEncoder;

    // slide
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    // odometry

    public Encoder rightOdo;
    public Encoder leftOdo;
    public Encoder centerOdo;

    // imu
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;
    // public TwoWheelLocalizer localizer;

    // vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

//    public MecanumDrivetrain drivetrain;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    public boolean enabled;

    private static double p = 0.0, i = 0.0, d = 0.0;
    private double errorTolerance = 0.0;
    private int drivePosTolerance = 50;

    private ArrayList<WSubsystem> subsystems;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry   Saved for later in the event FTC Dashboard used
     */
     public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
//        this.values = new HashMap<>();
        this.telemetry = telemetry;

        // Drivetrain motors and encoders

        leftFrontMotor = hardwareMap.get(Motor.class, "leftFront");
//        leftFrontMotor.setInverted(true);
        leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftBackMotor = hardwareMap.get(Motor.class, "leftBack");
//        leftBackMotor.setInverted(true);
        leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor = hardwareMap.get(Motor.class, "rightFront");
        rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightBackMotor = hardwareMap.get(Motor.class, "rightBack");
        rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        leftFrontEncoder = new Encoder((leftFrontMotor.encoder);
//        leftBackEncoder = new Encoder(leftBackMotor.encoder);
//        rightFrontEncoder = new Encoder(rightFrontMotor.encoder);
//        rightBackEncoder = new Encoder(rightBackMotor.encoder);

        // Intake motor and encoder

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        intakeEncoder = new Encoder(intakeMotor.encoder);

        // Slide motors

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
//        leftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
//        rightSlide.setInverted(true);
//        rightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Odometry encoders

        rightOdo = new Encoder(new Motor(hardwareMap, "rightSlide").encoder);
        leftOdo = new Encoder(new Motor(hardwareMap, "leftSlide").encoder);
        centerOdo = new Encoder(new Motor(hardwareMap, "centerOdo").encoder);

        // IMU
         synchronized (imuLock) {
             imu = hardwareMap.get(BNO055IMU.class, "imu");
             BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
             parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
             imu.initialize(parameters);
         }

         subsystems = new ArrayList<>();

//         drivetrain = new MecanumDrivetrain(telemetry);
     }
     public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuAngle = AngleUnit.normalizeRadians(imu.getAngularOrientation().firstAngle + startOffset);
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return AngleUnit.normalizeRadians(imuAngle - imuOffset);
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }

        imuOffset = imuAngle;
    }

    public void setStartOffset(double off){
        startOffset = off;
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

     public double getVoltage() {
        return voltage;
    }

    public void resetYaw() {
        synchronized (imuLock) {
            imuOffset = imuAngle;
        }
    }

}