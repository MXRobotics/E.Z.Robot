package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.util.wrappers.Encoder;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;


public class RobotHardware {

    // motors
    public DcMotorEx leftFrontMotor;

    public DcMotorEx leftBackMotor;

    public DcMotorEx rightFrontMotor;
    
    public DcMotorEx rightBackMotor;

    public DcMotorEx intakeMotor;

    public DcMotorEx leftSlide;

    public DcMotorEx rightSlide;

    // encoders

    public Encoder leftFrontEncoder;

    public Encoder leftBackEncoder;

    public Encoder rightFrontEncoder;

    public Encoder rightBackEncoder;

    public Encoder intakeEncoder;

    public Encoder rightOdoEncoder;

    public Encoder leftOdoEncoder;

    public Encoder centerOdoEncoder;


     private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    public boolean enabled;


    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

     public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
//        this.values = new HashMap<>();
        this.telemetry = telemetry;

        // Drivetrain motors and encoders

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        leftFrontEncoder = new Encoder(new MotorEx(hardwareMap, "leftFront").encoder);
        leftBackEncoder = new Encoder(new MotorEx(hardwareMap, "leftBack").encoder);
        rightFrontEncoder = new Encoder(new MotorEx(hardwareMap, "rightFront").encoder);
        rightBackEncoder = new Encoder(new MotorEx(hardwareMap, "rightBack").encoder);

        // Intake motor and encoder

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeEncoder = new Encoder(new MotorEx(hardwareMap, "intakeMotor").encoder);

        // Slide motors

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setDirection(DcMotorEx.Direction.FORWARD);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Odometry encoders

        rightOdoEncoder = new Encoder(new MotorEx(hardwareMap, "rightSlide").encoder);
        leftOdoEncoder = new Encoder(new MotorEx(hardwareMap, "leftSlide").encoder);
        centerOdoEncoder = new Encoder(new MotorEx(hardwareMap, "centerOdo").encoder);


     }

     public void read() {
        if (!enabled) {
            return;
        }

        // Drivetrain motors and encoders
//
//        leftFrontMotor.setPower(values.get("leftFrontMotor"));
//        leftBackMotor.setPower(values.get("leftBackMotor"));
//        rightFrontMotor.setPower(values.get("rightFrontMotor"));
//        rightBackMotor.setPower(values.get("rightBackMotor"));
//
//        // Intake motor and encoder
//
//        intakeMotor.setPower(values.get("intakeMotor"));
//
//        // Slide motors
//
//        leftSlide.setPower(values.get("leftSlide"));
//        rightSlide.setPower(values.get("rightSlide"));
//
//        // Odometry encoders
//
//        rightOdoEncoder.setPower(values.get("rightOdoEncoder"));
//        leftOdoEncoder.setPower(values.get("leftOdoEncoder"));
//        centerOdoEncoder.setPower(values.get("centerOdoEncoder"));
     }
}