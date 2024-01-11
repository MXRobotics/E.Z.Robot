package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="TeleOpPlus")
public class TeleOpPlus extends CommandOpMode {
    GamepadEx controller;
    //    HardwareMap hardwareMap = new HardwareMap();
//    Telemetry telemetry;
    MecanumDrive mecanum;

    RobotHardware robotHardware = RobotHardware.getInstance();

    @Override
    public void initialize() {
        controller = new GamepadEx(gamepad1);
//        mecanum = new MecanumDrive(new Motor(hardwareMap, "leftFront"),
//                new Motor(hardwareMap, "rightFront"),
//                new Motor(hardwareMap, "leftBack"),
//                new Motor(hardwareMap, "rightBack")
//
//        );
        mecanum = new MecanumDrive(robotHardware.leftFrontMotor, robotHardware.rightFrontMotor, robotHardware.leftBackMotor, robotHardware.rightBackMotor);

        while(opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.update();
        }

    }
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(!isStopRequested()) {
            mecanum.driveRobotCentric(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
        }


    }

}
