//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivebase;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//
//
//@TeleOp(name="t1", group="Linear OpMode")
//public class t1 extends LinearOpMode {
//    //RobotHardware robot = RobotHardware.getInstance();
//    MecanumDrivebase mecanum = new MecanumDrivebase();
//    GamepadEx controller1;
//    GamepadEx getController2;
//
////    @Override
////    public void init() {
////    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        controller1 = new GamepadEx(gamepad1);
//        getController2 = new GamepadEx(gamepad2);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            double x = controller1.getLeftX();
//            double y = controller1.getLeftY();
//            double rot = controller1.getRightX();
//            mecanum.drive(x, y, rot);
//        }
//    }
//}
