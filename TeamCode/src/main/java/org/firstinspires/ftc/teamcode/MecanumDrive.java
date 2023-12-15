package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum Drive", group = "Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    Motor leftFront = new Motor(hardwareMap, "leftFront", GoBILDA.RPM_312);
    Motor leftBack =
}
