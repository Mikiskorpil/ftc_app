package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

    @Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
    public class Automatic_Encoder_Op_Mode extends LinearOpMode {

        /* Declare OpMode members. */
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor rightDrive = null;
        private DcMotor leftDrive = null;

        static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 3.54;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.14159265363);
        static final double DRIVE_SPEED = 2.0;
        static final double TURN_SPEED = 0.5;

        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            leftDrive.setPower(-DRIVE_SPEED);
            rightDrive.setPower(DRIVE_SPEED);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setTargetPosition(2240);
            rightDrive.setTargetPosition(2240);

            while (leftDrive.isBusy() && opModeIsActive()) {

            }
            while (rightDrive.isBusy() && opModeIsActive()) {

            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

