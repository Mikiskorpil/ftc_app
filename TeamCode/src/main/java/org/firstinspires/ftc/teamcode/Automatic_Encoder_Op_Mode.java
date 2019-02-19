package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
    public class Automatic_Encoder_Op_Mode extends LinearOpMode {

        /* Declare OpMode members. */
        HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
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

            //ZAČÁTEK NA STRANĚ S KRÁTEREM
            //Základní trasa, otočky můžeme dopsat až s definitivně připevněnýma kolečkama
            //Vzdálenosti jsou jenom odhady by oko podle obrázků

            encoderDrive(DRIVE_SPEED, 24, 24,0);
            //před 3 minerálama, Truny přinesl jeden Colour Sensor, můžeme zkusit příště
            encoderDrive(TURN_SPEED, );
            encoderDrive(DRIVE_SPEED, 36, 36,0);
            //zatáčka na cestě do team čtverce v rohu
            encoderDrive(TURN_SPEED, );
            encoderDrive(DRIVE_SPEED, 52, 52, 0);
            //team čtverec v rohu, musíme vyhodit marker
            encoderDrive(TURN_SPEED,);
            encoderDrive(DRIVE_SPEED,80, 80, 0);
            //cesta do kráteru

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
        public void encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutS) {
            int newLeftTarget;
            int newRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

