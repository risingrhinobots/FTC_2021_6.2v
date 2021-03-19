package org.firstinspires.ftc.teamcode.auto.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum_hardware;



@Autonomous(name="Mecanum encoder", group="mecanum")
public class mecanum_encoder extends LinearOpMode {

    Mecanum_hardware mecanum = new Mecanum_hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanum.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        mecanum.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mecanum.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                mecanum.frontRight.getCurrentPosition(),
                mecanum.frontRight.getCurrentPosition());
        telemetry.update();

        waitForStart();


        //forward
        encoderDrive(DRIVE_SPEED,40,40,40,40,10);

        sleep(2000);

        //backward
        encoderDrive(DRIVE_SPEED,-40,-40,-40,-40,10);

        sleep(2000);

        //strafe right
        encoderDrive(DRIVE_SPEED,40,-40,-40,40,10);

        sleep(2000);

        //strafe left
        encoderDrive(DRIVE_SPEED,-40,40,40,-40,10);

        sleep(2000);



    }

    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches,
                             double backleftInches, double backrightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = mecanum.frontRight.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = mecanum.frontLeft.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newBackLeftTarget = mecanum.backLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newBackRightTarget = mecanum.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            mecanum.frontRight.setTargetPosition(newFrontRightTarget);
            mecanum.frontLeft.setTargetPosition(newFrontLeftTarget);
            mecanum.backLeft.setTargetPosition(newBackLeftTarget);
            mecanum.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            mecanum.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            mecanum.frontRight.setPower(Math.abs(speed));
            mecanum.frontLeft.setPower(Math.abs(speed));
            mecanum.backRight.setPower(Math.abs(speed));
            mecanum.backLeft.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanum.frontRight.isBusy() && mecanum.frontLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        mecanum.frontRight.getCurrentPosition(),
                        mecanum.frontLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mecanum.frontRight.setPower(0);
            mecanum.frontLeft.setPower(0);
            mecanum.backRight.setPower(0);
            mecanum.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            mecanum.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanum.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanum.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanum.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
