package org.firstinspires.ftc.teamcode.auto.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum_hardware;


@Autonomous(name="Blue_right_C", group="mecanum")
public class Blue_right_C extends LinearOpMode {

    Mecanum_hardware mecanum = new Mecanum_hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1500;
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
        encoderDrive(DRIVE_SPEED,65,65,65,65,50);

        //strafe left
        encoderDrive(DRIVE_SPEED,-20,20,20,-20,50);

        //shoot 3 rings
        sleep(3000);

        //forward to line up to wobble zone C
        encoderDrive(DRIVE_SPEED, 30,30,30,30,50);

        //strafe Left into wobble zone C
        encoderDrive(DRIVE_SPEED,-10,10,10,-10,50);

        //strafe Right out of wobble zone C
        encoderDrive(DRIVE_SPEED,20,-20,-20,20,50);

        //Reverse to Launch Line
        encoderDrive(DRIVE_SPEED,-35,-35,-35,-35,50);


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
            newFrontLeftTarget = mecanum.frontLeft.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = mecanum.frontRight.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newBackLeftTarget = mecanum.backLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newBackRightTarget = mecanum.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            mecanum.frontRight.setTargetPosition(newFrontRightTarget);
            mecanum.frontLeft.setTargetPosition(newFrontLeftTarget);
            mecanum.backLeft.setTargetPosition(newBackLeftTarget);
            mecanum.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            mecanum.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            mecanum.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            mecanum.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            mecanum.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            mecanum.frontRight.setVelocity(speed);
            mecanum.frontLeft.setVelocity(speed);
            mecanum.backRight.setVelocity(speed);
            mecanum.backLeft.setVelocity(speed);

            //mecanum.frontRight.setPower(Math.abs(speed));
            //mecanum.frontLeft.setPower(Math.abs(speed));
            //mecanum.backRight.setPower(Math.abs(speed));
            //mecanum.backLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanum.frontLeft.isBusy() && mecanum.frontRight.isBusy() &&
                            mecanum.backRight.isBusy() && mecanum.backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        mecanum.frontLeft.getCurrentPosition(),
                        mecanum.frontRight.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            mecanum.frontRight.setVelocity(0);
            mecanum.frontLeft.setVelocity(0);
            mecanum.backRight.setVelocity(0);
            mecanum.backLeft.setVelocity(0);

            // Turn off RUN_TO_POSITION
            mecanum.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mecanum.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mecanum.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mecanum.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(1500);   // optional pause after each move
        }
    }
}
