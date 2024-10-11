

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Auto", group="Autonomous")
@Disabled
public class AutoTrainingCamp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //todo If robot doesnt drive correct directions this is the culprit
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        ResetWheelEncoders();
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // turn(10000,0.5);
        double lengthOfTheField = 0; // inches
        //todo Step 2:
        //  find the length of the field
        // Uncomment the 2 lines below
        // The goal is to go from 1 corner of the field to the other. using only verticalDrive and turn.
        // Hint: legnthOfTheField variable could also be called distanceFromStartToEnd

//        verticalDrive(lengthOfTheField,0.5);
//        turn(1,0.5); // make sure to change the degrees.
        //todo Step 3:
        // return back to start using horizontalDrive and turn
//        horizontalDrive(lengthOfTheField,0.5);
    }

    /**
     *  Turn inches into a tick value.
     * @param inches the amount of inches to drive
     * @return amount of ticks per inch * the amount of inches
     */
    public int inchesToTicks(int inches) {
        int ticksPerRotation = 0; // todo can be found on motors information page
        double wheelDiameter = 0; // todo Measure the wheels Hint: Wheel has a website as well.
        double doubleTicks = 0; //todo figure out this line. Hint: Circumference of a circle is 2 * Radius * 3.14 .
        int ticksInt = (int) Math.round(doubleTicks);
        return ticksInt;
    }

    /**
     * Turn Degrees into ticks
     * @param degrees
     * @return ticks for the motors to use.
     */
    public int degreesToTicks(int degrees) {
        double rotations = 0; // todo use turn(10000,0.5); and count the amount of rotations. Line 46
        double Ddoubleticks = (degrees / 360.0 * (10000 / rotations));
        int ticksint = (int) Math.round(Ddoubleticks);
        return ticksint;
    }


    /**
     * Resets the state of the motor encoders
     */
    public void ResetWheelEncoders(){
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    /**
     *  Drive forward using inches use negative for going backward
     * @param inches the distance the robot goes
     * @param power how fast the robot goes
     */
    public void verticalDrive(int inches, double power) {
        leftFrontDrive.setTargetPosition(inchesToTicks(inches));
        rightFrontDrive.setTargetPosition(inchesToTicks(inches));
        leftBackDrive.setTargetPosition(inchesToTicks(inches));
        rightBackDrive.setTargetPosition(inchesToTicks(inches));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        telemetry.addData("here ", 1);
        telemetry.update();
        //noinspection StatementWithEmptyBody
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                && leftBackDrive.isBusy() && rightBackDrive.isBusy()
                && opModeIsActive());
        telemetry.addData("here ", 2);
        telemetry.update();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        ResetWheelEncoders();
    }

    /**
     *  Strafe right using inches and can go left using negative inches.
     * @param inches how far the robot goes
     * @param power how fast the robot goes
     */
    public void horizontalDrive(int inches, double power) {
        leftFrontDrive.setTargetPosition(+inchesToTicks(inches));
        rightFrontDrive.setTargetPosition(-inchesToTicks(inches));
        leftBackDrive.setTargetPosition(-inchesToTicks(inches));
        rightBackDrive.setTargetPosition(+inchesToTicks(inches));

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        //noinspection StatementWithEmptyBody
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy() && opModeIsActive()) ;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        ResetWheelEncoders();
    }

    /**
     *  Turn the robot
     * @param degrees How many degrees for the robot to turn
     * @param power How fast the robot turns
     */
    public void turn(int degrees, double power) {
        // todo Remove comments
//        leftFrontDrive.setTargetPosition(-degreesToTicks(degrees));
//        rightFrontDrive.setTargetPosition(degreesToTicks(degrees));
//        leftBackDrive.setTargetPosition(-degreesToTicks(degrees));
//        rightBackDrive.setTargetPosition(degreesToTicks(degrees));
        // todo remove next 4 lines
        leftFrontDrive.setTargetPosition(-(degrees));
        rightFrontDrive.setTargetPosition((degrees));
        leftBackDrive.setTargetPosition(-(degrees));
        rightBackDrive.setTargetPosition((degrees));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        //noinspection StatementWithEmptyBody
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy() && opModeIsActive()) ;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        ResetWheelEncoders();
    }
}
