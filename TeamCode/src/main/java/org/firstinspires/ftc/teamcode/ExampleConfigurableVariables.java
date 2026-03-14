package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// The Turret class would typically be defined in a separate file. I'm defining it in the same
// file to make it easier to follow the example
//
// Alternatively, if you're working on a simple teleop program for testing, there will not be a
// second class like I've shown with the Turret class. You'll just define your Configurable
// variables inside the Teleop class and use those variables inside the the Teleop class
class Turret {
    // I'm passing the telemetry object into the Turret class show I can print the variable values
    // to the driver's station from inside the class
    public Telemetry telem;
    public int myInteger;
    public int[] myArray;

    // This is the constructor method for the Turret class. I'm using this to show the affect of
    // passing either immutable (thisInteger) or mutable (thisArray) variables into a constructor
    public Turret(Telemetry telemetry, int thisInteger, int[] thisArray) {
        telem = telemetry;
        myInteger = thisInteger;  // immutable
        myArray = thisArray; // mutable
    }
    // this is a standard method. I'm showing how immutable variable can be passed into a method
    // from the teleop program and immediate used
    public void setPower(double doubleParam, String stringParam, boolean booleanParam) {

        // variables that were passed into the constructor
        telem.addData("variables passed into the constructor", "---");
        telem.addData("integer (immutable)=", myInteger);
        telem.addData("array (immutable)", myArray[0]);

        // variables that were passed into this method directly
        telem.addData("variables passed into the method", "---");
        telem.addData("double=", doubleParam);
        telem.addData("string=", stringParam);
        telem.addData("boolean=", booleanParam);
        telem.update();
    }
}

// this line is called a class "decorator". Adding "@Configurable" here causes certain variables
// defined inside the class to be configurable through the Panels dashboard
@Configurable
@TeleOp(name="ExampleConfigurableVariables", group="Linear Opmode")
public class ExampleConfigurableVariables extends LinearOpMode {
    // Immutable variables: Most of your standard variable types such as integer, double, float,
    // boolean and string are "immutable". This means when you pass the variable into a method
    // only the current value gets passed into the method.
    //
    // Mutable variables: Complex variable types such as lists and sets are "mutable". When
    // you pass an mutable variable into a method, only a pointer to the variable is passed into the
    // method
    //
    // This difference is important for Configurable variables.
    //
    // In the following example, we pass immutable variables directly into a regular class method
    // and then immediate use the values inside the method. As we change the values of these
    // Configurable variables in the Panels dashboard, the new value appears in the class method
    //
    // We also pass both an immutable variable (cfgInteger) and a mutable variable (cfgArray) into
    // the class constructor method (Turret). What you see is that, when this immutable variable
    // is used later in the setPower() method, the value doesn't change from what we initially
    // passed into the constructor. BUT, the mutuable variable we passed into the constructor does
    // change because only a pointer to the variable was passed into the constructor, so as you
    // change the value in the main Teleop program the value also changes inside the class.

    // Only public static variables will appear in the Configurables screen in the Panels dashboard

    // immutable variables
    public static int cfgInteger = 0;
    public static double cfgDouble = 0.0;
    public static String cfgString = "test";
    public static boolean cfgBoolean = true;

    // mutable variables
    public static int[] cfgArray = {1};

    @Override
    public void runOpMode() {

        waitForStart();

        // here we pass immutable (cfgInteger) and mutable (cfgArray) Configurable variables into
        // the class constructor to show how they behave differently
        Turret myTurret = new Turret(telemetry, cfgInteger, cfgArray);

        while (opModeIsActive()) {
            // here we pass immutable variables into a regular method
            myTurret.setPower(cfgDouble, cfgString, cfgBoolean);
        }
    }
}
