using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FlightController : MonoBehaviour
{

    // Propeller gameobject
    public GameObject propellerFL;
    public GameObject propellerFR;
    public GameObject propellerBL;
    public GameObject propellerBR;


    [Header("Internal")]
    public float maxForce;
    public float maxPitchAngle; // Degree
    public float maxRollAngle;  // Degree
    public float moveSpeed;
    public float yawSpeed;

    [SerializeField]
    private float thrust;
    [SerializeField]
    private float pitchDir;
    [SerializeField]
    private float yawDir;
    [SerializeField]
    private float rollDir;
    // Drone Rigidbody
    Rigidbody droneRb;

    // PID Gains
    // {Kp, Ki, Kd}
    // { x,  y,  z}
    public Vector3 pitchGains;
    public Vector3 rollGains;
    public Vector3 yawGains;
    public Vector4 altitudeGains; // {Kp, Ki, Kd, Anti WindUp Cutoff}

    public float altitude; // meter
    public bool manual = true;


    private PIDController pitchPID;
    private PIDController rollPID;
    private PIDController yawPID;
    private PIDController altitudePID;


    void Start()
    {
        droneRb = GetComponent<Rigidbody>();

        pitchPID = new PIDController();
        rollPID = new PIDController();
        yawPID = new PIDController();
        altitudePID = new PIDController();
    }

    void FixedUpdate()
    {
        //GetAltitudeError(altitude);
        //GetPitchError();
        InputControls(); // Reading the controller input

        MotorMixingAlgorithm();

    }

    
    void MotorMixingAlgorithm()
    {
        // Getting the pitch error of the desired pitch angle
        // Direction range is [-1,1]
        float pitchError = GetPitchError(pitchDir * maxPitchAngle);

        // In Unity Coordinate System, Left is positive and Right is negative
        // Output from GetRollError, Left is Negative and Right is Positive
        // Thus we multiply it by -1 to change it to unity coordinate system
        // Direction range is [-1,1]
        float rollError = GetRollError(rollDir * maxRollAngle) *-1f;
        float altitudeError = GetAltitudeError(altitude);

        float altitudePIDOutput = altitudePID.GetOutputPID(altitudeGains, altitudeError);
        float pitchPIDOutput = pitchPID.GetOutputPID(pitchGains, pitchError);
        float rollPIDOutput = rollPID.GetOutputPID(rollGains, rollError);

        Debug.Log("pitchError : " + pitchError + " rollError : " + rollError + " altitudeError : " + altitudeError);
        Debug.Log("pitchPidOutput : " + pitchPIDOutput + " rollPIDOutput : "+ rollPIDOutput + " altitudePIDOutput : " + altitudePIDOutput);

        // FL
        float propellerForceFL = thrust + altitudePIDOutput - pitchPIDOutput - rollPIDOutput;

        // FR
        float propellerForceFR = thrust + altitudePIDOutput - pitchPIDOutput + rollPIDOutput;

        // BL
        float propellerForceBL = thrust + altitudePIDOutput + pitchPIDOutput - rollPIDOutput;

        // BR
        float propellerForceBR = thrust + altitudePIDOutput + pitchPIDOutput + rollPIDOutput;


        Vector4 forces = new Vector4(propellerForceFL, propellerForceFR, propellerForceBL, propellerForceBR);
        Debug.Log(forces);
        AddForceToPropeller(propellerFL, propellerForceFL);
        AddForceToPropeller(propellerFR, propellerForceFR);
        AddForceToPropeller(propellerBL, propellerForceBL);
        AddForceToPropeller(propellerBR, propellerForceBR);

    }


    void InputControls()
    {
        if (manual)
        {
            thrust = Input.GetAxis("Vertical"); // If manual control is used
            //if (Input.GetAxis("Vertical")>0)
            //{
            //    thrust += Input.GetAxis("Vertical");
            //}
            //if (Input.GetAxis("Vertical") < 0)
            //{
            //    thrust += Input.GetAxis("Vertical");
            //}
            //thrust = Mathf.Clamp(thrust, 0f, maxForce);

            yawDir = Input.GetAxis("Horizontal");
            pitchDir = Input.GetAxis("Pitch");
            rollDir = Input.GetAxis("Roll");
        }
    }

    void AddForceToPropeller(GameObject propellerObj, float propellerForce)
    {
        Vector3 propellerUp = propellerObj.transform.up;

        Vector3 propellerPos = propellerObj.transform.position;

        droneRb.AddForceAtPosition(propellerUp * propellerForce, propellerPos);

        //Debug
        Debug.DrawRay(propellerPos, propellerUp * (1f + 20f), Color.red);
    }

    private float GetAltitudeError(float desired)
    {
        float error;
        RaycastHit hit;
        Vector3 pos = transform.position;
        Physics.Raycast(pos,Vector3.down,out hit);

        error = desired - hit.distance;
        Debug.DrawRay(pos, Vector3.down, Color.black);
        Debug.Log(error);
        return error;
    }

    private float GetPitchError(float desired) // Pitch angle
    {
        float error;
        Vector3 rotation = transform.rotation.eulerAngles;

        float xAngle = rotation.x;

        //Make sure the angle is between 0 and 360
        xAngle = WrapAngle(xAngle);

        //This angle going from 0 -> 360 when pitching forward
        //So if angle is > 180 then it should move from 0 to 180 if pitching back
        if (xAngle > 180f && xAngle < 360f)
        {
            xAngle = 360f - xAngle;

            //-1 so we know if we are pitching back or forward
            xAngle *= -1f;
        }

        error = Mathf.Clamp(desired-xAngle, -maxPitchAngle, maxPitchAngle);

        //Debug.Log(rotation + " " + error);
        return error;
    }
    private float GetRollError(float desired) // Roll angle
    {
        float error;
        Vector3 rotation = transform.rotation.eulerAngles;

        float zAngle = rotation.z;

        //Make sure the angle is between 0 and 360
        zAngle = WrapAngle(zAngle);

        if (zAngle > 180f && zAngle < 360f)
        {
            zAngle = 360f - zAngle;

            //-1 so we know if we are rolling left or right
            zAngle *= -1f;
        }

        error = Mathf.Clamp(desired - zAngle, -maxRollAngle, maxRollAngle);

        //Debug.Log(rotation + " " + error);
        return error;
    }


    //Wrap between 0 and 360 degrees
    float WrapAngle(float inputAngle)
    {
        //The inner % 360 restricts everything to +/- 360
        //+360 moves negative values to the positive range, and positive ones to > 360
        //the final % 360 caps everything to 0...360
        return ((inputAngle % 360f) + 360f) % 360f;
    }
}
