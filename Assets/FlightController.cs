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
    private float thrust = 0f;
    [SerializeField]
    private float pitchDir = 0f;
    [SerializeField]
    private float yawDir = 0f;
    [SerializeField]
    private float rollDir = 0f;
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

        MotorMixingAlgorithm(); // Motor Mixing Algorithm for the propeller 

    }

    
    void MotorMixingAlgorithm()
    {
        // Getting the pitch error of the desired pitch angle
        // Direction range is [-1,1]
        float pitchError = GetPitchError(pitchDir * maxPitchAngle);

        // In Unity Coordinate System, Left is positive and Right is negative
        // Direction range is [-1,1]
        float rollError = GetRollError(rollDir * maxRollAngle);
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


        // yaw

        float yawError = droneRb.angularVelocity.y;

        float yawPIDOutput = yawPID.GetOutputPID(yawGains, yawError);

        droneRb.AddTorque(Vector3.up * yawDir * yawSpeed);

        droneRb.AddTorque(Vector3.up * yawPIDOutput * -1f);
    }


    void InputControls()
    {
        if (manual)
        {
            thrust = Input.GetAxis("Vertical"); // If manual control is used
            yawDir = Input.GetAxis("Horizontal");
            pitchDir = Input.GetAxis("Pitch");
            rollDir = Input.GetAxis("Roll") *-1f; // Due to unity coordinate system, Left = Positive, Right = Negative
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
        LayerMask layerMask = 1 << 6;
        Physics.Raycast(pos,Vector3.down,out hit,150f,layerMask);

        error = desired - hit.distance;
        Debug.DrawRay(pos, Vector3.down*(2f+15f), Color.black);
        //Debug.Log(error);
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

        error = Mathf.Clamp(desired - xAngle, -maxPitchAngle, maxPitchAngle);
        //error = desired - xAngle;

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
        //desired = WrapAngle(desired);
        if (zAngle > 180f && zAngle < 360f)
        {
            zAngle = 360f - zAngle;

            //-1 so we know if we are rolling left or right
            zAngle *= -1f;
        }
        error = Mathf.Clamp(desired - zAngle, -maxRollAngle, maxRollAngle);
        //error = desired - zAngle;
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
    static public float ModularClamp(float val, float min, float max, float rangemin = -180f, float rangemax = 180f)
    {
        var modulus = Mathf.Abs(rangemax - rangemin);
        if ((val %= modulus) < 0f) val += modulus;
        return Mathf.Clamp(val + Mathf.Min(rangemin, rangemax), min, max);
    }
}
