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
    public Vector3 altitudeGains;

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

        MotorMixedAlgorithm();

    }

    
    void MotorMixedAlgorithm()
    {
        float pitchError = GetPitchError();
        float rollError = GetRollError() * -1f;

        float pitchOutputPID = pitchPID.GetOutputPID(pitchGains, pitchError);
        float rollOutputPID = rollPID.GetOutputPID(rollGains, rollError);

        //Calculate the propeller forces
        //FR
        float propellerForceFR = thrust  + (pitchOutputPID + rollOutputPID);

        //Add steering
        propellerForceFR -= pitchDir * thrust * moveSpeed;
        propellerForceFR -= rollDir * thrust;


        //FL
        float propellerForceFL = thrust + (pitchOutputPID - rollOutputPID);

        propellerForceFL -= pitchDir * thrust * moveSpeed;
        propellerForceFL += rollDir * thrust;


        //BR
        float propellerForceBR = thrust  + (-pitchOutputPID + rollOutputPID);

        propellerForceBR += pitchDir * thrust * moveSpeed;
        propellerForceBR -= rollDir * thrust;


        //BL 
        float propellerForceBL = thrust  + (-pitchOutputPID - rollOutputPID);

        propellerForceBL += pitchDir * thrust * moveSpeed;
        propellerForceBL += rollDir * thrust;


        //Clamp
        propellerForceFR = Mathf.Clamp(propellerForceFR, 0f, maxForce);
        propellerForceFL = Mathf.Clamp(propellerForceFL, 0f, maxForce);
        propellerForceBR = Mathf.Clamp(propellerForceBR, 0f, maxForce);
        propellerForceBL = Mathf.Clamp(propellerForceBL, 0f, maxForce);
        Debug.Log(propellerForceFR + " " + propellerForceFL + " " + propellerForceBR + " " + propellerForceBL);
        //Add the force to the propellers
        AddForceToPropeller(propellerFR, propellerForceFR);
        AddForceToPropeller(propellerFL, propellerForceFL);
        AddForceToPropeller(propellerBR, propellerForceBR);
        AddForceToPropeller(propellerBL, propellerForceBL);

        //Yaw
        //Minimize the yaw error (which is already signed):
        float yawError = droneRb.angularVelocity.y;

        float yawOutputPid = yawPID.GetOutputPID(yawGains, yawError);

        //First we need to add a force (if any)
        droneRb.AddTorque(transform.up * yawDir *  yawSpeed * thrust);

        //Then we need to minimize the error
        droneRb.AddTorque(transform.up * thrust * yawOutputPid * -1f);
    }


    void InputControls()
    {
        if (manual)
        {
            //thrust = Input.GetAxis("Vertical"); // If manual control is used
            if (Input.GetAxis("Vertical")>0)
            {
                thrust += Input.GetAxis("Vertical");
            }
            if (Input.GetAxis("Vertical") < 0)
            {
                thrust += Input.GetAxis("Vertical");
            }
            thrust = Mathf.Clamp(thrust, 0f, maxForce);

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
        //Debug.DrawRay(propellerPos, propellerUp * 1f, Color.red);
    }

    private float GetAltitudeError(float desired)
    {
        float error;
        RaycastHit hit;
        Vector3 pos = transform.position;
        Physics.Raycast(pos,Vector3.down,out hit);

        error = hit.distance - desired;
        Debug.DrawRay(pos, Vector3.down, Color.black);
        Debug.Log(error);
        return error;
    }

    private float GetPitchError()
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

        error = Mathf.Clamp(xAngle, -maxPitchAngle, maxPitchAngle);

        //Debug.Log(rotation + " " + error);
        return error;
    }
    private float GetRollError()
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

        error = Mathf.Clamp(zAngle, -maxRollAngle, maxRollAngle);

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
