using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class FlightController : MonoBehaviour
{

    // Propeller gameobject
    public GameObject propellerFL;
    public GameObject propellerFR;
    public GameObject propellerBL;
    public GameObject propellerBR;
    
    public Toggle isManual; 

    [Header("Internal")]
    public float thrustFactor = 1.4f;
    public float maxPitchAngle; // Degree
    public float maxRollAngle;  // Degree
    public float moveSpeed;
    public float yawSpeed;

    [Header("Turbulence")]
    public float minTurbForce = 0f;
    public float maxTurbForce = 2f;
    [Range(0f,1f)]
    public float turbSlider;
    
    public float finalValue;

    [Header("Direction")]
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
    private float maxForce; // Max force for each propellers

    // PID Gains
    // {Kp, Ki, Kd}
    // { x,  y,  z}
    public Vector4 pitchGains;
    public Vector4 rollGains;
    public Vector3 yawGains;
    public Vector4 altitudeGains; // {Kp, Ki, Kd, Anti WindUp Cutoff}
    public float desiredAltitude; // meter
    // public float altitude; // meter
    public bool manual = true;


    private PIDController pitchPID;
    private PIDController rollPID;
    private PIDController yawPID;
    private PIDController altitudePID;

    // private bool Flag = false;
    float totalTimeElapsed = 0f;



    
    void Start()
    {
        droneRb = GetComponent<Rigidbody>();
        maxForce = 6f * droneRb.mass;
        pitchPID = new PIDController();
        rollPID = new PIDController();
        yawPID = new PIDController();
        altitudePID = new PIDController();
    }

    void FixedUpdate()
    {
        finalValue = Mathf.Lerp(minTurbForce,maxTurbForce,turbSlider);
        //GetAltitudeError(altitude);
        //GetPitchError();
        InputControls(); // Reading the controller input

        MotorMixingAlgorithm(); // Motor Mixing Algorithm for the propeller
        totalTimeElapsed += Time.fixedDeltaTime;
        // Debug.Log(droneRb.transform.position.y +","+ totalTimeElapsed +";");
        // Debug.LogFormat("{0},{1}\n",droneRb.transform.position.y,totalTimeElapsed);
    }

    // void LateUpdate(){
    //     if(manual){
            
    //     }
    // }
    void MotorMixingAlgorithm()
    {
        // Getting the pitch error of the desired pitch angle
        // Direction range is [-1,1]
        AddForceToPropeller(propellerFL, (finalValue * Random.Range(-1,1)));
        AddForceToPropeller(propellerFR, (finalValue * Random.Range(-1,1)));
        AddForceToPropeller(propellerBL, (finalValue * Random.Range(-1,1)));
        AddForceToPropeller(propellerBR, (finalValue * Random.Range(-1,1)));
        float pitchError = GetPitchError(pitchDir * maxPitchAngle);

        // In Unity Coordinate System, Left is positive and Right is negative
        // Direction range is [-1,1]
        float rollError = GetRollError(rollDir * maxRollAngle);
        float altitudeError = GetAltitudeError(desiredAltitude);
        float altitudePIDOutput = altitudePID.GetOutputPID(altitudeGains, altitudeError);
        float pitchPIDOutput = pitchPID.GetOutputPID(pitchGains, pitchError);
        float rollPIDOutput = rollPID.GetOutputPID(rollGains, rollError);
        
        // Debug.Log("pitchError : " + pitchError + " rollError : " + rollError + " altitudeError : " + altitudeError);
        // Debug.Log("pitchPidOutput : " + pitchPIDOutput + " rollPIDOutput : "+ rollPIDOutput + " altitudePIDOutput : " + altitudePIDOutput);
        altitudePIDOutput = Mathf.Clamp(altitudePIDOutput,0.5f*droneRb.mass,maxForce);
        pitchPIDOutput    = Mathf.Clamp(pitchPIDOutput,-maxForce,maxForce);
        rollPIDOutput     = Mathf.Clamp(rollPIDOutput,-maxForce,maxForce);
        // FL
        
        float propellerForceFL = thrust*thrustFactor + altitudePIDOutput - pitchPIDOutput - rollPIDOutput ;

        // FR
        float propellerForceFR = thrust*thrustFactor + altitudePIDOutput - pitchPIDOutput + rollPIDOutput ;

        // BL
        float propellerForceBL = thrust*thrustFactor + altitudePIDOutput + pitchPIDOutput - rollPIDOutput ;

        // BR
        float propellerForceBR = thrust*thrustFactor + altitudePIDOutput + pitchPIDOutput + rollPIDOutput ;
    


        // Rotation speed of propellers
        // propellerFL.Rotate.rotationSpeed
        Rotate FL = propellerFL.GetComponent<Rotate>();
        FL.rotationSpeed = propellerForceFL;
        Rotate FR = propellerFR.GetComponent<Rotate>();
        FR.rotationSpeed = -propellerForceFR;
        Rotate BR = propellerBR.GetComponent<Rotate>();
        BR.rotationSpeed = propellerForceBR;
        Rotate BL = propellerBL.GetComponent<Rotate>();
        BL.rotationSpeed = -propellerForceBL;
        
        Vector4 forces = new Vector4(propellerForceFL, propellerForceFR, propellerForceBL, propellerForceBR);
        // Debug.Log(forces);
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
        manual = isManual.isOn;
        if (manual)
        {
            thrust = Input.GetAxis("Vertical"); // If manual control is used
            if(thrust <= -.1f || thrust >= .1f){
                desiredAltitude = droneRb.transform.position.y;
            }
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
        Debug.DrawRay(propellerPos,( propellerUp * propellerForce).normalized, Color.red);
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
    
    public void setMovement(float pitch, float roll){
        pitchDir = pitch;
        rollDir = roll *-1f; // Due to unity coordinate system, Left = Positive, Right = Negative
    }

    public float Pitch {get{return pitchDir;}}
    public float Roll {get{return rollDir;}}
}
