using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Propeller : MonoBehaviour
{

    public float rotationSpeed = 10;
    public float forwardSpeed = 5.0f;
    public float flySpeed = 3.0f;
    public List<float> torque;
    public bool clockWise = true;
    public float liftForce = 0;
    public Rigidbody rb;
    public GameObject drone;
    public List<GameObject> propellers;
    public float leftAnalogDeadZone = .3f;
    public float rightAnalogDeadZone = .3f;
    private float initialTorque;
    AudioSource audio;
    private void Awake()
    {
        initialTorque = torque[0];
    }
    // Start is called before the first frame update
    void Start()
    {
        audio = GetComponent<AudioSource>();
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        /* foreach(GameObject props in propellers)
         {
             rb.AddForceAtPosition(Vector3.up * torque, props.transform.position,ForceMode.Acceleration);
             props.transform.Rotate(Quaternion.Euler(0, rotationSpeed, 0).eulerAngles);
         }*/


        // FRONT LEFT
        rb.AddForceAtPosition(Vector3.up * torque[0] + 
            Vector3.forward * (float)System.Math.Round(drone.transform.rotation.x/15.0f,2) * forwardSpeed + 
            Vector3.right * (float)System.Math.Round(drone.transform.rotation.y/15.0f,2) * forwardSpeed  , 
            propellers[0].transform.position, ForceMode.Acceleration);

        //rb.AddRelativeTorque(Vector3.up * torque[0], ForceMode.Acceleration);

        propellers[0].transform.Rotate(Quaternion.Euler(0, rotationSpeed, 0).eulerAngles);
        
        // FRONT RIGHT
        rb.AddForceAtPosition(Vector3.up * torque[1] + 
            Vector3.forward * (float)System.Math.Round(drone.transform.rotation.x/15.0f,2) * forwardSpeed + 
            Vector3.right * (float)System.Math.Round(drone.transform.rotation.y/15.0f,2) * forwardSpeed  , 
            propellers[1].transform.position, ForceMode.Acceleration);

        propellers[1].transform.Rotate(Quaternion.Euler(0, -rotationSpeed, 0).eulerAngles);

        //rb.AddRelativeTorque(Vector3.up * -torque[1], ForceMode.Acceleration);
        
        // BACK LEFT
        rb.AddForceAtPosition(Vector3.up * torque[2] + 
            Vector3.forward * (float)System.Math.Round(drone.transform.rotation.x/15.0f,2) * forwardSpeed + 
            Vector3.right * (float)System.Math.Round(drone.transform.rotation.y/15.0f,2) * forwardSpeed  , 
            propellers[2].transform.position, ForceMode.Acceleration);

        propellers[2].transform.Rotate(Quaternion.Euler(0, rotationSpeed, 0).eulerAngles);

       // rb.AddRelativeTorque(Vector3.up * torque[2], ForceMode.Acceleration);

        // BACK RIGHT
        rb.AddForceAtPosition(Vector3.up * torque[3] + 
            Vector3.forward * (float)System.Math.Round(drone.transform.rotation.x/15.0f,2) * forwardSpeed + 
            Vector3.right * (float)System.Math.Round(drone.transform.rotation.y / 15.0f, 2) * forwardSpeed  , 
            propellers[3].transform.position, ForceMode.Acceleration);

        propellers[3].transform.Rotate(Quaternion.Euler(0, -rotationSpeed, 0).eulerAngles);

       // rb.AddRelativeTorque(Vector3.up * -torque[3], ForceMode.Acceleration);



        

       /* Debug.Log("Magnitude : " + rb.velocity.magnitude);
        if (rb.velocity.magnitude <= forwardSpeed)
        {
            if (drone.transform.rotation.x >= .5f)
            {
                torque[0] = torque[1] = initialTorque + initialTorque* rb.velocity.magnitude * Time.fixedDeltaTime;
                torque[2] = torque[3] = initialTorque - initialTorque * rb.velocity.magnitude * Time.fixedDeltaTime
               *//* Quaternion target = new Quaternion(0, drone.transform.rotation.y, 0, drone.transform.rotation.w);
                drone.transform.rotation = Quaternion.Slerp(transform.rotation, target, -2 * Time.fixedDeltaTime);*//*
            }
            else if (drone.transform.rotation.x <= -.5f)
            {
                *//*Quaternion target = new Quaternion(0, drone.transform.rotation.y, 0, drone.transform.rotation.w);
                drone.transform.rotation = Quaternion.Slerp(transform.rotation, target, 2 * Time.fixedDeltaTime);*//*
            }
        }*/
        /* rb.AddRelativeTorque(torque * Vector3.up);
        *//* rb.angularVelocity = Vector3.up * rotationSpeed * 40;*//*
         liftForce = rb.angularVelocity.magnitude/4;
         rb.AddRelativeForce(Vector3.up * liftForce, ForceMode.Acceleration);*/
        Debug.Log(rb.angularVelocity);
    }
    void Update()
    {

        // droneRb.AddRelativeTorque(Vector3.up * Input.GetAxis("Horizontal") * 5);
        // droneRb.AddRelativeForce(Vector3.up * Input.GetAxis("Vertical") * 5);
        // LEFT ANALOG
        float verticalAxis = Input.GetAxis("Vertical");
        float horizontalAxis = Input.GetAxis("Horizontal");
        // RIGHT ANALOG
        float rightVerticalAxis = Input.GetAxis("Pitch");

        if (verticalAxis > leftAnalogDeadZone)
        {
            audio.pitch = Mathf.Lerp(audio.pitch, 1.2f, 1f * Time.deltaTime);
            DroneUp(verticalAxis);
        }
        else if (verticalAxis < -leftAnalogDeadZone)
        {
            audio.pitch = Mathf.Lerp(audio.pitch, .9f, 1f * Time.deltaTime);
            DroneUp(verticalAxis);
        }
        else if (verticalAxis >= -leftAnalogDeadZone && verticalAxis <= leftAnalogDeadZone)
        {
            audio.pitch = Mathf.Lerp(audio.pitch, 1f, 1f * Time.deltaTime);
            DroneUp(verticalAxis);
        }

        

        if (horizontalAxis > leftAnalogDeadZone)
        {
            DroneYaw(horizontalAxis);
        }
        else if (horizontalAxis < -leftAnalogDeadZone)
        {
            DroneYaw(horizontalAxis);
        }
        else
        {
            DroneYaw(horizontalAxis);
        }




        if (rightVerticalAxis > rightAnalogDeadZone)
        {
            DronePitch(rightVerticalAxis);
        }
        else if (rightVerticalAxis < -rightAnalogDeadZone)
        {
            DronePitch(rightVerticalAxis);
        }
        else
        {
            DronePitch(rightVerticalAxis);
        }

        float rightHorizontalAxis = Input.GetAxis("Roll");

        if (rightHorizontalAxis > rightAnalogDeadZone)
        {
            DroneRoll(rightHorizontalAxis);
        }
        else if (rightHorizontalAxis < -rightAnalogDeadZone)
        {
            DroneRoll(rightHorizontalAxis);
        }
        else
        {
            if(rightVerticalAxis >= -rightAnalogDeadZone && rightVerticalAxis <= rightAnalogDeadZone)
                DroneRoll(rightHorizontalAxis);
        }
        Debug.Log(verticalAxis + " " + horizontalAxis + " " + rightVerticalAxis + " " + rightHorizontalAxis);

    }

    


    public void DroneUp(float dir) // UP or DOWN drone controller
    {

        for(int i = 0; i < 4; ++i)
        {
            if(dir > leftAnalogDeadZone)
            {
                torque[i] = initialTorque * 1.5f * dir;
            }
            else if(dir < -leftAnalogDeadZone)
            {
                torque[i] = initialTorque * .5f * -dir;
            }
            else
            {
                torque[i] = initialTorque;
            }
        }
    }

    public void DroneYaw(float dir) // YAW drone Controller
    {
        if (dir > leftAnalogDeadZone)
        {
            drone.transform.Rotate(Quaternion.Euler(Vector3.up * dir).eulerAngles,Space.World);
            /*rb.AddRelativeTorque(Vector3.up * torque[0] * .5f, ForceMode.Acceleration);*/
        }
        else if (dir < -leftAnalogDeadZone)
        {
            drone.transform.Rotate(Quaternion.Euler(Vector3.up * dir).eulerAngles, Space.World);
            /*rb.AddRelativeTorque(Vector3.up * -torque[1] * .5f, ForceMode.Acceleration);*/
        }
    }
    
    public void DronePitch(float dir) // Pitch drone controller
    {
        if (dir > rightAnalogDeadZone)
        {
            torque[0] = torque[1] = initialTorque * .8f * dir;
            torque[2] = torque[3] = initialTorque * 1.2f * dir;
        }
        else if (dir < -rightAnalogDeadZone)
        {
            torque[0] = torque[1] = initialTorque * 1.2f * -dir;
            torque[2] = torque[3] = initialTorque * .8f * -dir;
        }
        /*else
        {
            for (int i = 0; i < 4; ++i) torque[i] = initialTorque;
        }*/
    }

    public void DroneRoll(float dir)
    {
        if (dir > rightAnalogDeadZone)
        {
            torque[1] = torque[3] = initialTorque * .7f * dir;
            torque[0] = torque[2] = initialTorque * 1.3f * dir;
        }
        else if (dir < -rightAnalogDeadZone)
        {
            torque[1] = torque[3] = initialTorque * 1.3f * -dir;
            torque[0] = torque[2] = initialTorque * .7f * -dir;
        }
        /*else
        {
            for (int i = 0; i < 4; ++i) torque[i] = initialTorque;
        }*/
    }
}
