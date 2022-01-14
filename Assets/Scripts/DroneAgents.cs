using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class DroneAgents : Agent
{
    public Rigidbody rb;
    public FlightController fcs;
    public GameObject cube;
    public override void Initialize(){
        
    }
    public override void OnEpisodeBegin()
    {
        if(cube != null){
            cube.transform.localScale = new Vector3(Random.Range(4f,11f),Random.Range(4f,11f),cube.transform.localScale.z);
            cube.transform.localPosition = new Vector3(Random.Range(-8,-2f),Random.Range(0,4f),cube.transform.localPosition.z);
        }
        // cube.transform.localScale = new Vector3(Random.Range(4f,11f),Random.Range(4f,11f),cube.transform.localScale.z);
        // cube.transform.localPosition = new Vector3(Random.Range(-8,-2f),Random.Range(0,4f),cube.transform.localPosition.z);
        rb.transform.localPosition = new Vector3(-5.17f,4,-8.68f);
        rb.velocity = Vector3.zero;
        rb.transform.localRotation = new Quaternion(0,0,0,1);
    }
    public override void OnActionReceived(ActionBuffers actions){
        fcs.setMovement(Mathf.Clamp(actions.ContinuousActions[0],.3f,1f), Mathf.Clamp(actions.ContinuousActions[1],-1f,1f));
        // Debug.Log(actions.ContinuousActions);
    }

    public void OnTriggerEnter(Collider collider){
        if(collider.gameObject.CompareTag("Goal")){
            AddReward(.5f); 
            EndEpisode();
        }
        else if(collider.gameObject.CompareTag("Obstacle")){
            AddReward(-.2f);
            EndEpisode();
        }
        
    }
    // public override void OnEpisodeBegin(){

    // }

    public override void Heuristic(in ActionBuffers actionsOut){
        var continuous = actionsOut.ContinuousActions;
        continuous[0] = fcs.Pitch;
        continuous[1] = fcs.Roll;
    }
}
