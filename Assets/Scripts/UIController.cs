using System.Collections;
using System.Collections.Generic;
using UnityEngine;
// using UnityEngine.UIElements;
using UnityEngine.UI;
public class UIController : MonoBehaviour
{
    public Rigidbody rb;
    public bool isDebug = false;
    Vector3 prevPosition;
    float speedMeasured = 0;
    // public Label testLabel;
    private Text speed;
    public GameObject debug;
    public GameObject debugInfo;
    private Canvas canvas;

    public FlightController fcs;
    void Start(){
        // var abc = GetComponent<UIDocument>().rootVisualElement;
        // testLabel = abc.Q<Label>("TestLabel");
        // speed = GetComponent<Text>();
        speed = this.gameObject.transform.GetChild(0).GetComponent<Text>();
        prevPosition = rb.transform.position;
    }

    void FixedUpdate(){
        // testLabel.text = rb.velocity + " m/s";
        // testLabel.style.display = DisplayStyle.Flex;
        speedMeasured = (rb.transform.position - prevPosition).magnitude / Time.fixedDeltaTime;
        prevPosition = rb.transform.position;
        speed.text = "Speed : " + (int)(speedMeasured*3.6f) + " km/h";

        if(isDebug){
            debug.SetActive(true);
            debugInfo.SetActive(true);
            Text infoText = debugInfo.GetComponent<Text>();
            infoText.text = "";
        }
        else{
            debug.SetActive(false);
            debugInfo.SetActive(false);
        }
    }
}
