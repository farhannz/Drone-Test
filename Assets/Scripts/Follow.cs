using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Follow : MonoBehaviour
{
    public GameObject obj;
    public float xOffset = 0;
    public float yOffset = 0;
    public float zOffset = -20;
    public float cameraTurnSpeed = 5.0f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(obj.transform.position.x + xOffset, obj.transform.position.y + yOffset, obj.transform.position.z + zOffset);
        transform.Rotate(Vector3.up * obj.transform.rotation.y);
        zOffset += Input.GetAxis("Mouse ScrollWheel");
    }
}
