using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotate : MonoBehaviour
{
    public float rotationSpeed = 1f;
    public float rotationFactor = 3f;
    // Update is called once per frame
    void Update()
    {
        transform.Rotate(Vector3.up*(Mathf.Clamp(rotationSpeed*rotationFactor,-25,25)) + Quaternion.identity.eulerAngles);
    }
}
