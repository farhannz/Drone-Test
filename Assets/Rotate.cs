using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotate : MonoBehaviour
{
    public float rotationSpeed = 25f;

    // Update is called once per frame
    void Update()
    {
        transform.Rotate(Vector3.up*rotationSpeed + Quaternion.identity.eulerAngles);
    }
}
