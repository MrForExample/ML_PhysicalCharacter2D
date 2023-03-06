using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FunctionTest : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Vector3 v = Quaternion.AngleAxis(90f, Vector3.forward) * Vector3.right;
        Debug.Log(v);
    }
}
