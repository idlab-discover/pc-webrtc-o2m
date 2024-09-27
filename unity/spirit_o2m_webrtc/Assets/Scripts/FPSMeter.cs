using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class FPSMeter : MonoBehaviour
{
    public Text fpsText;
    public float deltaTime;
    // Start is called before the first frame update
    void Start()
    {
        //fpsText = GetComponent<Text>();
        //Application.targetFrameRate = 120;
    }



    void Update()
    {
        deltaTime += (Time.deltaTime - deltaTime) * 0.1f;
        float fps = 1.0f / deltaTime;
        fpsText.text = Mathf.Ceil(fps).ToString();
    }
}
