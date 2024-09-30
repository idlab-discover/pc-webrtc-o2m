using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using UnityEngine;
using System.Runtime.InteropServices;
using Debug = UnityEngine.Debug;
public unsafe class SessionManager : MonoBehaviour
{
    [DllImport("PCStreamingPlugin")]
    static extern int setup_connection(string addr, UInt32 port, UInt32 self_port);
    [DllImport("PCStreamingPlugin")]
    private static extern int start_listening();

    [DllImport("PCStreamingPlugin")]
    private static extern int send_data_to_server(byte* data, UInt32 size);
    public PCReceiverPrefab ReceiverPrefab;
    public Camera cam;
    public GameObject camOffset;
    public GameObject rig;
    private Process peerProcess;
    private SessionInfo sessionInfo;
    private PCReceiverPrefab receiver;

    public float CameraUpdateTimer = 0.300f;
    private float currentCameraUpdateTimer = 0;
    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = 120;
        sessionInfo = SessionInfo.CreateFromJSON(Application.dataPath + "/config/session_config.json");
        string clientAddress = $"{sessionInfo.cltAddr}:{sessionInfo.cltPort}";
        string selfAddress = $"{sessionInfo.selfAddr}:{sessionInfo.selfPort}";
        // If this functions returns 0 everything is fine, 1=>WSA startup error, 2=>socket creation error, 3=>sendto (L4S client) error
        Debug.Log(setup_connection(sessionInfo.cltAddr, (uint)sessionInfo.cltPort, (uint)sessionInfo.selfPort));
        start_listening();
        
        peerProcess = new Process();
        peerProcess.StartInfo.FileName = Application.dataPath + "/peer/client.exe";
        peerProcess.StartInfo.Arguments = $"-p {selfAddress} -clt {clientAddress} -o -srv {sessionInfo.srvAddr}";
        Debug.Log(peerProcess.StartInfo.Arguments);
        peerProcess.StartInfo.CreateNoWindow = false;
        peerProcess.StartInfo.Arguments = $"/K {peerProcess.StartInfo.FileName} {peerProcess.StartInfo.Arguments}";
        peerProcess.StartInfo.FileName = "CMD.EXE";
        if (!peerProcess.Start())
        {
            Debug.LogError("Failed to start peer process");
            peerProcess = null;
            return;
        }
        CreatePrefabs();
    }

    // Update is called once per frame
    void Update()
    {
        currentCameraUpdateTimer += Time.deltaTime;
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        camOffset.transform.eulerAngles = new Vector3(camOffset.transform.rotation.eulerAngles.x + (100 * verticalInput * Time.deltaTime), camOffset.transform.rotation.eulerAngles.y + (100 * horizontalInput * Time.deltaTime), 0);
        if (currentCameraUpdateTimer > CameraUpdateTimer)
        {
            Matrix4x4 worldToCameraMatrix = cam.worldToCameraMatrix;
            Matrix4x4 projectionMatrix = cam.projectionMatrix;
            Debug.Log(worldToCameraMatrix);
            Debug.Log(projectionMatrix);
            string output = "";
            for (int i = 0; i < 4; i++)
            {
                Vector4 row = worldToCameraMatrix.GetRow(i);
                for (int j = 0; j < 4; j++)
                {
                    output += row[j].ToString("0.00000") + ";";
                }
            }
            for (int i = 0; i < 4; i++)
            {
                Vector4 row = projectionMatrix.GetRow(i);
                for (int j = 0; j < 4; j++)
                {
                    output += row[j].ToString("0.00000") + ";";
                }
            }

            Vector3 pos = rig.transform.position;
            output += $"{pos.x};{pos.y + 1};{pos.z};";
            output += $"{receiver.transform.position.x};{receiver.transform.position.y};{receiver.transform.position.z};";
            byte[] outputBytes = Encoding.ASCII.GetBytes(output);
            unsafe
            {
                fixed (byte* bufferPointer = outputBytes)
                {
                   int n = send_data_to_server(bufferPointer, (uint)outputBytes.Length);
                   Debug.Log(n);
                    Debug.Log(outputBytes.Length);
                }
            }
            currentCameraUpdateTimer -= CameraUpdateTimer;
        }
    }

    void CreatePrefabs() {
        receiver = Instantiate(ReceiverPrefab, transform.position, transform.rotation);
        receiver.PCReceiver.SessionInfo = sessionInfo;
    }
}
