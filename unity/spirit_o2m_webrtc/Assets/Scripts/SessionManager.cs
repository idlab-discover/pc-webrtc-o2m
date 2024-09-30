using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;
public class SessionManager : MonoBehaviour
{
    [DllImport("PCStreamingPlugin")]
    static extern int setup_connection(string addr, UInt32 port, UInt32 self_port);
    [DllImport("PCStreamingPlugin")]
    private static extern int start_listening();
    public PCReceiverPrefab ReceiverPrefab;

    private Process peerProcess;
    private SessionInfo sessionInfo;
    private PCReceiverPrefab receiver;
    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = 120;
        sessionInfo = SessionInfo.CreateFromJSON(Application.dataPath + "/config/session_config.json");
        string clientAddress = $"{sessionInfo.cltAddr}:{sessionInfo.cltPort}";
        string selfAddress = $"{sessionInfo.selfAddr}:{sessionInfo.selfPort}";
        // If this functions returns 0 everything is fine, 1=>WSA startup error, 2=>socket creation error, 3=>sendto (L4S client) error
        Debug.Log(setup_connection(SessionInfo.cltAddr, (uint)SessionInfo.cltPort, (uint)SessionInfo.selfPort));
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
        
    }

    void CreatePrefabs() {
        receiver = Instantiate(ReceiverPrefab, transform.position, transform.rotation);
        receiver.PCReceiver.SessionInfo = sessionInfo;
    }
}
