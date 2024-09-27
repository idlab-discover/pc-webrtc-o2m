using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
[System.Serializable]
public class SessionInfo
{
    public string srvAddr;
    public int selfPort;
    public string selfAddr; 
    public int cltPort;
    public string cltAddr;
    

    public static SessionInfo CreateFromJSON(string path)
    {
        return JsonUtility.FromJson<SessionInfo>(File.ReadAllText(path));
    }

}
