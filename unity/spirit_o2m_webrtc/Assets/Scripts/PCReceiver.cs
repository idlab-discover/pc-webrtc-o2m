using UnityEngine;
using System;
using UnityEngine.Rendering;
using System.Linq;
using System.Runtime.InteropServices;
using System.Collections;
using static UnityEngine.Mesh;
using Draco;
using System.Threading.Tasks;
using Unity.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using System.IO;
using Unity.VisualScripting;
using UnityEditor.PackageManager;
using UnityEditor.Search;
using System.Collections.Concurrent;
using System.Threading;

public class PCReceiver : MonoBehaviour
{
    [DllImport("PCStreamingPlugin")]
    static extern int setup_connection(string addr, UInt32 port, UInt32 self_port);
    [DllImport("PCStreamingPlugin")]
    private static extern int start_listening();
    [DllImport("PCStreamingPlugin")]
    private static extern int next_frame();
    [DllImport("PCStreamingPlugin")]
    // You should technically be able to pass any type of pointer (array) to the plugin, however this has not yet been tested
    // This means that you should be able to pass an array of structures, i.e. points, and that the array should fill itself
    // And that you don't need to do any parsing in Unity (however, not yet tested)
    private static extern int set_data(byte[] points);
    [DllImport("PCStreamingPlugin")]
    private static extern void clean_up();
    [DllImport("PCStreamingPlugin")]
    private static extern int send_data_to_server(byte[] data, uint size);
    private MeshFilter meshFilter;
    private int num;
    private int numTemp;
    private int currentFrame = 0;
    private bool isDecoding = false;
    private bool frameReady = false;
    private byte[] data;
    private Mesh.MeshDataArray meshDataArray;
    private List<Task<DracoMeshLoader.DecodeResult>> decodeTasks;
    private List<int> sizes;
    private int nLayers;
    private Mesh mesh;
    private long previousAnimationTime;
    private long decodeStartTime;
    private long frameReadyTime;
    private long frameIdleTime;

    private StreamWriter writer;

    public Text animLatency;
    public Text decodeLatency;
    public Text frameReadyLatency;
    public Text temp;

    public GameObject HQ;
    public GameObject MQ;
    public GameObject LQ;

    private MeshFilter hqFilter;
    private MeshFilter mqFilter;
    private MeshFilter lqFilter;

    private int quality = 0;
    // Start is called before the first frame update
    public SessionInfo SessionInfo;
    private Dictionary<int, DecodedPointCloud> inProgessFrames;
    private ConcurrentQueue<DecodedPointCloud> queue;
  //  private List<ConcurrentQueue<DecodeTask>> task_queue;
    private Mutex mut = new Mutex();
    private int lastCompletedFrameNr = -1;
    private Thread wrk;
    void Start()
    {
        queue = new();
        inProgessFrames = new();
        // If this functions returns 0 everything is fine, 1=>WSA startup error, 2=>socket creation error, 3=>sendto (L4S client) error
        Debug.Log(setup_connection(SessionInfo.cltAddr, (uint)SessionInfo.cltPort, (uint)SessionInfo.selfPort));
        start_listening();
        meshFilter =GetComponent<MeshFilter>();
        hqFilter= HQ.GetComponent<MeshFilter>();
        mqFilter = MQ.GetComponent<MeshFilter>();
        lqFilter = LQ.GetComponent<MeshFilter>();
        decodeTasks = new();
        sizes= new();
        previousAnimationTime = DateTimeOffset.Now.ToUnixTimeMilliseconds();
        writer = new StreamWriter("output.csv", false);
        writer.WriteLine("frame_nr;dc_latency;interframe_latency;idle_time;quality;timestamp;size");
        writer.Flush();
        wrk = new Thread(() => listenWork());
        wrk.Start();
    }

    // Update is called once per frame
    void Update()
    {
        if (!queue.IsEmpty)
        {
            DecodedPointCloud dec;
            bool succes = queue.TryDequeue(out dec);
            if (succes)
            {
                Destroy(mesh);
                mesh = new Mesh();
                mesh.indexFormat = dec.NPoints > 65535 ?
                        IndexFormat.UInt32 : IndexFormat.UInt16;
                mesh.SetVertices(dec.Points);
                mesh.SetColors(dec.Colors);
                mesh.SetIndices(
                    Enumerable.Range(0, mesh.vertexCount).ToArray(),
                    MeshTopology.Points, 0
                );
            //    Debug.Log(mesh.vertexCount);
                mesh.UploadMeshData(true);
                HQ.SetActive(false);
                MQ.SetActive(false);
                LQ.SetActive(false);
                if (dec.Quality >= 60)
                {
                    HQ.SetActive(true);
                    hqFilter.mesh = mesh;
                //    Debug.Log("HQ");
                }
                else if (dec.Quality >= 40)
                {
                    MQ.SetActive(true);
                    mqFilter.mesh = mesh;
                //    Debug.Log("MQ");
                }
                else
                {
                    LQ.SetActive(true);
                    lqFilter.mesh = mesh;
                  //  Debug.Log("LQ");
                }

            }
        }
    }
    void OnApplicationQuit()
    { 
        writer.Close();
        clean_up();
    }
    private void listenWork() {
        bool keep_working = true;
        while(keep_working)
        {
            int num = next_frame();
         //   Debug.Log($"f {num}");
            if (num == -1)
            {
               
                keep_working = false;
                continue;
            }
            if(num < 50)
            {
                continue;
            }
            quality = 0;
            currentFrame++;
            data = new byte[num];
            set_data(data);
            nLayers = (int)BitConverter.ToUInt32(data, 0);
           // Debug.Log(nLayers);
          //  meshDataArray = Mesh.AllocateWritableMeshData(nLayers);
            int offset = 0 + 7 * 4;
            inProgessFrames.Add(currentFrame, new DecodedPointCloud(currentFrame, nLayers));
            for (int i = 0; i < nLayers; i++)
            {
                UInt32 layerId = BitConverter.ToUInt32(data, offset);
                switch (layerId)
                {
                    case 0:
                        quality = 60;
                        break;
                    case 1:
                        quality = 25;
                        break;
                    case 2:
                        quality = 15;
                        break;
                }
                offset += 4;
                UInt32 layerSize = BitConverter.ToUInt32(data, offset);
                offset += 4;
                byte[] layerData = new byte[layerSize];
                Array.Copy(data, offset, layerData, 0, layerSize);
                offset += (int)layerSize;
                sizes.Add((int)layerSize);
                //Debug.Log($"{currentFrame} {layerSize}");
                //var d = new Thread(() => decodeWork(new DecodeTask(currentFrame, (int)layerId, nLayers, (int)layerSize, quality, layerData)));
                //d.
                Task.Run(() => decodeWork(new DecodeTask(currentFrame, (int)layerId, nLayers, (int)layerSize, quality, layerData)));
            }
        }
        //Debug.Log("Stopped listening");
    }
    public async Task decodeWork(DecodeTask dc)
    {
       // Debug.Log("start decode");
        IntPtr decoderPtr = IntPtr.Zero;
        unsafe
        {
            fixed (byte* ptr = dc.DescriptionData)
            {
                decoderPtr = DracoInvoker.decode_pc(ptr, (uint)dc.DescriptionSize);
            }
        }
        if (decoderPtr == IntPtr.Zero)
        {
            Debug.Log($"Debug error at client {dc.DescriptionNr}");
            return;
        }
        mut.WaitOne();
        DecodedPointCloud pcData;
        if (!inProgessFrames.TryGetValue(dc.FrameNr, out pcData))
        {
            DracoInvoker.free_decoder(decoderPtr);
            Debug.Log($"Decoder freed, frame does not exit");
            return;
        }
        UInt32 nDecodedPoints = DracoInvoker.get_n_points(decoderPtr);
      
        unsafe
        {  
            IntPtr pointsPtr = DracoInvoker.get_point_array(decoderPtr);
            IntPtr colorPtr = DracoInvoker.get_color_array(decoderPtr);
            float* pointsUnsafePtr = (float*)pointsPtr;
            byte* colorsUnsafePtr = (byte*)colorPtr;

            for (int i = 0; i < nDecodedPoints; i++)
            {
                //    points[i] = new Vector3(0, 0, 0);
                pcData.Points.Add(new Vector3(pointsUnsafePtr[(i * 3)] * -1, pointsUnsafePtr[(i * 3) + 1] * -1, pointsUnsafePtr[(i * 3) + 2] * -1));
                pcData.Colors.Add(new Color32(colorsUnsafePtr[(i * 3)], colorsUnsafePtr[(i * 3) + 1], colorsUnsafePtr[(i * 3) + 2], 255));
            }
        }
        DracoInvoker.free_decoder(decoderPtr);
      //  Debug.Log($"Decoder freed");
        pcData.CurrentNDescriptions++;
        pcData.NPoints += (int)nDecodedPoints;
        pcData.Quality += dc.Quality;
        if (pcData.IsCompleted)
        {
            if (dc.FrameNr % 10 == 0)
            {
               // Debug.Log($"Frame {dc.FrameNr} completed, last compl= {lastCompletedFrameNr}");
            }

            inProgessFrames.Remove(dc.FrameNr);
            if (dc.FrameNr >= lastCompletedFrameNr)
            {
                lastCompletedFrameNr = pcData.FrameNr;
                queue.Enqueue(pcData);
            }

        }
        mut.ReleaseMutex();
    }
}
