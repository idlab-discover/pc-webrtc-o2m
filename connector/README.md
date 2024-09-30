# PC Streaming Unity Plugin
This repository contains the Unity plugin that can be used to communicate with the WebRTC client. You can build the plugin with Visual Studio (make sure you build the release version and not the debug one) and copy the .dll file to your Unity application.

## Using it in Unity
First you will need to place the built .dll file in a place where Unity can find it. Ideally this should be a Plugin folder in the Assets folder. 

### DLL import
Whenever you have a class that needs to use the functionalities of the plugin, you will need to define the functions you want to use as follows:

    DLLExport void set_data(void* d);
    DLLExport int setup_connection(char* server_str, uint32_t server_port, uint32_t self_port);
    DLLExport void start_listening();
    DLLExport int next_frame();
    DLLExport void clean_up();
    DLLExport int send_data_to_server(void* data, uint32_t size);

```csharp
  // Setup the connection with the WebRTC client application, has to be called before starting the client application
  [DllImport("PCStreamingPlugin")]
  private static extern int setup_connection(string server_str, uint32_t server_port, uint32_t self_port);

  // Start listening for incoming packets, has to be called before starting the client application
  [DllImport("PCStreamingPlugin")]
  private static extern void start_listening();

  // Get the size of the next frame, will block until a frame is available or until the application ends (will return 0 when that happens)  
  [DllImport("PCStreamingPlugin")]
  private static extern int next_frame();
  // Copies the data (position and color) from the current frame into the provided buffer
  [DllImport("PCStreamingPlugin")]
  private static extern void set_data(byte[] points);

  // Send control data such as position and fov to the server
  [DllImport("PCStreamingPlugin")]
  private static extern int send_data_to_server(void* data, uint32_t size);

  // Clean up WSA / sockets
  [DllImport("PCStreamingPlugin")]
  private static extern void clean_up();
```

First you should call the setup_connection(string ip) and start_listening() whenever you want to begin listening to WebRTC client, this could be in the Start() function or whenever a condition is fullfilled. When closing the application it is best to clean up any resources and make sure sockets are closed, this can be done by calling the clean_up() function. Unity has a function that is called whenever the application is closed which you can use as follows:

```csharp
void OnApplicationQuit()
{
  clean_up();
}
```

## Editing the Plugin
You should know that once a plugin is loaded by Unity it is never unloaded unless the editor (or application) is closed. So if you want to make changes to the plugin you will need to close Unity for them to take effect. There is also an advanced method which allows you to reload plugins, if you want to do this use the Native.cs file from the Unity test application and work as follows:

```csharp
delegate int setup_connection(string server_str);
delegate void start_listening();
delegate int next_frame();
delegate void clean_up();
delegate int set_data(byte[] data);
static IntPtr nativeLibraryPtr;
private MeshFilter meshFilter;
void Awake()
{
    if (nativeLibraryPtr != IntPtr.Zero) return;
    nativeLibraryPtr = Native.LoadLibrary("PCStreamingPlugin");
    if (nativeLibraryPtr == IntPtr.Zero)
    {
        Debug.LogError("Failed to load native library");
    }
}
// Start is called before the first frame update
void Start()
{
    Debug.Log(Native.Invoke<int, setup_connection>(nativeLibraryPtr, "172.22.107.250"));
    Native.Invoke<start_listening>(nativeLibraryPtr);
}

// Update is called once per frame
void Update()
{

    int num = Native.Invoke<int, next_frame>(nativeLibraryPtr);
    if (num > 0)
    {
        Native.Invoke<set_data>(nativeLibraryPtr, data);
        // ... Processing code
    }
}
void OnApplicationQuit()
{
    Native.Invoke<clean_up>(nativeLibraryPtr);
    if (nativeLibraryPtr == IntPtr.Zero) return;

    Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                  ? "Native library successfully unloaded."
                  : "Native library could not be unloaded.");
}
```

If you do it this way make sure you place the .dll in the root of your project and not in the Plugin folder.
