# PC Streaming Unity Plugin
This repository contains the Unity plugin that can be used to communicate with the WebRTC client. You can build the plugin with Visual Studio (make sure you build the release version and not the debug one) and copy the .dll file to your Unity application.

## Using it in Unity
First you will need to place the built .dll file in a place where Unity can find it. Ideally this should be a Plugin folder in the Assets folder. 

Now whenever you have a class that needs to use the plugin you will need to define the functions of the plugin that you want to use like follows:

```csharp
  [DllImport("PCStreamingPlugin")]
  private static extern int setup_connection(string ip, uint32_t plugin);
  [DllImport("PCStreamingPlugin")]
  private static extern void start_listening();
  [DllImport("PCStreamingPlugin")]
  private static extern int next_frame();
  [DllImport("PCStreamingPlugin")]
  // You should technically be able to pass any type of pointer (array) to the plugin, however this has not yet been tested
  // This means that you should be able to pass an array of structures, i.e. points, and that the array should fill itself
  // And that you don't need to do any parsing in Unity (however, not yet tested)
  private static extern void set_data(byte[] points);
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

You yourself are responsible for parsing the raw data returned from the plugin. You can pass any pointer to the set_data() function as this function just simply copies data from an internal buffer to your (array) pointer.

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
