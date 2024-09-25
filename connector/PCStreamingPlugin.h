#ifndef LS4PROXYDLL_NATIVE_LIB_H
#define LS4PROXYDLL_NATIVE_LIB_H

#define DLLExport __declspec(dllexport)

// All exported functions should be declared here
extern "C"
{
    DLLExport void set_data(void* d);
    DLLExport int setup_connection(char* server_str, uint32_t server_port, uint32_t self_port);
    DLLExport void start_listening();
    DLLExport int next_frame();
    DLLExport void clean_up();
    DLLExport int send_data_to_server(void* data, uint32_t size);
}
#endif