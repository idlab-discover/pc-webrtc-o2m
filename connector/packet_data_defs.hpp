#include<stdio.h>
#include <iostream>

#define BEGIN_DECL_DATA(name)                                                   \
    struct name {																\
        static constexpr auto size() { return sizeof(struct name); }            \
        name(char **buf, size_t &avail) {									\
			std::memcpy(data(), *buf, size()); 									\
			*buf += size();                                                     \
            avail -= size();  													\
        }																		\
		char* data() { return reinterpret_cast<char*>(this); }					\

#define END_DECL_DATA(name)                                                     \
    };  
BEGIN_DECL_DATA(PacketType)
    uint32_t type;
END_DECL_DATA(PacketType)
BEGIN_DECL_DATA(PacketHeader)
    uint32_t framenr;
    uint32_t framelen;
    uint32_t frameoffset;
    uint32_t packetlen;
END_DECL_DATA(PacketHeader)