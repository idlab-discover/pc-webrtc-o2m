package main

import (
	"encoding/binary"
	"fmt"
	"net"
	"sync"
)

const (
	ReadyPacketType   uint32 = 0
	FramePacketType   uint32 = 1
	AudioPacketType   uint32 = 2
	ControlPacketType uint32 = 3
)

type RemoteInputPacketHeader struct {
	Framenr     uint32
	Framelen    uint32
	Frameoffset uint32
	Packetlen   uint32
}

type RemoteFrame struct {
	currentLen uint32
	frameLen   uint32
	frameData  []byte
}

type ProxyConnection struct {
	// General
	addr *net.UDPAddr
	conn *net.UDPConn

	// Receiving
	m                 sync.RWMutex
	incomplete_frames map[uint32]RemoteFrame
	complete_frames   []RemoteFrame
	frameCounter      uint32

	wsHandler *WebsocketHandler
}

func NewProxyConnection() *ProxyConnection {
	return &ProxyConnection{nil, nil, sync.RWMutex{}, make(map[uint32]RemoteFrame), make([]RemoteFrame, 0), 0, nil}
}

func (pc *ProxyConnection) sendPacket(b []byte, offset uint32, packet_type uint32) {
	buffProxy := make([]byte, 1300)
	binary.LittleEndian.PutUint32(buffProxy[0:], packet_type)
	copy(buffProxy[4:], b[offset:])
	_, err := pc.conn.WriteToUDP(buffProxy, pc.addr)
	if err != nil {
		fmt.Println("Error sending response:", err)
		panic(err)
	}
}

func (pc *ProxyConnection) SetupConnection(proxyAddr string, cltAddr string) {
	address, err := net.ResolveUDPAddr("udp", cltAddr)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}

	// Create a UDP connection
	pc.conn, err = net.ListenUDP("udp", address)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}

	pc.addr, err = net.ResolveUDPAddr("udp", proxyAddr)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}

	pc.SendPeerReadyPacket()
	buffer := make([]byte, 1500)

	// Wait for incoming messages
	fmt.Println("WebRTCPeer: Waiting for a message...", cltAddr, pc.addr.IP.String())
	_, pc.addr, err = pc.conn.ReadFromUDP(buffer)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}
	fmt.Println("WebRTCPeer: Connected to Unity DLL")
	pc.StartListening()
}

func (pc *ProxyConnection) StartListening() {
	go func() {
		for {
			buffer := make([]byte, 1500)
			_, _, _ = pc.conn.ReadFromUDP(buffer)
			if pc.wsHandler != nil {
				pc.wsHandler.SendMessage(WebsocketPacket{
					1,
					7,
					string(buffer[4:]),
				})
			}

		}
	}()
}
func (pc *ProxyConnection) SendFramePacket(b []byte, offset uint32) {
	pc.sendPacket(b, offset, FramePacketType)
}
func (pc *ProxyConnection) SendControlPacket(b []byte) {
	pc.sendPacket(b, 0, ControlPacketType)
}

func (pc *ProxyConnection) SendPeerReadyPacket() {
	pc.sendPacket(make([]byte, 100), 0, ReadyPacketType)
}

func (pc *ProxyConnection) SetWsHandler(wsHandler *WebsocketHandler) {
	pc.wsHandler = wsHandler
}
