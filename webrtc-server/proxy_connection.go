package main

import (
	"bytes"
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
	ClientID    uint32
	Framenr     uint32
	Framelen    uint32
	Frameoffset uint32
	Packetlen   uint32
}

type RemoteFrame struct {
	frameNr    uint32
	currentLen uint32
	frameLen   uint32
	frameData  []byte
}

type ProxyConnection struct {
	// General
	addr *net.UDPAddr
	conn *net.UDPConn

	// Receiving
	incomplete_frames map[uint32]RemoteFrame
	complete_frames   []RemoteFrame
	frameCounter      uint32
	indi_mode         bool

	mtx_pccon  sync.Mutex
	cond_video map[uint32]*sync.Cond
}

func NewProxyConnection(indi_mode bool) *ProxyConnection {
	pc := &ProxyConnection{nil, nil, make(map[uint32]RemoteFrame), make([]RemoteFrame, 0), 0, indi_mode, sync.Mutex{}, make(map[uint32]*sync.Cond)}
	//pc.cond_video = sync.NewCond(&pc.mtx_video)
	if !indi_mode {
		pc.cond_video[0] = sync.NewCond(&pc.mtx_pccon)
	}
	return pc
}

func (pc *ProxyConnection) sendPacket(b []byte, offset uint32, packet_type uint32) {
	buffProxy := make([]byte, 1500)
	binary.LittleEndian.PutUint32(buffProxy[0:], packet_type)
	copy(buffProxy[4:], b[offset:])
	_, err := pc.conn.WriteToUDP(buffProxy, pc.addr)
	if err != nil {
		fmt.Println("Error sending response:", err)
		panic(err)
	}
}

func (pc *ProxyConnection) SetupConnection(capAddr string, srvAddr string) {
	address, err := net.ResolveUDPAddr("udp", srvAddr)
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

	pc.addr, err = net.ResolveUDPAddr("udp", capAddr)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}

	pc.SendPeerReadyPacket()
	buffer := make([]byte, 1500)

	// Wait for incoming messages
	fmt.Println("WebRTCPeer: Waiting for a message...", srvAddr, pc.addr.IP.String())
	_, pc.addr, err = pc.conn.ReadFromUDP(buffer)
	if err != nil {
		fmt.Printf("WebRTCPeer: ERROR: %s\n", err)
		return
	}
	fmt.Println("WebRTCPeer: Connected to Unity DLL")
	pc.StartListening()
}

func (pc *ProxyConnection) StartListening() {
	println("listen")
	go func() {
		for {
			buffer := make([]byte, 1500)
			_, _, _ = pc.conn.ReadFromUDP(buffer)
			var packetType uint32
			err := binary.Read(bytes.NewReader(buffer[:4]), binary.LittleEndian, &packetType)
			bufBinary := bytes.NewBuffer(buffer[4:24])
			// Read the fields from the buffer into a struct
			if packetType == FramePacketType {
				var p RemoteInputPacketHeader
				err = binary.Read(bufBinary, binary.LittleEndian, &p)
				if err != nil {
					fmt.Println("Error Proxy:", err)
					return
				}
				pc.mtx_pccon.Lock()
				_, exists := pc.incomplete_frames[p.Framenr]
				if !exists {
					r := RemoteFrame{
						p.Framenr,
						0,
						p.Framelen,
						make([]byte, p.Framelen),
					}
					pc.incomplete_frames[p.Framenr] = r
				}
				value := pc.incomplete_frames[p.Framenr]

				copy(value.frameData[p.Frameoffset:p.Frameoffset+p.Packetlen], buffer[24:24+p.Packetlen])
				value.currentLen = value.currentLen + p.Packetlen
				pc.incomplete_frames[p.Framenr] = value

				if value.currentLen == value.frameLen {
					if p.Framenr%100 == 0 {
						println("REMOTE FRAME ", p.Framenr, " COMPLETE")
					}
					pc.complete_frames = append(pc.complete_frames, value)
					delete(pc.incomplete_frames, p.Framenr)
					pc.cond_video[p.ClientID].Broadcast()
				}
				//println(p.Frameoffset, p.Framenr, value.currentLen, p.Framelen)
				pc.mtx_pccon.Unlock()
			} else if packetType == ControlPacketType {
				pc.SendBitrates()
			}
		}
	}()
}
func (pc *ProxyConnection) SendFramePacket(b []byte, offset uint32) {
	pc.sendPacket(b, offset, FramePacketType)
}

func (pc *ProxyConnection) SendPeerReadyPacket() {
	pc.sendPacket(make([]byte, 100), 0, ReadyPacketType)
}

func (pc *ProxyConnection) SendBitrates() bool {
	pcMapMutex.Lock()
	defer pcMapMutex.Unlock()
	nClients := uint64(len(peerConnections))
	for _, peerConn := range peerConnections {
		if !peerConn.isReady {
			nClients--
		}
	}

	buffer := new(bytes.Buffer)
	err := binary.Write(buffer, binary.LittleEndian, uint32(nClients))
	if err != nil {
		return false
	}
	for key, peerConn := range peerConnections {
		// Write the key to the buffer
		if !peerConn.isReady {
			continue
		}
		err := binary.Write(buffer, binary.LittleEndian, uint32(key))
		if err != nil {
			return false
		}

		// Write the PeerConnection.GetBitrate() to the buffer
		err = binary.Write(buffer, binary.LittleEndian, uint32(peerConn.GetBitrate()))
		if err != nil {
			return false
		}

	}
	pc.sendPacket(buffer.Bytes(), 0, ControlPacketType)
	return true
}

func (pc *ProxyConnection) NextFrame(clientID uint32) (uint32, []byte) {
	isNextFrameReady := false

	for !isNextFrameReady {
		pc.mtx_pccon.Lock()
		if len(pc.complete_frames) > 0 {
			isNextFrameReady = true
		} else {
			pc.cond_video[clientID].Wait()
			isNextFrameReady = true
		}
	}
	data := pc.complete_frames[0].frameData
	frameNr := pc.complete_frames[0].frameNr
	if pc.frameCounter%100 == 0 {
		println("SENDING FRAME ", pc.frameCounter)
	}
	pc.complete_frames = pc.complete_frames[1:]
	pc.frameCounter = pc.frameCounter + 1
	pc.mtx_pccon.Unlock()
	return frameNr, data
}

func (pc *ProxyConnection) OnNewClientConnected(clientID uint32) {
	if !pc.indi_mode {
		return
	}
	pc.mtx_pccon.Lock()
	defer pc.mtx_pccon.Unlock()
	pc.cond_video[clientID] = sync.NewCond(&pc.mtx_pccon)
}
func (pc *ProxyConnection) OnNewClientDisconnected(clientID uint32) {
	if !pc.indi_mode {
		return
	}
	pc.mtx_pccon.Lock()
	defer pc.mtx_pccon.Unlock()
	delete(pc.cond_video, clientID)
}
