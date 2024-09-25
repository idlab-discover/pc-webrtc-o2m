package main

import (
	"log"
	"net/http"

	"github.com/gorilla/websocket"
)

type WebsocketPacket struct {
	ClientID    uint64
	MessageType uint64
	Message     string
}

type WebsocketCallback func(WebsocketPacket, *PeerConnection)
type NewUserCallback func(*websocket.Conn)

type WebsocketHandler struct {
	newUserCb NewUserCallback
	upgrader  *websocket.Upgrader
}

// TODO USE websockets for control data
func NewWSServer(addr string, newUserCb NewUserCallback) *WebsocketHandler {
	upgrader := &websocket.Upgrader{}
	wsServer := &WebsocketHandler{newUserCb, upgrader}
	http.HandleFunc("/", wsServer.getNewClientCbFunc)
	go http.ListenAndServe(addr, nil)
	return wsServer
}

func (ws *WebsocketHandler) getNewClientCbFunc(w http.ResponseWriter, r *http.Request) {
	c, err := ws.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Print("upgrade:", err)
		return
	}
	ws.newUserCb(c)
	// Add peer connection
	// TODO Change to peer closing it

}
