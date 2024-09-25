package main

import "github.com/eapache/queue"

// https://github.com/eapache/channels/blob/master/ring_channel.go
// RingChannel implements the Channel interface in a way that never blocks the writer.
// Specifically, if a value is written to a RingChannel when its buffer is full then the oldest
// value in the buffer is discarded to make room (just like a standard ring-buffer).
// Note that Go's scheduler can cause discarded values when they could be avoided, simply by scheduling
// the writer before the reader, so caveat emptor.
// For the opposite behaviour (discarding the newest element, not the oldest) see OverflowingChannel.
type RingChannel struct {
	input, output chan interface{}
	length        chan int
	buffer        *queue.Queue
	size          uint32
}

func NewRingChannel(size uint32) *RingChannel {
	if size < 0 {
		panic("channels: invalid negative size in NewRingChannel")
	}
	ch := &RingChannel{
		input:  make(chan interface{}),
		output: make(chan interface{}),
		buffer: queue.New(),
		size:   size,
	}
	ch.length = make(chan int)
	go ch.ringBuffer()

	return ch
}

func (ch *RingChannel) In() chan<- interface{} {
	return ch.input
}

func (ch *RingChannel) Out() <-chan interface{} {
	return ch.output
}

func (ch *RingChannel) Len() int {
	return <-ch.length
}

func (ch *RingChannel) Cap() uint32 {
	return ch.size
}

func (ch *RingChannel) Close() {
	close(ch.input)
}

// for all buffered cases
func (ch *RingChannel) ringBuffer() {
	var input, output chan interface{}
	var next interface{}
	input = ch.input

	for input != nil || output != nil {
		select {
		// Prefer to write if possible, which is surprisingly effective in reducing
		// dropped elements due to overflow. The naive read/write select chooses randomly
		// when both channels are ready, which produces unnecessary drops 50% of the time.
		case output <- next:
			ch.buffer.Remove()
		default:
			select {
			case elem, open := <-input:
				if open {
					ch.buffer.Add(elem)
					if ch.buffer.Length() > int(ch.size) {
						ch.buffer.Remove()
					}
				} else {
					input = nil
				}
			case output <- next:
				ch.buffer.Remove()
			case ch.length <- ch.buffer.Length():
			}
		}

		if ch.buffer.Length() > 0 {
			output = ch.output
			next = ch.buffer.Peek()
		} else {
			output = nil
			next = nil
		}
	}

	close(ch.output)
	close(ch.length)
}
