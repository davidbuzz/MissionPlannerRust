// udp-vehicle: a vehicle on a tailnet that sends first over UDP, for web/check/tailnet_e2e.sh. A
// companion computer's MAVLink router does the same - the autopilot's stream sent as datagrams
// to the ground station's address, its answers taken from where they come. Here the autopilot is
// a SITL on this machine's TCP port; the node joins the tailnet with Tailscale's tsnet, as the
// page's own node does (../main.go), and relays between the two until it is stopped.
//
// Part of MissionPlannerRust's tests, not of the planner (GPL-3.0-only, as the repository).
//
//	udp-vehicle -control URL -authkey KEY -to ADDRESS:PORT [-sitl 127.0.0.1:5760] [-dir DIR]
package main

import (
	"context"
	"flag"
	"io"
	"log"
	"net"
	"os"
	"time"

	"tailscale.com/tsnet"
)

func main() {
	control := flag.String("control", "", "the tailnet's coordination server")
	authKey := flag.String("authkey", "", "a key to join it with")
	to := flag.String("to", "", "the ground station's tailnet address and UDP port")
	sitl := flag.String("sitl", "127.0.0.1:5760", "the autopilot's TCP port")
	dir := flag.String("dir", "", "the node's state (a temporary folder when empty)")
	flag.Parse()
	if *control == "" || *authKey == "" || *to == "" {
		flag.Usage()
		os.Exit(2)
	}
	state := *dir
	if state == "" {
		var err error
		if state, err = os.MkdirTemp("", "udp-vehicle-"); err != nil {
			log.Fatal(err)
		}
		defer os.RemoveAll(state)
	}
	server := &tsnet.Server{
		Dir:        state,
		Hostname:   "udp-vehicle",
		AuthKey:    *authKey,
		ControlURL: *control,
		Ephemeral:  true,
		Logf:       func(string, ...any) {},
	}
	defer server.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	if _, err := server.Up(ctx); err != nil {
		log.Fatalf("joining the tailnet: %v", err)
	}
	autopilot, err := net.Dial("tcp", *sitl)
	if err != nil {
		log.Fatalf("the autopilot at %s: %v", *sitl, err)
	}
	ground, err := server.Dial(ctx, "udp", *to)
	if err != nil {
		log.Fatalf("udp to %s: %v", *to, err)
	}
	log.Printf("relaying %s <-> udp %s over the tailnet", *sitl, *to)
	// The autopilot's stream out, each read a datagram; the ground station's answers back.
	go func() {
		buf := make([]byte, 1024)
		for {
			n, err := autopilot.Read(buf)
			if n > 0 {
				if _, err := ground.Write(buf[:n]); err != nil {
					log.Fatalf("to %s: %v", *to, err)
				}
			}
			if err != nil {
				log.Fatalf("the autopilot: %v", err)
			}
		}
	}()
	answered := 0
	buf := make([]byte, 65536)
	for {
		n, err := ground.Read(buf)
		if n > 0 {
			if answered == 0 {
				log.Printf("the ground station answered")
			}
			answered += n
			if _, err := autopilot.Write(buf[:n]); err != nil {
				log.Fatalf("to the autopilot: %v", err)
			}
		}
		if err != nil && err != io.EOF {
			log.Fatalf("from %s: %v", *to, err)
		}
	}
}
