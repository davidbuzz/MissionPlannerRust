// Copyright (c) Tailscale Inc & contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Adapted for MissionPlannerRust's browser build from tailscale.com's cmd/tsconnect/wasm/wasm_js.go
// (v1.104.0): the same node - Tailscale's own control client, WireGuard engine and netstack, its
// traffic over DERP relays on WebSockets - with `ssh` and `fetch` replaced by `dial`, which hands
// the page a TCP or UDP stream to a tailnet address for the planner's links
// (experiments/web-experiment/www/link.js), and with logs kept to the console rather than uploaded.
//
// When run in the browser, newIPN(config) is added to the global namespace. It returns an object
// with run(callbacks), login(), logout() and dial(network, address, callbacks).

package main

import (
	"context"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log"
	"math/rand/v2"
	"net"
	"net/netip"
	"strings"
	"sync"
	"syscall/js"
	"time"

	"tailscale.com/control/controlclient"
	"tailscale.com/ipn"
	"tailscale.com/ipn/ipnauth"
	"tailscale.com/ipn/ipnlocal"
	"tailscale.com/ipn/ipnserver"
	"tailscale.com/ipn/store/mem"
	"tailscale.com/logpolicy"
	"tailscale.com/logtail"
	"tailscale.com/net/netns"
	"tailscale.com/net/tsdial"
	"tailscale.com/safesocket"
	"tailscale.com/tailcfg"
	"tailscale.com/tsd"
	"tailscale.com/types/views"
	"tailscale.com/wgengine"
	"tailscale.com/wgengine/netstack"
	"tailscale.com/words"
)

func main() {
	js.Global().Set("newIPN", js.FuncOf(func(this js.Value, args []js.Value) any {
		if len(args) != 1 {
			log.Fatal("Usage: newIPN(config)")
			return nil
		}
		return newIPN(args[0])
	}))
	// Keep the Go runtime alive for newIPN's callers.
	<-make(chan bool)
}

func newIPN(jsConfig js.Value) map[string]any {
	netns.SetEnabled(false)

	var store ipn.StateStore
	if jsStateStorage := jsConfig.Get("stateStorage"); !jsStateStorage.IsUndefined() {
		store = &jsStateStore{jsStateStorage}
	} else {
		store = new(mem.Store)
	}

	controlURL := ipn.DefaultControlURL
	if jsControlURL := jsConfig.Get("controlURL"); jsControlURL.Type() == js.TypeString {
		controlURL = jsControlURL.String()
	}
	var authKey string
	if jsAuthKey := jsConfig.Get("authKey"); jsAuthKey.Type() == js.TypeString {
		authKey = jsAuthKey.String()
	}
	var hostname string
	if jsHostname := jsConfig.Get("hostname"); jsHostname.Type() == js.TypeString {
		hostname = jsHostname.String()
	} else {
		hostname = generateHostname()
	}

	// The node's log id, as tsconnect keeps it; the logs themselves stay in the console.
	lpc := getOrCreateLogPolicyConfig(store)
	logf := log.Printf

	sys := tsd.NewSystem()
	sys.Set(store)
	dialer := &tsdial.Dialer{Logf: logf}
	dialer.SetBus(sys.Bus.Get())
	eng, err := wgengine.NewUserspaceEngine(logf, wgengine.Config{
		Dialer:        dialer,
		SetSubsystem:  sys.Set,
		ControlKnobs:  sys.ControlKnobs(),
		HealthTracker: sys.HealthTracker.Get(),
		ExtraRootCAs:  sys.ExtraRootCAs,
		Metrics:       sys.UserMetricsRegistry(),
		EventBus:      sys.Bus.Get(),
	})
	if err != nil {
		log.Fatal(err)
	}
	sys.Set(eng)

	ns, err := netstack.Create(logf, sys.Tun.Get(), eng, sys.MagicSock.Get(), dialer, sys.DNSManager.Get(), sys.ProxyMapper())
	if err != nil {
		log.Fatalf("netstack.Create: %v", err)
	}
	sys.Set(ns)
	ns.ProcessLocalIPs = true
	ns.ProcessSubnets = true

	dialer.UseNetstackForIP = func(ip netip.Addr) bool {
		return true
	}
	dialer.NetstackDialTCP = func(ctx context.Context, dst netip.AddrPort) (net.Conn, error) {
		tcpConn, err := ns.DialContextTCP(ctx, dst)
		if err != nil {
			return nil, err
		}
		return tcpConn, nil
	}
	dialer.NetstackDialUDP = func(ctx context.Context, dst netip.AddrPort) (net.Conn, error) {
		udpConn, err := ns.DialContextUDP(ctx, dst)
		if err != nil {
			return nil, err
		}
		return udpConn, nil
	}
	sys.NetstackRouter.Set(true)
	sys.Tun.Get().Start()

	logid := lpc.PublicID
	srv := ipnserver.New(logf, logid, sys.Bus.Get(), sys.NetMon.Get())
	lb, err := ipnlocal.NewLocalBackend(logf, logid, sys, controlclient.LoginEphemeral)
	if err != nil {
		log.Fatalf("ipnlocal.NewLocalBackend: %v", err)
	}
	if err := ns.Start(lb); err != nil {
		log.Fatalf("failed to start netstack: %v", err)
	}
	srv.SetLocalBackend(lb)

	jsIPN := &jsIPN{
		dialer:     dialer,
		srv:        srv,
		lb:         lb,
		controlURL: controlURL,
		authKey:    authKey,
		hostname:   hostname,
	}

	return map[string]any{
		"run": js.FuncOf(func(this js.Value, args []js.Value) any {
			if len(args) != 1 {
				log.Print("Usage: run({notifyState, notifyNetMap, notifyBrowseToURL, notifyPanicRecover})")
				return nil
			}
			jsIPN.run(args[0])
			return nil
		}),
		"login": js.FuncOf(func(this js.Value, args []js.Value) any {
			jsIPN.login()
			return nil
		}),
		"logout": js.FuncOf(func(this js.Value, args []js.Value) any {
			jsIPN.logout()
			return nil
		}),
		"dial": js.FuncOf(func(this js.Value, args []js.Value) any {
			if len(args) != 3 {
				log.Print("Usage: dial(network, address, {onOpen, onData, onClose})")
				return nil
			}
			return jsIPN.dial(args[0].String(), args[1].String(), args[2])
		}),
	}
}

type jsIPN struct {
	dialer     *tsdial.Dialer
	srv        *ipnserver.Server
	lb         *ipnlocal.LocalBackend
	controlURL string
	authKey    string
	hostname   string
}

var jsIPNState = map[ipn.State]string{
	ipn.NoState:          "NoState",
	ipn.InUseOtherUser:   "InUseOtherUser",
	ipn.NeedsLogin:       "NeedsLogin",
	ipn.NeedsMachineAuth: "NeedsMachineAuth",
	ipn.Stopped:          "Stopped",
	ipn.Starting:         "Starting",
	ipn.Running:          "Running",
}

var jsMachineStatus = map[tailcfg.MachineStatus]string{
	tailcfg.MachineUnknown:      "MachineUnknown",
	tailcfg.MachineUnauthorized: "MachineUnauthorized",
	tailcfg.MachineAuthorized:   "MachineAuthorized",
	tailcfg.MachineInvalid:      "MachineInvalid",
}

func (i *jsIPN) run(jsCallbacks js.Value) {
	notifyState := func(state ipn.State) {
		jsCallbacks.Call("notifyState", jsIPNState[state])
	}
	notifyState(ipn.NoState)

	i.lb.SetNotifyCallback(func(n ipn.Notify) {
		defer func() {
			if r := recover(); r != nil {
				fmt.Println("Panic recovered:", r)
				jsCallbacks.Call("notifyPanicRecover", fmt.Sprint(r))
			}
		}()
		if n.State != nil {
			notifyState(*n.State)
		}
		if n.SelfChange != nil {
			if nm := i.lb.NetMapWithPeers(); nm != nil {
				jsNetMap := jsNetMap{
					Self: jsNetMapSelfNode{
						jsNetMapNode: jsNetMapNode{
							Name:       nm.SelfName(),
							Addresses:  mapSliceView(nm.GetAddresses(), func(a netip.Prefix) string { return a.Addr().String() }),
							NodeKey:    nm.NodeKey.String(),
							MachineKey: nm.MachineKey.String(),
						},
						MachineStatus: jsMachineStatus[nm.GetMachineStatus()],
					},
					Peers: mapSlice(nm.Peers, func(p tailcfg.NodeView) jsNetMapPeerNode {
						name := p.Name()
						if name == "" {
							name = p.Hostinfo().Hostname()
						}
						addrs := make([]string, p.Addresses().Len())
						for i, ap := range p.Addresses().All() {
							addrs[i] = ap.Addr().String()
						}
						return jsNetMapPeerNode{
							jsNetMapNode: jsNetMapNode{
								Name:       name,
								Addresses:  addrs,
								MachineKey: p.Machine().String(),
								NodeKey:    p.Key().String(),
							},
							Online: p.Online().Clone(),
						}
					}),
					LockedOut: nm.TKAEnabled && nm.SelfNode.KeySignature().Len() == 0,
				}
				if jsonNetMap, err := json.Marshal(jsNetMap); err == nil {
					jsCallbacks.Call("notifyNetMap", string(jsonNetMap))
				} else {
					log.Printf("Could not generate JSON netmap: %v", err)
				}
			}
		}
		if n.BrowseToURL != nil {
			jsCallbacks.Call("notifyBrowseToURL", *n.BrowseToURL)
		}
	})

	go func() {
		err := i.lb.Start(ipn.Options{
			UpdatePrefs: &ipn.Prefs{
				ControlURL:  i.controlURL,
				RouteAll:    false,
				WantRunning: true,
				Hostname:    i.hostname,
			},
			AuthKey: i.authKey,
		})
		if err != nil {
			log.Printf("Start error: %v", err)
		}
	}()

	go func() {
		ln, err := safesocket.Listen("")
		if err != nil {
			log.Fatalf("safesocket.Listen: %v", err)
		}
		err = i.srv.Run(context.Background(), ln)
		log.Fatalf("ipnserver.Run exited: %v", err)
	}()
}

func (i *jsIPN) login() {
	go i.lb.StartLoginInteractive(context.Background())
}

func (i *jsIPN) logout() {
	go func() {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		i.lb.Logout(ctx, ipnauth.Self)
	}()
}

// dial opens network ("tcp" or "udp") to address ("host:port", a tailnet address or MagicDNS
// name) through the tailnet, and hands the page the stream: callbacks.onOpen(), onData(Uint8Array)
// for each read, onClose(reason) once. The object returned has write(Uint8Array), which queues
// until the stream is open, and close().
func (i *jsIPN) dial(network, address string, callbacks js.Value) map[string]any {
	stream := &jsStream{writes: make(chan []byte, 256), closed: make(chan struct{})}
	go stream.run(i.dialer, network, address, callbacks)
	return map[string]any{
		"write": js.FuncOf(func(this js.Value, args []js.Value) any {
			if len(args) != 1 {
				return false
			}
			data := make([]byte, args[0].Get("length").Int())
			js.CopyBytesToGo(data, args[0])
			select {
			case stream.writes <- data:
				return true
			case <-stream.closed:
				return false
			}
		}),
		"close": js.FuncOf(func(this js.Value, args []js.Value) any {
			stream.close()
			return nil
		}),
	}
}

type jsStream struct {
	writes    chan []byte
	closed    chan struct{}
	closeOnce sync.Once
	mu        sync.Mutex
	conn      net.Conn
}

func (s *jsStream) close() {
	s.closeOnce.Do(func() {
		close(s.closed)
		s.mu.Lock()
		if s.conn != nil {
			s.conn.Close()
		}
		s.mu.Unlock()
	})
}

func (s *jsStream) run(dialer *tsdial.Dialer, network, address string, callbacks js.Value) {
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	conn, err := dialer.UserDial(ctx, network, address)
	cancel()
	if err != nil {
		callbacks.Call("onClose", fmt.Sprintf("dial %s %s: %v", network, address, err))
		s.close()
		return
	}
	s.mu.Lock()
	s.conn = conn
	s.mu.Unlock()
	select {
	case <-s.closed:
		conn.Close()
		return
	default:
	}
	callbacks.Call("onOpen")
	go func() {
		for {
			select {
			case data := <-s.writes:
				if _, err := conn.Write(data); err != nil {
					s.close()
					return
				}
			case <-s.closed:
				return
			}
		}
	}()
	buf := make([]byte, 16384)
	for {
		n, err := conn.Read(buf)
		if n > 0 {
			array := js.Global().Get("Uint8Array").New(n)
			js.CopyBytesToJS(array, buf[:n])
			callbacks.Call("onData", array)
		}
		if err != nil {
			callbacks.Call("onClose", err.Error())
			s.close()
			return
		}
	}
}

type jsNetMap struct {
	Self      jsNetMapSelfNode   `json:"self"`
	Peers     []jsNetMapPeerNode `json:"peers"`
	LockedOut bool               `json:"lockedOut"`
}

type jsNetMapNode struct {
	Name       string   `json:"name"`
	Addresses  []string `json:"addresses"`
	MachineKey string   `json:"machineKey"`
	NodeKey    string   `json:"nodeKey"`
}

type jsNetMapSelfNode struct {
	jsNetMapNode
	MachineStatus string `json:"machineStatus"`
}

type jsNetMapPeerNode struct {
	jsNetMapNode
	Online *bool `json:"online,omitempty"`
}

type jsStateStore struct {
	jsStateStorage js.Value
}

func (s *jsStateStore) ReadState(id ipn.StateKey) ([]byte, error) {
	jsValue := s.jsStateStorage.Call("getState", string(id))
	if jsValue.String() == "" {
		return nil, ipn.ErrStateNotExist
	}
	return hex.DecodeString(jsValue.String())
}

func (s *jsStateStore) WriteState(id ipn.StateKey, bs []byte) error {
	s.jsStateStorage.Call("setState", string(id), hex.EncodeToString(bs))
	return nil
}

func mapSlice[T any, M any](a []T, f func(T) M) []M {
	n := make([]M, len(a))
	for i, e := range a {
		n[i] = f(e)
	}
	return n
}

func mapSliceView[T any, M any](a views.Slice[T], f func(T) M) []M {
	n := make([]M, a.Len())
	for i, v := range a.All() {
		n[i] = f(v)
	}
	return n
}

func filterSlice[T any](a []T, f func(T) bool) []T {
	n := make([]T, 0, len(a))
	for _, e := range a {
		if f(e) {
			n = append(n, e)
		}
	}
	return n
}

func generateHostname() string {
	tails := words.Tails()
	scales := words.Scales()
	if rand.IntN(2) == 0 {
		tails = filterSlice(tails, func(s string) bool { return strings.HasPrefix(s, "j") })
		scales = filterSlice(scales, func(s string) bool { return strings.HasPrefix(s, "s") })
	} else {
		tails = filterSlice(tails, func(s string) bool { return strings.HasPrefix(s, "w") })
		scales = filterSlice(scales, func(s string) bool { return strings.HasPrefix(s, "a") })
	}
	return fmt.Sprintf("mpr-%s-%s", tails[rand.IntN(len(tails))], scales[rand.IntN(len(scales))])
}

const logPolicyStateKey = "log-policy"

func getOrCreateLogPolicyConfig(state ipn.StateStore) *logpolicy.Config {
	if configBytes, err := state.ReadState(logPolicyStateKey); err == nil {
		if config, err := logpolicy.ConfigFromBytes(configBytes); err == nil {
			return config
		} else {
			log.Printf("Could not parse log policy config: %v", err)
		}
	} else if err != ipn.ErrStateNotExist {
		log.Printf("Could not get log policy config from state store: %v", err)
	}
	config := logpolicy.NewConfig(logtail.CollectionNode)
	if err := state.WriteState(logPolicyStateKey, config.ToBytes()); err != nil {
		log.Printf("Could not save log policy config to state store: %v", err)
	}
	return config
}
