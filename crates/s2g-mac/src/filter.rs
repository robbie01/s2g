//! Stateless "good neighbour" frame filter.
//!
//! A radio link shared with other amateurs should carry only what the
//! operators mean to send, and an OS attached to a TAP interface emits a
//! surprising amount it does not: IPv4 and ARP on a link meant to be
//! IPv6-only, router discovery and DHCPv6 when addressing is static and
//! routing comes from Babel, and the whole zoo of LAN discovery protocols
//! (mDNS, LLMNR, SSDP, WS-Discovery). Part 97 also forbids obscuring the
//! meaning of a transmission, which rules out SSH and TLS.
//!
//! The filter is a pure function of one Ethernet frame — no connection
//! tracking, no state — and is applied to frames leaving for the air and,
//! by default, to frames arriving from it. Everything not explicitly
//! blocked passes: neighbour discovery (NS/NA, needed to resolve link-local
//! addresses on the link), ICMPv6 echo and errors, Babel (UDP 6696 on
//! ff02::1:6), DNS, NTP, HTTP, OSPFv3, anything unlisted.
//!
//! Defaults, and the reasoning behind them, are in the README section
//! "Good-neighbour filter".

/// What to do with a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Pass,
    /// Dropped, with a short static reason for logging and counting.
    Drop(&'static str),
}

/// Filter policy. Every field is a switch or a list; construct with
/// [`FilterConfig::good_neighbor`] (the defaults below) or
/// [`FilterConfig::off`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FilterConfig {
    /// Apply the filter to frames leaving for the air.
    pub egress: bool,
    /// Apply the filter to frames arriving from the air before they reach
    /// the host.
    pub ingress: bool,
    /// Allow IPv4 and ARP (default: IPv6 only).
    pub allow_ipv4: bool,
    /// Allow ICMPv6 Router Solicitation / Advertisement / Redirect.
    pub allow_router_discovery: bool,
    /// Allow DHCPv6 (UDP 546/547).
    pub allow_dhcpv6: bool,
    /// Allow MLD (ICMPv6 130–132, 143).
    pub allow_mld: bool,
    /// Allow LAN discovery chatter: mDNS, LLMNR, SSDP, WS-Discovery,
    /// NetBIOS/SMB, NAT-PMP/PCP.
    pub allow_discovery: bool,
    /// Allow IPsec ESP (next header 50), i.e. encrypted payloads.
    pub allow_esp: bool,
    /// Allow non-IPv6 EtherTypes other than the identification frame:
    /// VLAN tags, LLDP, PPPoE, EAPOL, 802.3/LLC (STP) and the rest.
    pub allow_other_ethertypes: bool,
    /// TCP/UDP ports blocked in either direction (source or destination).
    pub blocked_ports: Vec<u16>,
}

/// Encrypted-transport ports Part 97 operation cannot carry: SSH and
/// HTTPS/QUIC by default.
pub const DEFAULT_BLOCKED_PORTS: [u16; 2] = [22, 443];

/// Further ports worth blocking on an amateur link (all encrypted
/// transports): DNS over TLS, SMTPS, IMAPS, POP3S, RDP, IKE and IPsec
/// NAT-T, WireGuard, HTTPS alternate. Listed here so `--block-port` has a
/// documented menu; not on by default because the operator asked for 22
/// and 443.
pub const RECOMMENDED_EXTRA_PORTS: [u16; 9] = [853, 465, 993, 995, 3389, 500, 4500, 51820, 8443];

impl FilterConfig {
    /// The default policy described in the module documentation.
    pub fn good_neighbor() -> Self {
        Self {
            egress: true,
            ingress: true,
            allow_ipv4: false,
            allow_router_discovery: false,
            allow_dhcpv6: false,
            allow_mld: false,
            allow_discovery: false,
            allow_esp: false,
            allow_other_ethertypes: false,
            blocked_ports: DEFAULT_BLOCKED_PORTS.to_vec(),
        }
    }

    /// Pass everything.
    pub fn off() -> Self {
        Self {
            egress: false,
            ingress: false,
            allow_ipv4: true,
            allow_router_discovery: true,
            allow_dhcpv6: true,
            allow_mld: true,
            allow_discovery: true,
            allow_esp: true,
            allow_other_ethertypes: true,
            blocked_ports: Vec::new(),
        }
    }

    /// Human-readable summary for a startup banner.
    pub fn describe(&self) -> String {
        if !self.egress && !self.ingress {
            return "off".to_string();
        }
        let mut parts = Vec::new();
        parts.push(match (self.egress, self.ingress) {
            (true, true) => "both directions".to_string(),
            (true, false) => "egress only".to_string(),
            _ => "ingress only".to_string(),
        });
        if !self.allow_ipv4 {
            parts.push("IPv6 only".into());
        }
        if !self.allow_router_discovery {
            parts.push("no RA/RS/redirect".into());
        }
        if !self.allow_dhcpv6 {
            parts.push("no DHCPv6".into());
        }
        if !self.allow_mld {
            parts.push("no MLD".into());
        }
        if !self.allow_discovery {
            parts.push("no mDNS/LLMNR/SSDP/WSD/NetBIOS".into());
        }
        if !self.allow_esp {
            parts.push("no ESP".into());
        }
        if !self.blocked_ports.is_empty() {
            let p: Vec<String> = self.blocked_ports.iter().map(|p| p.to_string()).collect();
            parts.push(format!("ports {} blocked", p.join(",")));
        }
        parts.join(", ")
    }
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self::good_neighbor()
    }
}

const ETHERTYPE_IPV6: u16 = 0x86DD;
const ETHERTYPE_IDENT: u16 = crate::ident::ETHERTYPE_IDENT;

/// IPv6 extension headers the walk steps over.
fn is_extension(nh: u8) -> bool {
    matches!(nh, 0 | 43 | 44 | 51 | 60 | 135 | 139 | 140)
}

fn ethertype_name(t: u16) -> &'static str {
    match t {
        0x0800 => "IPv4",
        0x0806 => "ARP",
        0x8100 | 0x88A8 => "VLAN-tagged frame",
        0x88CC => "LLDP",
        0x0842 => "Wake-on-LAN",
        0x8863 | 0x8864 => "PPPoE",
        0x888E => "EAPOL",
        0x8035 => "RARP",
        0x86DD => "IPv6",
        t if t < 0x0600 => "802.3/LLC frame (STP and friends)",
        _ => "other EtherType",
    }
}

fn port_reason(port: u16) -> &'static str {
    match port {
        22 => "SSH (port 22)",
        443 => "HTTPS/QUIC (port 443)",
        853 => "DNS over TLS (port 853)",
        465 => "SMTPS (port 465)",
        993 => "IMAPS (port 993)",
        995 => "POP3S (port 995)",
        3389 => "RDP (port 3389)",
        500 | 4500 => "IKE/IPsec (port 500/4500)",
        51820 => "WireGuard (port 51820)",
        8443 => "HTTPS alternate (port 8443)",
        _ => "blocked port",
    }
}

/// Classify one Ethernet frame (destination, source, EtherType, payload).
pub fn check(cfg: &FilterConfig, frame: &[u8]) -> Verdict {
    if frame.len() < 14 {
        return Verdict::Drop("runt frame");
    }
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    match ethertype {
        ETHERTYPE_IDENT => return Verdict::Pass,
        ETHERTYPE_IPV6 => {}
        0x0800 | 0x0806 | 0x8035 => {
            if !cfg.allow_ipv4 {
                return Verdict::Drop(ethertype_name(ethertype));
            }
            if ethertype == 0x0800 {
                return check_ipv4(cfg, &frame[14..]);
            }
            return Verdict::Pass;
        }
        _ => {
            return if cfg.allow_other_ethertypes { Verdict::Pass } else { Verdict::Drop(ethertype_name(ethertype)) };
        }
    }
    let ip = &frame[14..];
    if ip.len() < 40 || ip[0] >> 4 != 6 {
        return Verdict::Drop("malformed IPv6 header");
    }
    // Walk extension headers to the transport header.
    let mut nh = ip[6];
    let mut pos = 40usize;
    let mut fragment_tail = false;
    while is_extension(nh) {
        if pos + 8 > ip.len() {
            return Verdict::Drop("truncated IPv6 extension header");
        }
        let next = ip[pos];
        let len = match nh {
            44 => {
                // Fragment header: a nonzero offset means the transport
                // header is in another packet; nothing more to see here.
                let off = u16::from_be_bytes([ip[pos + 2], ip[pos + 3]]) >> 3;
                fragment_tail = off != 0;
                8
            }
            51 => (ip[pos + 1] as usize + 2) * 4, // AH
            _ => (ip[pos + 1] as usize + 1) * 8,
        };
        nh = next;
        pos += len;
        if fragment_tail {
            return Verdict::Pass;
        }
    }
    match nh {
        4 => Verdict::Drop("IPv4 tunnelled in IPv6"),
        50 if !cfg.allow_esp => Verdict::Drop("IPsec ESP (encrypted)"),
        58 => check_icmpv6(cfg, ip.get(pos..).unwrap_or(&[])),
        6 | 17 => check_ports(cfg, nh, ip.get(pos..).unwrap_or(&[])),
        _ => Verdict::Pass,
    }
}

fn check_icmpv6(cfg: &FilterConfig, icmp: &[u8]) -> Verdict {
    let Some(&ty) = icmp.first() else { return Verdict::Drop("truncated ICMPv6") };
    match ty {
        133 if !cfg.allow_router_discovery => Verdict::Drop("ICMPv6 Router Solicitation"),
        134 if !cfg.allow_router_discovery => Verdict::Drop("ICMPv6 Router Advertisement"),
        137 if !cfg.allow_router_discovery => Verdict::Drop("ICMPv6 Redirect"),
        130 | 131 | 132 | 143 if !cfg.allow_mld => Verdict::Drop("MLD"),
        _ => Verdict::Pass,
    }
}

fn check_ports(cfg: &FilterConfig, proto: u8, transport: &[u8]) -> Verdict {
    if transport.len() < 4 {
        return Verdict::Drop("truncated transport header");
    }
    let src = u16::from_be_bytes([transport[0], transport[1]]);
    let dst = u16::from_be_bytes([transport[2], transport[3]]);
    for p in [src, dst] {
        if cfg.blocked_ports.contains(&p) {
            return Verdict::Drop(port_reason(p));
        }
    }
    let udp = proto == 17;
    if !cfg.allow_dhcpv6 && udp && (dst == 546 || dst == 547 || src == 546 || src == 547) {
        return Verdict::Drop("DHCPv6");
    }
    if !cfg.allow_discovery {
        for p in [src, dst] {
            let r = match (udp, p) {
                (true, 5353) => Some("mDNS"),
                (true, 5355) => Some("LLMNR"),
                (true, 1900) => Some("SSDP/UPnP"),
                (true, 3702) => Some("WS-Discovery"),
                (true, 137 | 138) | (false, 139) => Some("NetBIOS"),
                (false, 445) => Some("SMB"),
                (true, 5350 | 5351) => Some("NAT-PMP/PCP"),
                (false, 5357 | 5358) => Some("WS-Discovery HTTP"),
                _ => None,
            };
            if let Some(r) = r {
                return Verdict::Drop(r);
            }
        }
    }
    Verdict::Pass
}

/// Only reached when IPv4 is allowed: still apply the port rules.
fn check_ipv4(cfg: &FilterConfig, ip: &[u8]) -> Verdict {
    if ip.len() < 20 || ip[0] >> 4 != 4 {
        return Verdict::Drop("malformed IPv4 header");
    }
    let ihl = (ip[0] & 0x0f) as usize * 4;
    let proto = ip[9];
    let frag_off = u16::from_be_bytes([ip[6], ip[7]]) & 0x1fff;
    if frag_off != 0 {
        return Verdict::Pass;
    }
    match proto {
        6 | 17 => check_ports(cfg, proto, ip.get(ihl..).unwrap_or(&[])),
        50 if !cfg.allow_esp => Verdict::Drop("IPsec ESP (encrypted)"),
        _ => Verdict::Pass,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eth(ethertype: u16, payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0x33, 0x33, 0, 0, 0, 1, 2, 0, 0, 0, 0, 9];
        f.extend_from_slice(&ethertype.to_be_bytes());
        f.extend_from_slice(payload);
        f
    }

    fn ipv6(next: u8, payload: &[u8]) -> Vec<u8> {
        let mut p = vec![0x60, 0, 0, 0];
        p.extend_from_slice(&(payload.len() as u16).to_be_bytes());
        p.push(next);
        p.push(64);
        p.extend_from_slice(&[0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        p.extend_from_slice(&[0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        p.extend_from_slice(payload);
        p
    }

    fn udp(src: u16, dst: u16) -> Vec<u8> {
        let mut t = Vec::new();
        t.extend_from_slice(&src.to_be_bytes());
        t.extend_from_slice(&dst.to_be_bytes());
        t.extend_from_slice(&[0, 12, 0, 0, 1, 2, 3, 4]);
        t
    }

    fn tcp(src: u16, dst: u16) -> Vec<u8> {
        let mut t = udp(src, dst);
        t.resize(20, 0);
        t
    }

    fn icmpv6(ty: u8) -> Vec<u8> {
        vec![ty, 0, 0, 0, 0, 0, 0, 0]
    }

    fn v(frame: &[u8]) -> Verdict {
        check(&FilterConfig::good_neighbor(), frame)
    }

    #[test]
    fn ipv4_arp_and_odd_ethertypes_are_dropped() {
        assert_eq!(v(&eth(0x0800, &[0x45; 40])), Verdict::Drop("IPv4"));
        assert_eq!(v(&eth(0x0806, &[0; 28])), Verdict::Drop("ARP"));
        assert_eq!(v(&eth(0x8100, &[0; 40])), Verdict::Drop("VLAN-tagged frame"));
        assert_eq!(v(&eth(0x88CC, &[0; 40])), Verdict::Drop("LLDP"));
        assert_eq!(v(&eth(0x0040, &[0x42, 0x42, 3, 0, 0])), Verdict::Drop("802.3/LLC frame (STP and friends)"));
        assert_eq!(v(&eth(0x88B5, b"DE W1AW")), Verdict::Pass);
        assert_eq!(v(&[1, 2, 3]), Verdict::Drop("runt frame"));
    }

    #[test]
    fn encrypted_ports_both_directions() {
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(51000, 443)))), Verdict::Drop("HTTPS/QUIC (port 443)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(22, 51000)))), Verdict::Drop("SSH (port 22)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(51000, 443)))), Verdict::Drop("HTTPS/QUIC (port 443)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(51000, 80)))), Verdict::Pass);
        // Extra ports opt in.
        let mut cfg = FilterConfig::good_neighbor();
        cfg.blocked_ports.push(51820);
        assert_eq!(check(&cfg, &eth(0x86DD, &ipv6(17, &udp(51820, 51820)))), Verdict::Drop("WireGuard (port 51820)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(51820, 51820)))), Verdict::Pass);
    }

    #[test]
    fn router_discovery_dhcpv6_mld_and_lan_chatter_are_dropped() {
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(133)))), Verdict::Drop("ICMPv6 Router Solicitation"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(134)))), Verdict::Drop("ICMPv6 Router Advertisement"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(137)))), Verdict::Drop("ICMPv6 Redirect"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(143)))), Verdict::Drop("MLD"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(546, 547)))), Verdict::Drop("DHCPv6"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(5353, 5353)))), Verdict::Drop("mDNS"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 5355)))), Verdict::Drop("LLMNR"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 1900)))), Verdict::Drop("SSDP/UPnP"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 3702)))), Verdict::Drop("WS-Discovery"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(50000, 445)))), Verdict::Drop("SMB"));
        assert_eq!(v(&eth(0x86DD, &ipv6(50, &[0; 16]))), Verdict::Drop("IPsec ESP (encrypted)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(4, &[0x45; 20]))), Verdict::Drop("IPv4 tunnelled in IPv6"));
    }

    #[test]
    fn what_a_babel_mesh_needs_passes() {
        // Neighbour discovery, echo, errors, Babel, DNS, NTP, OSPFv3.
        for ty in [135u8, 136, 128, 129, 1, 2, 3, 4] {
            assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(ty)))), Verdict::Pass, "ICMPv6 type {ty}");
        }
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(6696, 6696)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(40000, 53)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(123, 123)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(89, &[0; 16]))), Verdict::Pass);
        // Hop-by-hop options in front of Babel are walked, not dropped.
        let mut hbh = vec![17u8, 0, 5, 2, 0, 0, 1, 0]; // next = UDP, len 0 (8 octets), router alert
        hbh.extend_from_slice(&udp(6696, 6696));
        assert_eq!(v(&eth(0x86DD, &ipv6(0, &hbh))), Verdict::Pass);
        // Hop-by-hop in front of an RA is still an RA.
        let mut hbh = vec![58u8, 0, 5, 2, 0, 0, 1, 0];
        hbh.extend_from_slice(&icmpv6(134));
        assert_eq!(v(&eth(0x86DD, &ipv6(0, &hbh))), Verdict::Drop("ICMPv6 Router Advertisement"));
        // A non-first fragment cannot be inspected and is let through.
        let frag = [6u8, 0, 0x05, 0xa8, 0, 0, 0, 1];
        assert_eq!(v(&eth(0x86DD, &ipv6(44, &frag))), Verdict::Pass);
        // A first fragment is inspected.
        let mut frag0 = vec![6u8, 0, 0, 1, 0, 0, 0, 1];
        frag0.extend_from_slice(&tcp(1, 443));
        assert_eq!(v(&eth(0x86DD, &ipv6(44, &frag0))), Verdict::Drop("HTTPS/QUIC (port 443)"));
    }

    #[test]
    fn switches_and_off() {
        assert_eq!(check(&FilterConfig::off(), &eth(0x0800, &[0x45; 40])), Verdict::Pass);
        let mut cfg = FilterConfig::good_neighbor();
        cfg.allow_ipv4 = true;
        // IPv4 allowed, but the port rules still apply inside it.
        let mut ip4 = vec![0x45, 0, 0, 40, 0, 0, 0, 0, 64, 6, 0, 0, 10, 0, 0, 1, 10, 0, 0, 2];
        ip4.extend_from_slice(&tcp(4000, 22));
        assert_eq!(check(&cfg, &eth(0x0800, &ip4)), Verdict::Drop("SSH (port 22)"));
        cfg.allow_router_discovery = true;
        assert_eq!(check(&cfg, &eth(0x86DD, &ipv6(58, &icmpv6(134)))), Verdict::Pass);
        assert!(FilterConfig::good_neighbor().describe().contains("IPv6 only"));
        assert_eq!(FilterConfig::off().describe(), "off");
    }
}
