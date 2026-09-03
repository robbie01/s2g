//! Stateless "good neighbor" frame filter.
//!
//! A radio link shared with other amateurs should carry only what the
//! operators mean to send, and an OS attached to a TAP interface emits a
//! lot it does not: IPv4 and ARP on a link meant to be IPv6-only, router
//! discovery and DHCPv6 when addressing is static and routing comes from
//! Babel, and the LAN discovery protocols (mDNS, LLMNR, SSDP,
//! WS-Discovery). Part 97 also forbids obscuring the meaning of a
//! transmission, which rules out SSH, TLS and encrypted ESP.
//!
//! The filter is a pure function of one Ethernet frame (no connection
//! tracking, no state), applied to frames leaving for the air and, by
//! default, to frames arriving from it. Everything not explicitly blocked
//! passes: neighbor discovery (NS/NA, needed to resolve link-local
//! addresses on the link), ICMPv6 echo and errors, Babel (UDP 6696 on
//! ff02::1:6), DNS, NTP, HTTP, OSPFv3, AH, anything unlisted.
//!
//! Tunnels are looked into rather than trusted: IPv6-in-IPv6, GRE, VXLAN
//! and ESP-NULL payloads are checked with the same rules (the inner
//! destination may be global, the rest still applies). ESP is recognized as
//! ESP-NULL by the RFC 5879 heuristic (RFC 4303 default padding plus a
//! plausible inner header for one of the usual ICV lengths) and dropped as
//! encrypted otherwise.
//!
//! Defaults, and the reasoning behind them, are in the README section
//! "Good-neighbor filter".

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
    /// Allow IPv4 and ARP (default: IPv6 only, tunnels included).
    pub allow_ipv4: bool,
    /// Allow IPv6 destinations outside link-local, multicast and ULA
    /// scope (the inner packet of a tunnel is always allowed to be global).
    pub allow_global_dst: bool,
    /// Allow ICMPv6 Router Solicitation / Advertisement / Redirect.
    pub allow_router_discovery: bool,
    /// Allow DHCPv6 (UDP 546/547).
    pub allow_dhcpv6: bool,
    /// Allow MLD (ICMPv6 130–132, 143) and Multicast Router Discovery
    /// (151–153).
    pub allow_mld: bool,
    /// Allow LAN discovery chatter: mDNS, LLMNR, SSDP, WS-Discovery,
    /// NetBIOS/SMB, NAT-PMP/PCP.
    pub allow_discovery: bool,
    /// Allow every ESP packet, not only those recognized as ESP-NULL.
    pub allow_esp: bool,
    /// Allow non-IPv6 EtherTypes other than the identification frame:
    /// VLAN tags, LLDP, PPPoE, EAPOL, 802.3/LLC (STP) and the rest.
    pub allow_other_ethertypes: bool,
    /// TCP/UDP ports blocked in either direction (source or destination).
    pub blocked_ports: Vec<u16>,
}

/// Encrypted-transport ports a Part 97 link cannot carry: SSH, HTTPS/QUIC,
/// DNS over TLS, SMTPS, IMAPS, POP3S, RDP, IKE and IPsec NAT-T, WireGuard,
/// HTTPS alternate.
pub const DEFAULT_BLOCKED_PORTS: [u16; 11] = [22, 443, 853, 465, 993, 995, 3389, 500, 4500, 51820, 8443];

impl FilterConfig {
    /// The default policy described in the module documentation.
    pub fn good_neighbor() -> Self {
        Self {
            egress: true,
            ingress: true,
            allow_ipv4: false,
            allow_global_dst: false,
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
            allow_global_dst: true,
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
        if !self.allow_global_dst {
            parts.push("destinations link-local/multicast/ULA (tunnels exempt)".into());
        }
        if !self.allow_router_discovery {
            parts.push("no RA/RS/redirect".into());
        }
        if !self.allow_dhcpv6 {
            parts.push("no DHCPv6".into());
        }
        if !self.allow_mld {
            parts.push("no MLD/MRD".into());
        }
        if !self.allow_discovery {
            parts.push("no mDNS/LLMNR/SSDP/WSD/NetBIOS".into());
        }
        if !self.allow_esp {
            parts.push("ESP-NULL only".into());
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

const ETHERTYPE_IPV4: u16 = 0x0800;
const ETHERTYPE_IPV6: u16 = 0x86DD;
const ETHERTYPE_IDENT: u16 = crate::ident::ETHERTYPE_IDENT;
/// Tunnel nesting the filter follows before giving up.
const MAX_DEPTH: u8 = 3;

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

/// Link-local (fe80::/10), multicast (ff00::/8) or ULA (fc00::/7).
fn scope_ok(dst: &[u8]) -> bool {
    dst[0] == 0xff || (dst[0] == 0xfe && dst[1] & 0xc0 == 0x80) || dst[0] & 0xfe == 0xfc
}

/// Classify one Ethernet frame (destination, source, EtherType, payload).
pub fn check(cfg: &FilterConfig, frame: &[u8]) -> Verdict {
    check_frame(cfg, frame, 0, cfg.allow_global_dst)
}

fn check_frame(cfg: &FilterConfig, frame: &[u8], depth: u8, global_ok: bool) -> Verdict {
    if frame.len() < 14 {
        return Verdict::Drop("runt frame");
    }
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    check_ethertype(cfg, ethertype, &frame[14..], depth, global_ok)
}

fn check_ethertype(cfg: &FilterConfig, ethertype: u16, payload: &[u8], depth: u8, global_ok: bool) -> Verdict {
    match ethertype {
        ETHERTYPE_IDENT => Verdict::Pass,
        ETHERTYPE_IPV6 => check_ipv6(cfg, payload, depth, global_ok),
        ETHERTYPE_IPV4 | 0x0806 | 0x8035 => {
            if !cfg.allow_ipv4 {
                return Verdict::Drop(if depth > 0 { "IPv4 tunneled in IPv6" } else { ethertype_name(ethertype) });
            }
            if ethertype == ETHERTYPE_IPV4 {
                check_ipv4(cfg, payload, depth)
            } else {
                Verdict::Pass
            }
        }
        _ => {
            if cfg.allow_other_ethertypes {
                Verdict::Pass
            } else {
                Verdict::Drop(ethertype_name(ethertype))
            }
        }
    }
}

/// One IPv6 packet: scope rule, extension-header walk, transport rules,
/// tunnels followed.
fn check_ipv6(cfg: &FilterConfig, ip: &[u8], depth: u8, global_ok: bool) -> Verdict {
    if ip.len() < 40 || ip[0] >> 4 != 6 {
        return Verdict::Drop("malformed IPv6 header");
    }
    if !global_ok && !scope_ok(&ip[24..40]) {
        return Verdict::Drop("destination not link-local, multicast or ULA");
    }
    // Walk extension headers to the transport header.
    let mut nh = ip[6];
    let mut pos = 40usize;
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
                if off != 0 {
                    return Verdict::Pass;
                }
                8
            }
            51 => (ip[pos + 1] as usize + 2) * 4, // AH: authentication only, fine
            _ => (ip[pos + 1] as usize + 1) * 8,
        };
        nh = next;
        pos += len;
    }
    let rest = ip.get(pos..).unwrap_or(&[]);
    check_transport(cfg, nh, rest, depth)
}

/// Transport-level rules shared by IPv6, tunnels and ESP-NULL payloads.
fn check_transport(cfg: &FilterConfig, nh: u8, data: &[u8], depth: u8) -> Verdict {
    match nh {
        4 => {
            if cfg.allow_ipv4 {
                check_ipv4(cfg, data, depth + 1)
            } else {
                Verdict::Drop("IPv4 tunneled in IPv6")
            }
        }
        41 => {
            // IPv6-in-IPv6: the inner packet may be for anywhere, the
            // other rules still apply to it.
            if depth >= MAX_DEPTH {
                return Verdict::Drop("tunnel nested too deep");
            }
            check_ipv6(cfg, data, depth + 1, true)
        }
        47 => check_gre(cfg, data, depth),
        50 => check_esp(cfg, data, depth),
        58 => check_icmpv6(cfg, data),
        6 | 17 => check_ports(cfg, nh, data, depth),
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
        151..=153 if !cfg.allow_mld => Verdict::Drop("Multicast Router Discovery"),
        _ => Verdict::Pass,
    }
}

fn check_ports(cfg: &FilterConfig, proto: u8, transport: &[u8], depth: u8) -> Verdict {
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
    // VXLAN carries a whole Ethernet frame after an 8-octet header.
    if udp && dst == 4789 && transport.len() >= 16 + 14 {
        if depth >= MAX_DEPTH {
            return Verdict::Drop("tunnel nested too deep");
        }
        return check_frame(cfg, &transport[16..], depth + 1, true);
    }
    Verdict::Pass
}

/// GRE (RFC 2784/2890): flags, version, protocol type, optional checksum,
/// key and sequence fields, then the payload named by the protocol type.
fn check_gre(cfg: &FilterConfig, gre: &[u8], depth: u8) -> Verdict {
    if gre.len() < 4 {
        return Verdict::Drop("truncated GRE header");
    }
    if depth >= MAX_DEPTH {
        return Verdict::Drop("tunnel nested too deep");
    }
    let flags = gre[0];
    let proto = u16::from_be_bytes([gre[2], gre[3]]);
    let mut hdr = 4;
    if flags & 0x80 != 0 {
        hdr += 4; // checksum + reserved
    }
    if flags & 0x20 != 0 {
        hdr += 4; // key
    }
    if flags & 0x10 != 0 {
        hdr += 4; // sequence number
    }
    let inner = gre.get(hdr..).unwrap_or(&[]);
    match proto {
        // Transparent Ethernet bridging: a whole frame.
        0x6558 => check_frame(cfg, inner, depth + 1, true),
        _ => check_ethertype(cfg, proto, inner, depth + 1, true),
    }
}

/// ESP (RFC 4303): SPI, sequence number, payload, padding, pad length,
/// next header, ICV. The cipher is unknown without the SA, so the RFC 5879
/// heuristic applies: for each usual ICV length the trailer must show
/// RFC 4303 default padding (1, 2, 3, …) and a next-header value whose
/// payload parses. Then the payload is checked like any other packet.
/// Anything else is treated as encrypted.
fn check_esp(cfg: &FilterConfig, esp: &[u8], depth: u8) -> Verdict {
    if cfg.allow_esp {
        return Verdict::Pass;
    }
    if esp.len() < 8 + 2 {
        return Verdict::Drop("truncated ESP");
    }
    if depth >= MAX_DEPTH {
        return Verdict::Drop("tunnel nested too deep");
    }
    for icv in [12usize, 16, 24, 32, 0] {
        if esp.len() < 8 + 2 + icv {
            continue;
        }
        let trailer_end = esp.len() - icv;
        let nh = esp[trailer_end - 1];
        let pad_len = esp[trailer_end - 2] as usize;
        if 8 + pad_len + 2 > trailer_end {
            continue;
        }
        let padding = &esp[trailer_end - 2 - pad_len..trailer_end - 2];
        if padding.iter().enumerate().any(|(i, &b)| b as usize != i + 1) {
            continue;
        }
        let inner = &esp[8..trailer_end - 2 - pad_len];
        let plausible = match nh {
            41 => inner.len() >= 40 && inner[0] >> 4 == 6 && u16::from_be_bytes([inner[4], inner[5]]) as usize <= inner.len() - 40,
            4 => inner.len() >= 20 && inner[0] >> 4 == 4,
            6 => inner.len() >= 20 && (inner[12] >> 4) >= 5,
            17 => inner.len() >= 8 && u16::from_be_bytes([inner[4], inner[5]]) as usize == inner.len(),
            58 => inner.len() >= 4,
            59 => true, // dummy packet (RFC 4303 2.6): nothing inside
            _ => false,
        };
        if !plausible {
            continue;
        }
        return match nh {
            59 => Verdict::Pass,
            _ => check_transport(cfg, nh, inner, depth + 1),
        };
    }
    Verdict::Drop("IPsec ESP (encrypted, or ESP-NULL without RFC 4303 default padding)")
}

/// Only reached when IPv4 is allowed: still apply the port rules.
fn check_ipv4(cfg: &FilterConfig, ip: &[u8], depth: u8) -> Verdict {
    if ip.len() < 20 || ip[0] >> 4 != 4 {
        return Verdict::Drop("malformed IPv4 header");
    }
    let ihl = (ip[0] & 0x0f) as usize * 4;
    let proto = ip[9];
    let frag_off = u16::from_be_bytes([ip[6], ip[7]]) & 0x1fff;
    if frag_off != 0 {
        return Verdict::Pass;
    }
    let rest = ip.get(ihl..).unwrap_or(&[]);
    match proto {
        6 | 17 => check_ports(cfg, proto, rest, depth),
        47 => check_gre(cfg, rest, depth),
        50 => check_esp(cfg, rest, depth),
        _ => Verdict::Pass,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LL_DST: [u8; 16] = [0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2];
    const MC_DST: [u8; 16] = [0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
    const ULA_DST: [u8; 16] = [0xfd, 0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2];
    const GLOBAL_DST: [u8; 16] = [0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2];

    fn eth(ethertype: u16, payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0x33, 0x33, 0, 0, 0, 1, 2, 0, 0, 0, 0, 9];
        f.extend_from_slice(&ethertype.to_be_bytes());
        f.extend_from_slice(payload);
        f
    }

    fn ipv6_to(dst: [u8; 16], next: u8, payload: &[u8]) -> Vec<u8> {
        let mut p = vec![0x60, 0, 0, 0];
        p.extend_from_slice(&(payload.len() as u16).to_be_bytes());
        p.push(next);
        p.push(64);
        p.extend_from_slice(&[0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        p.extend_from_slice(&dst);
        p.extend_from_slice(payload);
        p
    }

    fn ipv6(next: u8, payload: &[u8]) -> Vec<u8> {
        ipv6_to(MC_DST, next, payload)
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
        t[12] = 0x50;
        t
    }

    fn icmpv6(ty: u8) -> Vec<u8> {
        vec![ty, 0, 0, 0, 0, 0, 0, 0]
    }

    /// ESP-NULL packet: SPI, seq, inner, RFC 4303 padding, pad len, next
    /// header, ICV of `icv` octets.
    fn esp_null(next: u8, inner: &[u8], icv: usize) -> Vec<u8> {
        let mut p = vec![0, 0, 0, 7, 0, 0, 0, 1];
        p.extend_from_slice(inner);
        let pad = (4 - (inner.len() + 2) % 4) % 4;
        for i in 0..pad {
            p.push(i as u8 + 1);
        }
        p.push(pad as u8);
        p.push(next);
        p.extend(std::iter::repeat_n(0xAB, icv));
        p
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
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(51820, 51820)))), Verdict::Drop("WireGuard (port 51820)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(51000, 853)))), Verdict::Drop("DNS over TLS (port 853)"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(51000, 80)))), Verdict::Pass);
        let mut cfg = FilterConfig::good_neighbor();
        cfg.blocked_ports.retain(|&p| p != 443);
        assert_eq!(check(&cfg, &eth(0x86DD, &ipv6(6, &tcp(51000, 443)))), Verdict::Pass);
    }

    #[test]
    fn router_discovery_dhcpv6_mld_and_lan_chatter_are_dropped() {
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(133)))), Verdict::Drop("ICMPv6 Router Solicitation"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(134)))), Verdict::Drop("ICMPv6 Router Advertisement"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(137)))), Verdict::Drop("ICMPv6 Redirect"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(143)))), Verdict::Drop("MLD"));
        assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(151)))), Verdict::Drop("Multicast Router Discovery"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(546, 547)))), Verdict::Drop("DHCPv6"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(5353, 5353)))), Verdict::Drop("mDNS"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 5355)))), Verdict::Drop("LLMNR"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 1900)))), Verdict::Drop("SSDP/UPnP"));
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(50000, 3702)))), Verdict::Drop("WS-Discovery"));
        assert_eq!(v(&eth(0x86DD, &ipv6(6, &tcp(50000, 445)))), Verdict::Drop("SMB"));
        assert_eq!(v(&eth(0x86DD, &ipv6(4, &[0x45; 20]))), Verdict::Drop("IPv4 tunneled in IPv6"));
    }

    #[test]
    fn destination_scope() {
        let babel = udp(6696, 6696);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(LL_DST, 17, &babel))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(MC_DST, 17, &babel))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 17, &babel))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(GLOBAL_DST, 17, &babel))), Verdict::Drop("destination not link-local, multicast or ULA"));
        let mut cfg = FilterConfig::good_neighbor();
        cfg.allow_global_dst = true;
        assert_eq!(check(&cfg, &eth(0x86DD, &ipv6_to(GLOBAL_DST, 17, &babel))), Verdict::Pass);
        // A global destination inside an IPv6-in-IPv6 tunnel to a ULA peer
        // is fine; a blocked port inside the tunnel is still blocked.
        let inner_ok = ipv6_to(GLOBAL_DST, 17, &udp(40000, 53));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 41, &inner_ok))), Verdict::Pass);
        let inner_bad = ipv6_to(GLOBAL_DST, 6, &tcp(40000, 443));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 41, &inner_bad))), Verdict::Drop("HTTPS/QUIC (port 443)"));
        // But a tunnel to a global outer destination is not.
        assert_eq!(v(&eth(0x86DD, &ipv6_to(GLOBAL_DST, 41, &inner_ok))), Verdict::Drop("destination not link-local, multicast or ULA"));
    }

    #[test]
    fn tunnels_are_looked_into() {
        // GRE carrying IPv4: dropped. GRE carrying IPv6 to a global host: fine.
        let mut gre4 = vec![0, 0, 0x08, 0x00];
        gre4.extend_from_slice(&[0x45; 24]);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 47, &gre4))), Verdict::Drop("IPv4 tunneled in IPv6"));
        let mut gre6 = vec![0x20, 0, 0x86, 0xDD, 0, 0, 0, 5]; // key present
        gre6.extend_from_slice(&ipv6_to(GLOBAL_DST, 17, &udp(1, 80)));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 47, &gre6))), Verdict::Pass);
        // GRE bridging a whole Ethernet frame that carries ARP: dropped.
        let mut greb = vec![0, 0, 0x65, 0x58];
        greb.extend_from_slice(&eth(0x0806, &[0; 28]));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 47, &greb))), Verdict::Drop("IPv4 tunneled in IPv6"));
        // VXLAN with an inner IPv6 frame to a global address: fine; inner IPv4: dropped.
        let mut vx6 = udp(40000, 4789);
        vx6.truncate(8);
        vx6.extend_from_slice(&[0x08, 0, 0, 0, 0, 0, 1, 0]);
        let mut vx4 = vx6.clone();
        vx6.extend_from_slice(&eth(0x86DD, &ipv6_to(GLOBAL_DST, 17, &udp(1, 80))));
        vx4.extend_from_slice(&eth(0x0800, &[0x45; 40]));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 17, &vx6))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 17, &vx4))), Verdict::Drop("IPv4 tunneled in IPv6"));
    }

    #[test]
    fn esp_null_passes_and_is_inspected_encrypted_esp_is_dropped() {
        // ESP-NULL tunnel mode: inner IPv6 to a global host, HMAC-SHA1-96 ICV.
        let inner = ipv6_to(GLOBAL_DST, 17, &udp(40000, 53));
        for icv in [12usize, 16, 32] {
            assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 50, &esp_null(41, &inner, icv)))), Verdict::Pass, "icv {icv}");
        }
        // ESP-NULL transport mode carrying TCP 443: the port rule still bites.
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 50, &esp_null(6, &tcp(1, 443), 12)))), Verdict::Drop("HTTPS/QUIC (port 443)"));
        // ESP-NULL carrying IPv4: dropped like any tunneled IPv4.
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 50, &esp_null(4, &[0x45; 24], 12)))), Verdict::Drop("IPv4 tunneled in IPv6"));
        // A dummy packet (next header 59) passes.
        assert_eq!(v(&eth(0x86DD, &ipv6_to(ULA_DST, 50, &esp_null(59, &[], 12)))), Verdict::Pass);
        // Ciphertext: no default padding, no plausible next header.
        let mut enc = vec![0, 0, 0, 7, 0, 0, 0, 1];
        enc.extend((0..60u32).map(|i| (i.wrapping_mul(2654435761) >> 13) as u8));
        assert_eq!(
            v(&eth(0x86DD, &ipv6_to(ULA_DST, 50, &enc))),
            Verdict::Drop("IPsec ESP (encrypted, or ESP-NULL without RFC 4303 default padding)")
        );
        // Explicitly allowed: anything.
        let mut cfg = FilterConfig::good_neighbor();
        cfg.allow_esp = true;
        assert_eq!(check(&cfg, &eth(0x86DD, &ipv6_to(ULA_DST, 50, &enc))), Verdict::Pass);
        // AH transport mode in front of Babel is an extension header: fine.
        let mut ah = vec![17u8, 4, 0, 0, 0, 0, 0, 9, 0, 0, 0, 1];
        ah.extend_from_slice(&[0xCC; 12]);
        ah.extend_from_slice(&udp(6696, 6696));
        assert_eq!(v(&eth(0x86DD, &ipv6_to(LL_DST, 51, &ah))), Verdict::Pass);
    }

    #[test]
    fn what_a_babel_mesh_needs_passes() {
        for ty in [135u8, 136, 128, 129, 1, 2, 3, 4] {
            assert_eq!(v(&eth(0x86DD, &ipv6(58, &icmpv6(ty)))), Verdict::Pass, "ICMPv6 type {ty}");
        }
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(6696, 6696)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(40000, 53)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(17, &udp(123, 123)))), Verdict::Pass);
        assert_eq!(v(&eth(0x86DD, &ipv6(89, &[0; 16]))), Verdict::Pass);
        // Hop-by-hop options in front of Babel are walked, not dropped.
        let mut hbh = vec![17u8, 0, 5, 2, 0, 0, 1, 0];
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
        assert!(FilterConfig::good_neighbor().describe().contains("ESP-NULL only"));
        assert_eq!(FilterConfig::off().describe(), "off");
    }
}
