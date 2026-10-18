#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 1mm0rt41PC — https://github.com/1mm0rt41PC/profibus-pbmaster
"""
pbmaster.py — mini maître PROFIBUS-DP classe 1 (DP-V0) en Python/pyserial.

ATTENTION: Requière un connecteur USB <=> Serial tel que https://github.com/INNO-MAKER/USB2RS485-C/blob/main/USB2RS485-C%20UserManual-v1.0.pdf

Testé sur le papier contre un Allen-Bradley 1794-APB/B (= Invensys I/A Series
G9900APBA) + 1794-OB16, via un convertisseur USB/RS-485 sur /dev/ttyUSB0.

    pip install pyserial          (ou: apt install python3-serial)
    sudo usermod -aG dialout $USER   (puis se reloguer)

Exemples :
    ./pbmaster.py probe                      # balaye les vitesses + adresses, puis inspecte
                                             # chaque esclave (ident, diag, config, modules)
    ./pbmaster.py scan                       # qui répond sur le bus (à --baud) ?
    ./pbmaster.py diag --slave 5             # diagnostic + config lue sur l'esclave
    ./pbmaster.py identify --slave 5         # référence exacte des cartes Flex I/O par slot
    ./pbmaster.py run  --slave 5             # paramétrage, config, puis échange cyclique
    ./pbmaster.py run  --slave 5 --out 0000FFFF   # sorties initiales (hex)
    ./pbmaster.py run  --slave 3 --account admin:secret      # interactif + API HTTP
    ./pbmaster.py httpd --slave 3 --account admin:secret [--http-bind 127.0.0.1:8080] [--json-key-prefix F]
                                             # daemon : API HTTP seule, sans clavier (systemd)
    sudo ./pbmaster.py install --slave 3 --account admin:secret [--http-bind ...] [--json-key-prefix F]
                                             # installe + démarre le service systemd pbmaster
                                             # (script -> /usr/local/bin, identifiants -> /etc/pbmaster.conf)
    sudo ./pbmaster.py uninstall

API HTTP (auth basic). Points nommés E<slot>.<bit> (entrées) et S<slot>.<bit> (sorties),
<slot> = position de la carte Flex I/O (1..8), <bit> = 0..15 :
    GET  /in     -> {"E1.0":1,"E1.1":0,...}
    GET  /out    -> {"S2.0":0,...}            état courant des sorties
    POST /out    <- {"S2.3":1,"S3.0":0}       tout est validé avant d'être appliqué ;
                                              une clé inconnue, un E, un slot du mauvais
                                              type ou un bit >15 -> 400 et rien n'est écrit
    GET  /io     -> cartes + entrées + sorties + âge des données
    GET  /       -> page HTML : tableau des cartes, LED pour les entrées, cases à cocher
                    pour les sorties (rafraîchie toutes les 500 ms)
    GET  /cards  -> {"E1":"entrées (16 bits)","S2":"sorties (16 bits)",...}
    curl -u admin:secret -X POST -d '{"S2.0":1}' http://host:8080/out

Numérotation des modules logiques sur un 1794-APB : chaque slot = 2 identifiants
(entrée, sortie). #0/#1 = mot de statut adaptateur, #2/#3 = slot 1, #4/#5 = slot 2…
Avec un seul 1794-OB16 en slot 1, son mot de sortie est le module #3.

En mode `run`, commandes au clavier (une par ligne). <mod> = numéro de module
logique (#3) ou de slot Flex I/O (s2) :
    b <mod> <bit> <0|1>   met un bit de sortie (ex: b s2 3 1 -> sortie 3 du slot 2 à 1)
    w <mod> <hex>         écrit les octets de sortie d'un module (ex: w s2 00FF0000)
    on <mod> / off <mod>  toutes les sorties du module à 1 / 0
    chrismas <mod>[,<mod>] [pas] [durée]
                          guirlande animée ; pas = durée d'une image (défaut 150ms),
                          durée totale optionnelle (ex: chrismas s2,s3 1s 60s ; 500ms, 2m, 1h)
    stop                  arrête la guirlande (sorties à 0)
    o <hex>               remplace tout le buffer de sortie
    i                     affiche les entrées courantes
    d                     relit le diagnostic esclave
    q                     quitte (après avoir remis les sorties à 0)

Couche physique : PROFIBUS = RS-485, 8E1 ("11 bits"), 9600 bauds par défaut ici.
Sur la DB9 de l'adaptateur : pin 3 = B (+, rouge), pin 8 = A (-, vert), pin 5 = GND,
pin 6 = +5 V pour la terminaison (390/220/390 Ω). Attention à la convention A/B
inversée des puces type MAX485 : leur "A/D+" va sur la pin 3 PROFIBUS.
"""

import argparse
import os
import queue
import random
import re
import signal
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant : pip install pyserial")

# ---------------------------------------------------------------------------
# Constantes FDL / DP
# ---------------------------------------------------------------------------
SD1, SD2, SD3, SC, ED = 0x10, 0x68, 0xA2, 0xE5, 0x16

FC_REQ, FC_FCB, FC_FCV = 0x40, 0x20, 0x10
FN_SRD_HI = 0x0D          # Send and Request Data, haute priorité
FN_FDL_STATUS = 0x09      # Request FDL status
FN_IDENT = 0x0E           # Request Ident with reply

# vitesses PROFIBUS ; au-delà de 500 kbit/s un PC/FTDI ne tient plus le timing
PROBE_BAUDS = [9600, 19200, 45450, 93750, 187500, 500000, 1500000]

# (mots lus, mots écrits) tels que renvoyés par Get_Cfg d'un 1794-APB au power-up
# (tailles maxi du module, manuel 1794-6.5.6 ch. 2) -> modules Flex I/O probables
# Clés module (Module ID) du 1794-APB, manuel 1794-6.5.6 ch. 4 (format condensé)
FLEX_KEYS = [(0x0281, "1794-IB16  16 entrées 24 V DC (sink)"),
             (0x0191, "1794-OB16  16 sorties 24 V DC (source)"),
             (0x0285, "1794-IA8   8 entrées 120 V AC"),
             (0x0195, "1794-OA8   8 sorties 120 V AC"),
             (0x0289, "1794-IB8S  8 entrées capteurs 24 V DC"),
             (0x0199, "1794-OW8   8 sorties relais"),
             (0x1920, "1794-IE8/A 8 entrées analogiques"),
             (0x1924, "1794-IE8/B 8 entrées analogiques"),
             (0x1121, "1794-OE4/A 4 sorties analogiques"),
             (0x1125, "1794-OE4/B 4 sorties analogiques"),
             (0x1522, "1794-IE4XOE2/A 4 entrées / 2 sorties analogiques"),
             (0x1526, "1794-IE4XOE2/B 4 entrées / 2 sorties analogiques"),
             (0x1B00, "1794-IT8   8 thermocouples"),
             (0x1B01, "1794-IR8   8 RTD"),
             (0x1600, "1203-FM1   SCANport")]

FLEX_GUESS = {(2, 1): "1794-IB16 / IB8S / IA8 (entrées TOR)",
              (1, 2): "1794-OB16 / OA8 / OW8 (sorties TOR)",
              (9, 6): "1794-IE8", (1, 6): "1794-OE4", (1, 14): "1794-OE4",
              (5, 4): "1794-IE4XOE2", (5, 10): "1794-IE4XOE2",
              (11, 3): "1794-IR8 / IT8", (11, 4): "1794-IR8 / IT8",
              (6, 5): "1203-FM1"}

SAP_GET_CFG = 59
SAP_SLAVE_DIAG = 60
SAP_SET_PRM = 61
SAP_CHK_CFG = 62
SAP_MASTER = 62           # SSAP côté maître pour les services DP

RESP_NAMES = {0x0: "OK", 0x1: "UE (erreur utilisateur)", 0x2: "RR (pas de ressource)",
              0x3: "RS (SAP non activé)", 0x8: "DL (données)", 0x9: "NR (pas de données)",
              0xA: "DH (données, prio haute)", 0xC: "RDL", 0xD: "RDH"}

STATUS1 = ["Station_Non_Existent", "Station_Not_Ready", "Cfg_Fault", "Ext_Diag",
           "Not_Supported", "Invalid_Slave_Response", "Prm_Fault", "Master_Lock"]
STATUS2 = ["Prm_Req", "Stat_Diag", "(always 1)", "WD_On", "Freeze_Mode", "Sync_Mode",
           "reserved", "Deactivated"]


class PBError(Exception):
    pass


class PBTimeout(PBError):
    pass


def hx(b):
    return " ".join("%02X" % x for x in b)


# ---------------------------------------------------------------------------
# Maître
# ---------------------------------------------------------------------------
class PBMaster:
    def __init__(self, port, baud=9600, master_addr=1, timeout=0.15,
                 debug=False, rts_toggle=False):
        self.ser = serial.Serial(port, baud, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
                                 timeout=0.02)
        try:
            self.ser.set_low_latency_mode(True)   # FTDI : latence 1 ms au lieu de 16
        except Exception:
            pass
        self.addr = master_addr
        self.bit_t = 1.0 / baud
        self.timeout = timeout
        self.debug = debug
        self.rts_toggle = rts_toggle
        self.fcb = {}            # état FCB par esclave (None = à réinitialiser)
        self.noise = 0           # octets reçus hors trame valide (polarité ? vitesse ?)
        self.echo_seen = False   # l'adaptateur RS-485 renvoie-t-il nos propres trames ?
        if rts_toggle:
            self.ser.rts = False

    def set_baud(self, baud):
        self.ser.baudrate = baud      # pyserial gère les vitesses non standard sous Linux
        self.bit_t = 1.0 / baud
        self.fcb = {}
        self.noise = 0

    # ----- bas niveau --------------------------------------------------
    def _fcb_bits(self, da):
        st = self.fcb.get(da)
        if st is None:           # 1er télégramme : FCV=0, FCB=1 (init côté esclave)
            self.fcb[da] = 1
            return FC_FCB
        st ^= 1
        self.fcb[da] = st
        return FC_FCV | (FC_FCB if st else 0)

    def _build(self, da, fc, data=b"", dsap=None, ssap=None):
        if dsap is None:
            body = bytes([da, self.addr, fc]) + data
        else:
            body = bytes([da | 0x80, self.addr | 0x80, fc, dsap, ssap]) + data
        fcs = sum(body) & 0xFF
        if dsap is None and not data:
            return bytes([SD1]) + body + bytes([fcs, ED])
        le = len(body)
        return bytes([SD2, le, le, SD2]) + body + bytes([fcs, ED])

    def _write(self, frame):
        time.sleep(max(40 * self.bit_t, 0.002))        # Tsyn : >= 33 bits de repos
        self.ser.reset_input_buffer()
        if self.rts_toggle:
            self.ser.rts = True
        if self.debug:
            print("TX:", hx(frame))
        self.ser.write(frame)
        self.ser.flush()
        if self.rts_toggle:
            time.sleep(len(frame) * 11 * self.bit_t + 0.0005)
            self.ser.rts = False

    @staticmethod
    def _expected_len(buf):
        sd = buf[0]
        if sd == SC:
            return 1
        if sd == SD1:
            return 6
        if sd == SD3:
            return 14
        if sd == SD2:
            return buf[1] + 6 if len(buf) >= 2 else None
        raise PBError("délimiteur inconnu 0x%02X (bruit/parité/câblage ?) : %s" % (sd, hx(buf)))

    def _read_frame(self, sent, timeout):
        deadline = time.monotonic() + len(sent) * 11 * self.bit_t + timeout
        buf = bytearray()
        echo_done = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if buf:
                    raise PBTimeout("trame incomplète : %s" % hx(buf))
                raise PBTimeout("pas de réponse")
            self.ser.timeout = min(remaining, 0.05)
            # on ignore les octets de bruit avant un délimiteur valide
            if not buf:
                b = self.ser.read(1)
                if not b:
                    continue
                if b[0] not in (SC, SD1, SD2, SD3):
                    self.noise += 1
                    if self.debug:
                        print("RX (ignoré):", hx(b))
                    continue
                buf += b
            need = self._expected_len(buf)
            if need is None:
                buf += self.ser.read(1)
                continue
            if len(buf) < need:
                buf += self.ser.read(need - len(buf))
                continue
            # trame complète
            if not echo_done and bytes(buf) == sent:       # écho half-duplex
                echo_done = self.echo_seen = True
                buf = bytearray()
                continue
            if self.debug:
                print("RX:", hx(buf))
            return self._parse(bytes(buf))

    @staticmethod
    def _parse(f):
        if f[0] == SC:
            return {"sc": True, "fc": 0, "data": b""}
        if f[0] == SD1:
            body, fcs, ed = f[1:4], f[4], f[5]
        elif f[0] == SD3:
            body, fcs, ed = f[1:12], f[12], f[13]
        else:
            le = f[1]
            body, fcs, ed = f[4:4 + le], f[4 + le], f[5 + le]
        if ed != ED:
            raise PBError("ED invalide : %s" % hx(f))
        if (sum(body) & 0xFF) != fcs:
            raise PBError("FCS invalide : %s" % hx(f))
        r = {"sc": False, "da": body[0] & 0x7F, "sa": body[1] & 0x7F, "fc": body[2]}
        if body[0] & 0x80:
            r["dsap"], r["ssap"], r["data"] = body[3], body[4], bytes(body[5:])
        else:
            r["data"] = bytes(body[3:])
        return r

    # ----- services FDL -------------------------------------------------
    def fdl_status(self, da, timeout=0.04):
        frame = self._build(da, FC_REQ | FN_FDL_STATUS)
        self._write(frame)
        try:
            r = self._read_frame(frame, timeout)
        except PBTimeout:
            return None
        return r["fc"]

    def request_ident(self, da):
        """FDL Request Ident : chaînes vendeur / contrôleur / HW / SW (si supporté)."""
        frame = self._build(da, FC_REQ | FN_IDENT)
        self._write(frame)
        try:
            r = self._read_frame(frame, self.timeout)
        except PBError:
            return None
        if r["sc"] or (r["fc"] & 0x0F) not in (0x8, 0xA):
            return None
        d, out, i = r["data"], [], 0
        while i < len(d):            # 4 chaînes préfixées par leur longueur
            n = d[i]
            out.append(d[i + 1:i + 1 + n].decode("ascii", "replace").strip())
            i += 1 + n
        return out

    def srd(self, da, data=b"", dsap=None, ssap=None, retries=3):
        fc = FC_REQ | self._fcb_bits(da) | FN_SRD_HI
        frame = self._build(da, fc, data, dsap, ssap)
        last = None
        for _ in range(retries):
            self._write(frame)
            try:
                r = self._read_frame(frame, self.timeout)
            except PBError as e:
                last = e
                continue
            if r["sc"]:
                return b""
            if r["fc"] & FC_REQ:
                last = PBError("reçu une requête au lieu d'une réponse (autre maître ?)")
                continue
            code = r["fc"] & 0x0F
            if code in (0x8, 0xA, 0xC, 0xD, 0x0):
                return r["data"]
            raise PBError("réponse négative de l'esclave %d : %s" %
                          (da, RESP_NAMES.get(code, "0x%X" % code)))
        self.fcb[da] = None
        raise last

    # ----- services DP ---------------------------------------------------
    def slave_diag(self, da):
        d = self.srd(da, b"", SAP_SLAVE_DIAG, SAP_MASTER)
        if len(d) < 6:
            raise PBError("diag trop court : %s" % hx(d))
        return d

    def set_prm(self, da, ident, user_prm=b"\x00", wd_on=True, wd1=10, wd2=30,
                min_tsdr=11, lock=True):
        status = (0x80 if lock else 0x40) | (0x08 if wd_on else 0)
        prm = bytes([status, wd1, wd2, min_tsdr, ident >> 8, ident & 0xFF, 0]) + user_prm
        return self.srd(da, prm, SAP_SET_PRM, SAP_MASTER)

    def chk_cfg(self, da, cfg):
        return self.srd(da, cfg, SAP_CHK_CFG, SAP_MASTER)

    def get_cfg(self, da):
        return self.srd(da, b"", SAP_GET_CFG, SAP_MASTER)

    def data_exchange(self, da, out):
        return self.srd(da, out)


# ---------------------------------------------------------------------------
# Décodage DP
# ---------------------------------------------------------------------------
def parse_cfg(cfg):
    """Décode les octets identifiants (format général) -> liste de modules."""
    mods, in_off, out_off, i = [], 0, 0, 0
    while i < len(cfg):
        b = cfg[i]
        i += 1
        if b == 0x00:
            mods.append({"idx": len(mods), "id": b, "in_off": in_off, "in_len": 0,
                         "out_off": out_off, "out_len": 0, "word": False})
            continue
        if not (b & 0x30):
            raise PBError("identifiant au format spécial 0x%02X non géré" % b)
        n = (b & 0x0F) + 1
        word = bool(b & 0x40)
        n_bytes = n * 2 if word else n
        il = n_bytes if b & 0x10 else 0
        ol = n_bytes if b & 0x20 else 0
        mods.append({"idx": len(mods), "id": b, "in_off": in_off, "in_len": il,
                     "out_off": out_off, "out_len": ol, "word": word})
        in_off += il
        out_off += ol
    return mods, in_off, out_off


def describe_layout(mods, nin, nout):
    print("Modules logiques (Chk_Cfg) :")
    for m in mods:
        desc = "vide" if m["id"] == 0 else "%s%s%s" % (
            "in %d o " % m["in_len"] if m["in_len"] else "",
            "out %d o " % m["out_len"] if m["out_len"] else "",
            "(mots)" if m["word"] else "(octets)")
        where = []
        if m["in_len"]:
            where.append("in[%d:%d]" % (m["in_off"], m["in_off"] + m["in_len"]))
        if m["out_len"]:
            where.append("out[%d:%d]" % (m["out_off"], m["out_off"] + m["out_len"]))
        print("  #%-2d 0x%02X  %-22s %s" % (m["idx"], m["id"], desc, " ".join(where)))
    print("Entrées : %d octet(s), sorties : %d octet(s)" % (nin, nout))
    if len(mods) >= 2 and mods[0]["id"] == 0x50 and mods[1]["id"] == 0x60:
        print("  (1794-APB : #0/#1 = mot de statut adaptateur ; slot n = modules #%s)"
              % ", #".join("%d/#%d" % (2 * s, 2 * s + 1) for s in range(1, (len(mods)) // 2)))


def decode_diag(d):
    s1, s2, s3, master = d[0], d[1], d[2], d[3]
    ident = (d[4] << 8) | d[5]
    print("Diag : status1=0x%02X status2=0x%02X status3=0x%02X master=%d ident=0x%04X"
          % (s1, s2, s3, master, ident))
    f1 = [n for i, n in enumerate(STATUS1) if s1 & (1 << i)]
    f2 = [n for i, n in enumerate(STATUS2) if s2 & (1 << i) and i != 2]
    if f1:
        print("  status1 :", ", ".join(f1))
    if f2:
        print("  status2 :", ", ".join(f2))
    if s3 & 0x80:
        print("  status3 : Ext_Diag_Overflow")
    ext = d[6:]
    if ext:
        print("  ext diag :", hx(ext))
    i = 0
    while i < len(ext):
        h = ext[i]
        typ, ln = h >> 6, h & 0x3F
        if typ == 0 and ln >= 1:                         # bloc "device related"
            blk = ext[i + 1:i + ln]
            if len(blk) == 4:                            # format 1794-APB
                print("  APB : firmware %d.%d, status 0x%02X%s, roues codeuses = %02d"
                      % (blk[1], blk[0], blk[2],
                         " (adresse modifiée depuis le power-up)" if blk[2] & 1 else "",
                         blk[3]))
            i += ln
        elif typ == 1:                                   # "identifier related"
            bits = ext[i + 1:i + ln]
            faults = [k * 8 + b for k, byte in enumerate(bits)
                      for b in range(8) if byte & (1 << b)]
            if faults:
                slots = sorted({f // 2 for f in faults if f >= 2})
                where = ["slot %d" % s for s in slots] + (["adaptateur"] if min(faults) < 2 else [])
                print("  modules en défaut : %s (bits %s)" % (", ".join(where), faults))
            i += ln
        elif typ == 2:                                   # "channel related" : 3 octets
            i += 3
        else:
            break
    return ident


def set_bits(data, word):
    """Numéros des bits à 1 (bit 0 = entrée/sortie 0 ; mots : poids fort en premier)."""
    bits = []
    for i, byte in enumerate(data):
        for b in range(8):
            if byte & (1 << b):
                if word:
                    bits.append((i // 2) * 16 + (8 if i % 2 == 0 else 0) + b)
                else:
                    bits.append(i * 8 + b)
    return sorted(bits)


def apb_slots(mods):
    """Pour un 1794-APB : liste de (slot, module_in, module_out) des slots non vides."""
    if len(mods) < 2 or mods[0]["id"] != 0x50 or mods[1]["id"] != 0x60:
        return []
    slots = []
    for s in range(1, len(mods) // 2):
        mi, mo = mods[2 * s], mods[2 * s + 1]
        if mi["in_len"] or mo["out_len"]:
            slots.append((s, mi, mo))
    return slots


def ruler(data, word, n=16):
    """Réglette : X pour un bit à 1, . sinon, bit 0 à gauche."""
    on = set(set_bits(data, word))
    return "".join("X" if i in on else "." for i in range(min(n, len(data) * 8)))


def friendly_io(inp, out, mods):
    """Lignes lisibles par slot (APB) : entrées et sorties actives."""
    lines = []
    for s, mi, mo in apb_slots(mods):
        # module d'entrées (IB16 : 2 mots lus / 1 écrit) -> on montre IN ;
        # module de sorties (OB16 : 1 mot lu inutilisé / 2 écrits) -> on montre OUT
        if mi["in_len"] and mi["in_len"] >= mo["out_len"]:
            d = inp[mi["in_off"]:mi["in_off"] + 2]          # 1er mot = données
            on = set_bits(d, True)
            lines.append("slot %d IN  : %-14s |%s|" % (s, ",".join(map(str, on)) or "-", ruler(d, True)))
        if mo["out_len"] and mo["out_len"] >= mi["in_len"]:
            d = out[mo["out_off"]:mo["out_off"] + 2]
            on = set_bits(d, True)
            lines.append("slot %d OUT : %-14s |%s|" % (s, ",".join(map(str, on)) or "-", ruler(d, True)))
    if len(inp) >= 2 and mods and mods[0]["id"] == 0x50 and (inp[0] or inp[1]):
        notes = ["adresse modifiée"] if inp[1] & 1 else []
        notes += ["défaut slot %d" % (b + 1) for b in range(8) if inp[0] & (1 << b)]
        lines.append("APB : " + ", ".join(notes))
    return lines


def decode_inputs(inp, mods):
    parts = []
    for m in mods:
        if m["in_len"]:
            data = inp[m["in_off"]:m["in_off"] + m["in_len"]]
            on = set_bits(data, m["word"])
            parts.append("#%d=%s%s" % (m["idx"], hx(data),
                                       " [bits %s]" % ",".join(map(str, on)) if on else ""))
    line = "IN  " + hx(inp) + "   " + "  ".join(parts)
    if len(inp) >= 2 and mods and mods[0]["id"] == 0x50:   # mot de statut 1794-APB
        hi, lo = inp[0], inp[1]
        notes = []
        if lo & 1:
            notes.append("adresse modifiée")
        for s in range(8):
            if hi & (1 << s):
                notes.append("défaut slot %d" % (s + 1))
        if notes:
            line += "   [APB: " + ", ".join(notes) + "]"
    return line


# ---------------------------------------------------------------------------
# Commandes
# ---------------------------------------------------------------------------
def cmd_scan(m, args):
    lo, hi = args.range
    print("Scan FDL status %d..%d à %d bauds (maître = %d)…" % (lo, hi, args.baud, args.master))
    found = 0
    for a in range(lo, hi + 1):
        if a == args.master:
            continue
        fc = m.fdl_status(a)
        if fc is not None:
            typ = {0: "esclave", 1: "maître (pas prêt)", 2: "maître (prêt)", 3: "maître (dans l'anneau)"}
            print("  adresse %3d : %s (FC=0x%02X)" % (a, typ.get((fc >> 4) & 3, "?"), fc))
            found += 1
    print("%d station(s) trouvée(s)." % found)


def inspect_slave(m, sl):
    """Tout ce qu'on peut lire d'un esclave sans le paramétrer."""
    print("=== Esclave %d ===" % sl)
    ident = m.request_ident(sl)
    if ident:
        print("FDL Ident :", " | ".join(ident))
    try:
        d = m.slave_diag(sl)
    except PBError as e:
        print("Slave_Diag impossible :", e)
        return
    decode_diag(d)
    s1, s2, master = d[0], d[1], d[3]
    print("État :")
    if master == 0xFF:
        print("  - non paramétré : aucun maître ne le contrôle (Master_Add = 0xFF)")
    else:
        print("  - paramétré / verrouillé par le maître d'adresse %d%s"
              % (master, " (c'est nous)" if master == m.addr else ""))
    if s2 & 0x01:
        print("  - attend un Set_Prm (Prm_Req)")
    if s1 & 0x40:
        print("  - Prm_Fault : le dernier Set_Prm a été refusé (Ident_Number ? user_prm ?)")
    if s1 & 0x04:
        print("  - Cfg_Fault : le dernier Chk_Cfg a été refusé")
    if s1 & 0x02:
        print("  - Station_Not_Ready")
    print("  - watchdog esclave : %s" % ("actif" if s2 & 0x08 else "inactif"))
    if not (s1 & 0x47) and not (s2 & 0x01):
        print("  - prêt pour l'échange de données")
    try:
        cfg = m.get_cfg(sl)
    except PBError as e:
        print("Get_Cfg impossible :", e)
        return
    print("Get_Cfg :", hx(cfg))
    try:
        mods, nin, nout = parse_cfg(cfg)
    except PBError as e:
        print("  ", e)
        return
    describe_layout(mods, nin, nout)
    if len(mods) >= 2 and mods[0]["id"] == 0x50 and mods[1]["id"] == 0x60:
        print("Slots Flex I/O (d'après les tailles maxi renvoyées) :")
        for s in range(1, (len(mods) + 1) // 2):
            mi, mo = mods[2 * s], mods[2 * s + 1] if 2 * s + 1 < len(mods) else None
            words = (mi["in_len"] // 2, mo["out_len"] // 2 if mo else 0)
            if words == (0, 0):
                print("  slot %d : vide" % s)
            else:
                print("  slot %d : %d mot(s) lu(s) / %d écrit(s) -> %s"
                      % (s, words[0], words[1], FLEX_GUESS.get(words, "module inconnu")))


def cmd_probe(m, args):
    lo, hi = args.range
    bauds = args.bauds or PROBE_BAUDS
    print("Balayage adresses %d..%d sur %s" % (lo, hi, ", ".join(str(b) for b in bauds)))
    found = []
    for baud in bauds:
        try:
            m.set_baud(baud)
        except (ValueError, serial.SerialException) as e:
            print("%8d bauds : non supporté par le port (%s)" % (baud, e))
            continue
        hits = [a for a in range(lo, hi + 1) if a != args.master and m.fdl_status(a) is not None]
        extra = "" if not m.noise else "  (%d octet(s) de bruit)" % m.noise
        print("%8d bauds : %s%s" % (baud, ", ".join(str(a) for a in hits) or "-", extra))
        if hits:
            found.append((baud, hits))
            if not args.all_bauds:
                break
    print("Écho local de nos trames : %s" % ("oui (adaptateur half-duplex)" if m.echo_seen else "non"))
    if not found:
        print("Aucune station. À vérifier : transceiver RS-485, polarité A/B (pin 3 = B+, "
              "pin 8 = A-), terminaison/polarisation, 24 V sur l'adaptateur, roues codeuses. "
              "Un bruit >0 avec zéro réponse valide suggère une polarité ou vitesse fausse.")
        return
    baud, hits = found[0]
    m.set_baud(baud)
    print("Vitesse retenue : %d bauds (utilisez --baud %d pour diag/run)" % (baud, baud))
    for a in hits:
        inspect_slave(m, a)


def cmd_diag(m, args):
    inspect_slave(m, args.slave)


def cmd_identify(m, args):
    """Identifie les cartes Flex I/O d'un 1794-APB en testant les clés module (format condensé)."""
    sl = args.slave
    d = m.slave_diag(sl)
    ident = args.ident if args.ident is not None else (d[4] << 8) | d[5]
    cfg = bytes.fromhex(args.cfg) if args.cfg else m.get_cfg(sl)
    mods, nin, nout = parse_cfg(cfg)
    slots = apb_slots(mods)
    if not slots:
        raise PBError("pas un 1794-APB ou aucun slot occupé (config : %s)" % hx(cfg))
    occupied = [s for s, _, _ in slots]
    print("Esclave %d, ident 0x%04X, slots occupés : %s" % (sl, ident, occupied))
    print("Test des %d clés module connues (quelques secondes)…" % len(FLEX_KEYS))
    found = {}
    try:
        for key, name in FLEX_KEYS:
            todo = [s for s in occupied if s not in found]
            if not todo:
                break
            keys = [key if s in todo else 0xFFFF for s in range(1, 9)]
            prm = bytes([0x00, 0x02]) + b"".join(bytes([k >> 8, k & 0xFF]) for k in keys)
            m.set_prm(sl, ident, prm, wd_on=False)
            m.chk_cfg(sl, cfg)
            time.sleep(0.1)
            inp = None
            for _ in range(3):                      # laisse l'adaptateur vérifier les clés
                inp = m.data_exchange(sl, bytes(nout))
                time.sleep(0.05)
            fault_hi = inp[0] if inp and len(inp) >= 2 else 0xFF
            for s in todo:
                if not fault_hi & (1 << (s - 1)):
                    found[s] = (key, name)
                    print("  slot %d : %s (clé %04X)" % (s, name, key))
    finally:
        # retour au mode auto-configure, sorties à zéro
        try:
            m.set_prm(sl, ident, bytes.fromhex(args.prm), wd_on=not args.no_wd)
            m.chk_cfg(sl, cfg)
            m.data_exchange(sl, bytes(nout))
        except PBError:
            pass
    print()
    print("Slot  Taille (lu/écrit)   Carte")
    for s, mi, mo in slots:
        size = "%d/%d mots" % (mi["in_len"] // 2, mo["out_len"] // 2)
        if s in found:
            print("  %d   %-18s  %s" % (s, size, found[s][1]))
        else:
            guess = FLEX_GUESS.get((mi["in_len"] // 2, mo["out_len"] // 2), "inconnu")
            print("  %d   %-18s  non identifiée (d'après la taille : %s)" % (s, size, guess))
    empty = [s for s in range(1, len(mods) // 2) if s not in occupied]
    if empty:
        print("  vides : %s" % ", ".join(map(str, empty)))
    print("(une carte insérée après la mise sous tension n'apparaît qu'après un cycle d'alimentation de l'APB)")


def startup(m, args):
    """Diag -> Set_Prm -> Chk_Cfg -> Diag. Retourne (mods, nin, nout, cfg)."""
    sl = args.slave
    print("--- Slave_Diag")
    ident = decode_diag(m.slave_diag(sl))
    if args.ident is not None:
        ident = args.ident
    cfg = bytes.fromhex(args.cfg) if args.cfg else m.get_cfg(sl)
    print("--- Config utilisée :", hx(cfg))
    mods, nin, nout = parse_cfg(cfg)
    describe_layout(mods, nin, nout)
    user_prm = bytes.fromhex(args.prm)
    print("--- Set_Prm (ident 0x%04X, user_prm %s, watchdog %s)"
          % (ident, hx(user_prm), "off" if args.no_wd else "%.1f s" % (args.wd * 10 / 1000)))
    wd = max(1, args.wd)
    m.set_prm(sl, ident, user_prm, wd_on=not args.no_wd, wd1=10, wd2=wd // 10 or 1)
    print("--- Chk_Cfg")
    m.chk_cfg(sl, cfg)
    time.sleep(0.05)
    print("--- Slave_Diag")
    d = m.slave_diag(sl)
    decode_diag(d)
    if d[0] & 0x46 or d[1] & 0x01:
        raise PBError("esclave pas prêt pour l'échange de données (voir status1/status2 ci-dessus)")
    print("--- Esclave %d prêt, échange de données." % sl)
    return mods, nin, nout, cfg


def set_bit(out, mod, bit, val):
    if bit >= mod["out_len"] * 8:
        raise ValueError("bit %d hors du module (%d bits)" % (bit, mod["out_len"] * 8))
    if mod["word"]:                       # mot PROFIBUS : octet de poids fort en premier
        byte = mod["out_off"] + 2 * (bit // 16) + (0 if (bit % 16) >= 8 else 1)
    else:
        byte = mod["out_off"] + bit // 8
    mask = 1 << (bit % 8)
    out[byte] = (out[byte] | mask) if val else (out[byte] & ~mask & 0xFF)


def out_module(mods, ref):
    """'5' -> module logique #5 ; 's2' -> module de sortie du slot 2 (APB)."""
    if ref.lower().startswith("s"):
        for s, mi, mo in apb_slots(mods):
            if s == int(ref[1:]):
                if not mo["out_len"]:
                    raise ValueError("le slot %d n'a pas de sorties" % s)
                return mo
        raise ValueError("slot %s inconnu ou vide" % ref[1:])
    mod = mods[int(ref)]
    if not mod["out_len"]:
        raise ValueError("le module #%d n'a pas de sorties" % mod["idx"])
    return mod


def xmas_frame(step, nbits):
    """Bits allumés à l'étape `step` d'une guirlande sur nbits points (4 tableaux de 6 s)."""
    phase, i = (step // 40) % 4, step % 40
    if phase == 0:                                   # chenillard à 2 points
        return {i % nbits, (i + nbits // 2) % nbits}
    if phase == 1:                                   # alterné pair / impair
        return {b for b in range(nbits) if b % 2 == (i // 3) % 2}
    if phase == 2:                                   # remplissage puis vidage
        k = i * nbits // 20
        return set(range(k)) if i < 20 else set(range((i - 20) * nbits // 20, nbits))
    return set(random.sample(range(nbits), max(1, nbits // 4)))   # scintillement


def parse_duration(txt):
    """'500ms', '1s', '2m', '1h', '0.2' (secondes) -> secondes (float)."""
    t = txt.strip().lower()
    for suf, mult in (("ms", 0.001), ("s", 1.0), ("m", 60.0), ("h", 3600.0)):
        if t.endswith(suf):
            return float(t[:-len(suf)]) * mult
    return float(t)


def xmas_apply(out, anim):
    """Écrit l'image courante de la guirlande dans le buffer de sortie."""
    nbits = 16 * len(anim["mods"])
    on = xmas_frame(anim["step"], nbits)
    for k, mod in enumerate(anim["mods"]):
        out[mod["out_off"]:mod["out_off"] + mod["out_len"]] = bytes(mod["out_len"])
        for b in range(16):
            if 16 * k + b in on:
                set_bit(out, mod, b, 1)
    anim["step"] += 1


def apply_command(line, out, mods, m, sl, state=None):
    """Retourne False pour quitter."""
    state = state if state is not None else {}
    p = line.split()
    if not p:
        return True
    c = p[0].lower()
    try:
        if c in ("q", "quit", "exit"):
            return False
        if c in ("chrismas", "christmas", "xmas", "noel", "noël"):
            targets = [out_module(mods, r) for r in p[1].split(",")]
            period = parse_duration(p[2]) if len(p) > 2 else 0.15
            duration = parse_duration(p[3]) if len(p) > 3 else None
            if period < 0.05:
                raise ValueError("période mini 50 ms (le bus tourne à ~10 images/s)")
            state["anim"] = {"mods": targets, "step": 0, "t": 0.0, "period": period,
                             "end": time.monotonic() + duration if duration else None,
                             "slots": [s for s, _, mo in apb_slots(mods) if mo in targets]}
            print("guirlande sur %s, un pas toutes les %gs%s — `stop` pour arrêter"
                  % (p[1], period, ", pendant %gs" % duration if duration else ""))
            return True
        if c == "stop":
            if state.pop("anim", None):
                out[:] = bytes(len(out))
                print("guirlande arrêtée, sorties à 0")
            return True
        if c in ("b", "w", "on", "off", "o"):
            state.pop("anim", None)
        if c == "?":
            print(__doc__.split("commandes au clavier")[1].split("Couche physique")[0])
        elif c == "b":
            set_bit(out, out_module(mods, p[1]), int(p[2]), int(p[3]))
        elif c == "w":
            mod, data = out_module(mods, p[1]), bytes.fromhex(p[2])
            if len(data) != mod["out_len"]:
                raise ValueError("le module attend %d octet(s)" % mod["out_len"])
            out[mod["out_off"]:mod["out_off"] + mod["out_len"]] = data
        elif c in ("on", "off"):
            mod = out_module(mods, p[1])
            out[mod["out_off"]:mod["out_off"] + mod["out_len"]] = \
                bytes([0xFF if c == "on" else 0x00]) * mod["out_len"]
        elif c == "o":
            data = bytes.fromhex(p[1])
            if len(data) != len(out):
                raise ValueError("buffer de sortie = %d octet(s)" % len(out))
            out[:] = data
        elif c == "d":
            decode_diag(m.slave_diag(sl))
        elif c == "i":
            print("(affiché au prochain cycle)")
            return "show"
        else:
            print("commande inconnue (? pour l'aide)")
        if c in ("b", "w", "on", "off", "o"):
            print("OUT", hx(out))
            for ln in friendly_io(bytes(len(out)), out, mods):
                if "OUT" in ln:
                    print("   ", ln)
    except (IndexError, ValueError) as e:
        print("erreur :", e)
    return True


# ---------------------------------------------------------------------------
# API HTTP : GET /in -> {"E1.0":1,...}, POST /out {"S2.3":1,...}
# ---------------------------------------------------------------------------
class IOMap:
    """Nommage des points : E<slot>.<bit> = entrée, S<slot>.<bit> = sortie (APB)."""

    def __init__(self, mods, prefix=""):
        self.prefix = prefix
        self.inputs, self.outputs = {}, {}        # slot -> module logique
        for s, mi, mo in apb_slots(mods):
            if mi["in_len"] and mi["in_len"] >= mo["out_len"]:
                self.inputs[s] = mi
            if mo["out_len"] and mo["out_len"] >= mi["in_len"]:
                self.outputs[s] = mo

    def cards(self):
        return {"%sE%d" % (self.prefix, s): "entrées (16 bits)" for s in sorted(self.inputs)} | \
               {"%sS%d" % (self.prefix, s): "sorties (16 bits)" for s in sorted(self.outputs)}

    def parse_key(self, key, want):
        """'S2.03' -> (slot, bit) ; want = 'S' ou 'E'. ValueError sinon."""
        k = key
        if self.prefix:
            if not k.startswith(self.prefix):
                raise ValueError("clé %r : préfixe %r attendu" % (key, self.prefix))
            k = k[len(self.prefix):]
        mo = re.fullmatch(r"([ES])(\d+)\.(\d+)", k)
        if not mo:
            raise ValueError("clé %r : format attendu %s<slot>.<bit>" % (key, self.prefix + want))
        typ, slot, bit = mo.group(1), int(mo.group(2)), int(mo.group(3))
        if typ != want:
            raise ValueError("clé %r : %s" % (key, "les entrées ne s'écrivent pas (E)" if typ == "E"
                                              else "S désigne une sortie, pas une entrée"))
        table = self.outputs if want == "S" else self.inputs
        if slot not in table:
            other = "sortie" if slot in self.outputs else "entrée" if slot in self.inputs else None
            if other:
                raise ValueError("clé %r : la carte %d est une carte de %s uniquement" % (key, slot, other))
            raise ValueError("clé %r : pas de carte en slot %d" % (key, slot))
        if not 0 <= bit <= 15:
            raise ValueError("clé %r : bit %d hors de 0..15" % (key, bit))
        return slot, bit

    def read_inputs(self, inp):
        res = {}
        for s, mi in sorted(self.inputs.items()):
            on = set(set_bits(inp[mi["in_off"]:mi["in_off"] + 2], True))
            for b in range(16):
                res["%sE%d.%d" % (self.prefix, s, b)] = 1 if b in on else 0
        return res

    def read_outputs(self, out):
        res = {}
        for s, mo in sorted(self.outputs.items()):
            on = set(set_bits(out[mo["out_off"]:mo["out_off"] + 2], True))
            for b in range(16):
                res["%sS%d.%d" % (self.prefix, s, b)] = 1 if b in on else 0
        return res

    def write_outputs(self, out, data):
        """Valide tout, puis applique (atomique). Retourne le nombre de bits écrits."""
        if not isinstance(data, dict):
            raise ValueError("un objet JSON {clé: 0|1} est attendu")
        todo = []
        for key, val in data.items():
            slot, bit = self.parse_key(str(key), "S")
            if val in (0, 1, True, False, "0", "1"):
                v = int(val)
            elif isinstance(val, str) and val.lower() in ("on", "off", "true", "false"):
                v = 1 if val.lower() in ("on", "true") else 0
            else:
                raise ValueError("clé %r : valeur %r invalide (0/1)" % (key, val))
            todo.append((self.outputs[slot], bit, v))
        for mod, bit, v in todo:
            set_bit(out, mod, bit, v)
        return len(todo)


HTML_PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8"><title>pbmaster — Flex I/O</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 :root{--bg:#f5f5f2;--fg:#222;--card:#fff;--line:#d8d8d2;--on:#2e9e44;--off:#c9c9c2;--in:#2d6cdf}
 @media(prefers-color-scheme:dark){:root{--bg:#1b1b1d;--fg:#e6e6e2;--card:#242427;--line:#3a3a3e;--off:#4a4a4f}}
 body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,sans-serif}
 h1{font-size:18px;margin:0 0 4px} #status{color:#888;font-size:12px;margin-bottom:14px}
 .card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px;margin-bottom:12px}
 .card h2{font-size:15px;margin:0 0 8px} .card h2 small{color:#888;font-weight:normal;margin-left:8px}
 table{border-collapse:collapse;width:100%} td{text-align:center;padding:4px 2px;width:6.25%}
 td.n{color:#888;font-size:11px}
 .led{display:inline-block;width:18px;height:18px;border-radius:50%;background:var(--off);border:1px solid var(--line)}
 .led.on{background:var(--in);box-shadow:0 0 6px var(--in)}
 input[type=checkbox]{width:20px;height:20px;accent-color:var(--on);cursor:pointer}
 .err{color:#c33} button{margin-right:6px;padding:3px 10px;cursor:pointer}
 #xmas{display:flex;flex-wrap:wrap;gap:8px 14px;align-items:center}
 #xmas input[type=text]{width:64px;padding:3px 6px} #xmas label{white-space:nowrap}
 #xmas .go{background:#2e9e44;color:#fff;border:0;border-radius:4px;padding:5px 14px;font-weight:600}
 #xmas .stop{background:#c33;color:#fff;border:0;border-radius:4px;padding:5px 14px}
 #xmasinfo{color:#888;font-size:12px}
</style></head><body>
<h1>Flex I/O — 1794-APB</h1><div id="status">connexion…</div>
<div class=card id="xmas">
  <strong>🎄 Xmas</strong>
  <span id="xmascards"></span>
  <label>pas <input type=text id="xstep" value="50ms"></label>
  <label>durée <input type=text id="xdur" value="2m"></label>
  <button class=go onclick="xmasStart()">Démarrer</button>
  <button class=stop onclick="xmasStop()">Stop</button>
  <span id="xmasinfo"></span>
</div>
<div id="cards"></div>
<script>
const $=s=>document.querySelector(s); let built=false, pending=new Set();
function row(label, cells){return `<tr><td class=n>${label}</td>${cells}</tr>`}
function build(cards){
  $('#xmascards').innerHTML = Object.keys(cards).filter(id=>/S\d+$/.test(id))
    .map(id=>`<label><input type=checkbox class=xc value="${id}" checked> ${id}</label>`).join(' ');
  $('#cards').innerHTML = Object.entries(cards).map(([id,desc])=>{
    const isIn = id.replace(/^.*?([ES])(\d+)$/,'$1')==='E';
    const nums = Array.from({length:16},(_,b)=>`<td class=n>${b}</td>`).join('');
    const cells = Array.from({length:16},(_,b)=> isIn
      ? `<td><span class=led id="${id}.${b}"></span></td>`
      : `<td><input type=checkbox id="${id}.${b}" onchange="setOut('${id}.${b}',this.checked)"></td>`).join('');
    const btns = isIn ? '' : `<small><button onclick="setAll('${id}',1)">tout à 1</button><button onclick="setAll('${id}',0)">tout à 0</button></small>`;
    return `<div class=card><h2>${id} <small>${desc}</small>${btns}</h2><table>${row('bit',nums)}${row(isIn?'entrée':'sortie',cells)}</table></div>`;
  }).join(''); built=true;
}
async function post(data){
  pending = new Set(Object.keys(data));
  const r = await fetch('/out',{method:'POST',body:JSON.stringify(data)});
  const j = await r.json(); pending.clear();
  if(!r.ok){ $('#status').innerHTML='<span class=err>'+(j.error||r.status)+'</span>'; }
  return j;
}
function setOut(k,v){ post({[k]:v?1:0}); }
async function xmasStart(){
  const cards=[...document.querySelectorAll('.xc:checked')].map(e=>e.value);
  if(!cards.length){ $('#xmasinfo').innerHTML='<span class=err>choisir au moins une carte</span>'; return; }
  const r=await fetch('/xmas',{method:'POST',body:JSON.stringify({cards, step:$('#xstep').value, duration:$('#xdur').value})});
  const j=await r.json(); if(!r.ok){ $('#xmasinfo').innerHTML='<span class=err>'+j.error+'</span>'; errUntil=Date.now()+5000; }
}
let errUntil=0;
async function xmasStop(){ await fetch('/xmas/stop',{method:'POST'}); }
function fmt(s){ return s>=60 ? Math.floor(s/60)+'m'+String(s%60).padStart(2,'0')+'s' : s+'s'; }
function setAll(id,v){ const d={}; for(let b=0;b<16;b++) d[`${id}.${b}`]=v; post(d); }
async function refresh(){
  try{
    const r = await fetch('/io'); if(!r.ok) throw new Error('HTTP '+r.status);
    const j = await r.json(); if(!built) build(j.cards);
    for(const [k,v] of Object.entries(j.in||{})){ const e=document.getElementById(k); if(e) e.className='led'+(v?' on':''); }
    for(const [k,v] of Object.entries(j.out||{})){ const e=document.getElementById(k); if(e && !pending.has(k)) e.checked=!!v; }
    const x=j.xmas; if(Date.now()>errUntil) $('#xmasinfo').textContent = !x ? '' :
      'en cours sur '+x.cards.join(', ')+' — pas '+x.step_s+'s'+(x.remaining_s==null?' (sans fin)':' — reste '+fmt(x.remaining_s));
    $('#status').textContent = 'données de l’esclave : ' + (j.age_s==null?'aucune':'il y a '+j.age_s+' s') + ' — ' + new Date().toLocaleTimeString();
  }catch(e){ $('#status').innerHTML='<span class=err>'+e.message+'</span>'; }
}
refresh(); setInterval(refresh,500);
</script></body></html>
"""


def make_http_handler(shared):
    import base64
    import http.server
    import json

    user, _, pw = shared["auth"].partition(":")
    expected = "Basic " + base64.b64encode(("%s:%s" % (user, pw)).encode()).decode()

    class Handler(http.server.BaseHTTPRequestHandler):
        server_version = "pbmaster/1.0"

        def log_message(self, fmt, *a):
            if shared["verbose"]:
                print("%s http %s %s" % (time.strftime("%H:%M:%S"), self.address_string(), fmt % a))

        def _send(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False, indent=1).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _authed(self):
            if self.headers.get("Authorization") == expected:
                return True
            self.send_response(401)
            self.send_header("WWW-Authenticate", 'Basic realm="pbmaster"')
            self.send_header("Content-Length", "0")
            self.end_headers()
            return False

        def do_GET(self):
            if not self._authed():
                return
            path = self.path.split("?")[0].rstrip("/") or "/"
            if path in ("/", "/index.html"):
                body = HTML_PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            iomap = shared["iomap"]
            with shared["lock"]:
                inp, out, inp_t = shared["inp"], bytes(shared["out"]), shared["inp_t"]
            if path in ("/in", "/int", "/inputs"):
                if inp is None:
                    return self._send(503, {"error": "pas encore de données de l'esclave"})
                return self._send(200, iomap.read_inputs(inp))
            if path in ("/out", "/outputs"):
                return self._send(200, iomap.read_outputs(out))
            if path == "/io":
                return self._send(200, {"cards": iomap.cards(),
                                        "in": iomap.read_inputs(inp) if inp else None,
                                        "out": iomap.read_outputs(out),
                                        "age_s": None if inp_t is None else round(time.monotonic() - inp_t, 2),
                                        "xmas": shared.get("anim_info")})
            if path == "/cards":
                return self._send(200, iomap.cards())
            self._send(404, {"error": "routes : GET / (page), GET /in, GET /out, GET /io, GET /cards, POST /out"})

        def do_POST(self):
            if not self._authed():
                return
            path = self.path.split("?")[0].rstrip("/")
            if path in ("/xmas", "/chrismas", "/christmas"):
                return self._xmas()
            if path in ("/xmas/stop", "/stop"):
                with shared["lock"]:
                    shared["anim_stop"] = True
                return self._send(200, {"stopped": True})
            if path not in ("/out", "/outputs"):
                return self._send(404, {"error": "POST /out, POST /xmas, POST /xmas/stop"})
            try:
                n = int(self.headers.get("Content-Length") or 0)
                data = json.loads(self.rfile.read(n) or b"{}")
            except ValueError as e:
                return self._send(400, {"error": "JSON invalide : %s" % e})
            iomap = shared["iomap"]
            with shared["lock"]:
                try:
                    count = iomap.write_outputs(shared["out"], data)
                except ValueError as e:
                    return self._send(400, {"error": str(e)})
                shared["anim_stop"] = True
                res = iomap.read_outputs(shared["out"])
            if shared["verbose"]:
                print("%s http: %d sortie(s) écrite(s) %s" % (time.strftime("%H:%M:%S"), count, data))
            self._send(200, {"written": count, "out": res})

        def _xmas(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                data = json.loads(self.rfile.read(n) or b"{}")
            except ValueError as e:
                return self._send(400, {"error": "JSON invalide : %s" % e})
            iomap = shared["iomap"]
            try:
                cards = data.get("cards") or ["%sS%d" % (iomap.prefix, s) for s in sorted(iomap.outputs)]
                slots = []
                for c in cards:
                    k = str(c)
                    if iomap.prefix and k.startswith(iomap.prefix):
                        k = k[len(iomap.prefix):]
                    mo = re.fullmatch(r"S(\d+)", k)
                    if not mo or int(mo.group(1)) not in iomap.outputs:
                        raise ValueError("carte %r : pas une carte de sorties" % c)
                    slots.append(int(mo.group(1)))
                period = parse_duration(str(data.get("step", "150ms")))
                duration = parse_duration(str(data["duration"])) if data.get("duration") else None
                if period < 0.05:
                    raise ValueError("pas minimum 50ms")
                if duration is not None and duration <= 0:
                    raise ValueError("durée invalide")
            except (ValueError, KeyError, TypeError) as e:
                return self._send(400, {"error": str(e)})
            with shared["lock"]:
                shared["xmas_req"] = {"slots": slots, "period": period, "duration": duration}
            if shared["verbose"]:
                print("%s http: guirlande sur %s, pas %gs, durée %s" % (time.strftime("%H:%M:%S"), slots, period, duration))
            self._send(200, {"started": True, "cards": ["%sS%d" % (iomap.prefix, s) for s in slots],
                             "step_s": period, "duration_s": duration})

    return Handler


def start_httpd(shared, bind):
    import http.server
    host, _, port = bind.rpartition(":")
    srv = http.server.ThreadingHTTPServer((host or "0.0.0.0", int(port or 8080)),
                                          make_http_handler(shared))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def cmd_httpd(m, args):
    """Daemon : échange DP + API HTTP, sans clavier (pour systemd)."""
    account = args.account or os.environ.get("PBMASTER_ACCOUNT")
    if not account or ":" not in account:
        raise PBError("httpd : --account user:password requis (ou variable PBMASTER_ACCOUNT)")
    args.account = account
    def on_term(*_):
        raise KeyboardInterrupt          # -> le finally de cmd_run remet les sorties à 0
    signal.signal(signal.SIGTERM, on_term)
    cmd_run(m, args, interactive=False)


def cmd_run(m, args, interactive=True):
    sl = args.slave
    mods, nin, nout, cfg = startup(m, args)
    out = bytearray(nout)
    if args.out:
        data = bytes.fromhex(args.out)
        if len(data) != nout:
            raise PBError("--out doit faire %d octet(s)" % nout)
        out[:] = data
    print("OUT", hx(out), "   (tapez ? pour l'aide)" if interactive else "")
    if apb_slots(mods) and interactive:
        print("Réglette : bit 0 à gauche, bit 15 à droite ->  |0123456789012345|")

    shared = {"lock": threading.Lock(), "out": out, "inp": None, "inp_t": None,
              "iomap": IOMap(mods, args.json_key_prefix), "auth": args.account,
              "verbose": not args.http_quiet, "anim_stop": False, "xmas_req": None, "anim_info": None}
    if args.account:
        if ":" not in args.account:
            raise PBError("--account attend user:password")
        if not shared["iomap"].inputs and not shared["iomap"].outputs:
            raise PBError("HTTP : pas un 1794-APB (aucun slot E/S trouvé)")
        srv = start_httpd(shared, args.http_bind)
        print("HTTP sur http://%s:%d  (auth basic, utilisateur %r) — cartes : %s"
              % (srv.server_address[0], srv.server_address[1], args.account.split(":")[0],
                 ", ".join(shared["iomap"].cards())))

    q = queue.Queue()
    if interactive:
        def reader():
            for line in sys.stdin:
                q.put(line.rstrip("\n"))
            q.put("q")
        threading.Thread(target=reader, daemon=True).start()

    last_in, errors, show, state = None, 0, False, {}
    lock = shared["lock"]
    try:
        while True:
            with lock:
                while not q.empty():
                    r = apply_command(q.get(), out, mods, m, sl, state)
                    if r is False:
                        raise KeyboardInterrupt
                    if r == "show":
                        show = True
                if shared["anim_stop"]:                 # un POST /out ou /xmas/stop coupe la guirlande
                    shared["anim_stop"] = False
                    if state.pop("anim", None):
                        out[:] = bytes(len(out))
                req = shared.pop("xmas_req", None)
                if req:
                    shared["xmas_req"] = None
                    targets = [mods[2 * s + 1] for s in req["slots"]]
                    state["anim"] = {"mods": targets, "step": 0, "t": 0.0, "period": req["period"],
                                     "end": time.monotonic() + req["duration"] if req["duration"] else None,
                                     "slots": req["slots"]}
                anim = state.get("anim")
                shared["anim_info"] = None if not anim else {
                    "cards": ["%sS%d" % (shared["iomap"].prefix, s) for s in anim.get("slots", [])],
                    "step_s": anim["period"],
                    "remaining_s": None if not anim["end"] else max(0, round(anim["end"] - time.monotonic()))}
                if anim and anim["end"] and time.monotonic() >= anim["end"]:
                    state.pop("anim")
                    out[:] = bytes(len(out))
                    print("%s guirlande terminée, sorties à 0" % time.strftime("%H:%M:%S"))
                elif anim and time.monotonic() - anim["t"] >= anim["period"]:
                    anim["t"] = time.monotonic()
                    xmas_apply(out, anim)
                snapshot = bytes(out)
            try:
                inp = m.data_exchange(sl, snapshot)
                errors = 0
                with lock:
                    shared["inp"], shared["inp_t"] = inp, time.monotonic()
            except PBError as e:
                errors += 1
                print("%s  erreur échange : %s" % (time.strftime("%H:%M:%S"), e))
                if errors >= 5:
                    print("--- redémarrage de l'esclave")
                    time.sleep(0.5)
                    try:
                        startup(m, args)
                        errors = 0
                    except PBError as e2:
                        print("échec :", e2)
                time.sleep(0.2)
                continue
            if (inp != last_in or show) and (interactive or args.debug):
                ts = time.strftime("%H:%M:%S")
                lines = [ln for ln in friendly_io(inp, out, mods) if "OUT" not in ln]
                if lines:
                    print(ts, lines[0])
                    for ln in lines[1:]:
                        print(" " * len(ts), ln)
                    if args.debug or show:
                        print(" " * len(ts), decode_inputs(inp, mods))
                else:
                    print(ts, decode_inputs(inp, mods))
                last_in, show = inp, False
            time.sleep(args.cycle)
    except KeyboardInterrupt:
        pass
    finally:
        print("--- mise à zéro des sorties")
        for _ in range(3):
            try:
                m.data_exchange(sl, bytes(nout))
            except PBError:
                break
            time.sleep(0.02)


UDEV_PATH = "/etc/udev/rules.d/90-pbmaster.rules"
UDEV_RULE = 'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}", MODE="0660", GROUP="dialout"\n'
UNIT_PATH = "/etc/systemd/system/pbmaster.service"
ENV_PATH = "/etc/pbmaster.conf"
BIN_PATH = "/usr/local/bin/pbmaster.py"
SVC_USER = "pbmaster"

UNIT_TEMPLATE = """[Unit]
Description=pbmaster - maître PROFIBUS-DP + API HTTP (Flex I/O)
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
User={user}
SupplementaryGroups=dialout
EnvironmentFile={env}
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/env python3 {bin} httpd --port {port} --baud {baud} --slave {slave} \\
    --master {master} --prm {prm} --wd {wd} --http-bind {bind}{prefix}{extra}
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=10
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
DeviceAllow={port} rw

[Install]
WantedBy=multi-user.target
"""


def usb_ids(port):
    """(idVendor, idProduct) du périphérique USB derrière /dev/ttyUSBx, ou None."""
    try:
        dev = os.path.realpath("/sys/class/tty/%s/device" % os.path.basename(port))
        while dev and dev != "/":
            if os.path.exists(os.path.join(dev, "idVendor")):
                return (open(os.path.join(dev, "idVendor")).read().strip(),
                        open(os.path.join(dev, "idProduct")).read().strip())
            dev = os.path.dirname(dev)
    except OSError:
        pass
    return None


def port_access_ok(port, user):
    """Teste une ouverture lecture/écriture du port sous l'identité du service."""
    import subprocess
    r = subprocess.run(["runuser", "-u", user, "--", "python3", "-c",
                        "import sys; open(sys.argv[1], 'r+b').close()", port],
                       capture_output=True, text=True)
    return r.returncode == 0, r.stderr.strip().splitlines()[-1] if r.stderr.strip() else ""


def ensure_port_access(port, user):
    """Rend le port accessible au service : règle udev (groupe dialout) puis contrôle réel."""
    import subprocess
    import grp
    import stat
    if not os.path.exists(port):
        print("attention : %s absent pour l'instant (le service réessaiera toutes les 3 s)" % port)
        return
    st = os.stat(port)
    grp_name = grp.getgrgid(st.st_gid).gr_name
    mode = stat.S_IMODE(st.st_mode)
    print("port %s : groupe %s, mode %04o" % (port, grp_name, mode))
    ok, err = port_access_ok(port, user)
    if ok:
        print("accès au port OK pour l'utilisateur %r" % user)
        return
    print("l'utilisateur %r ne peut pas ouvrir %s (%s) -> règle udev" % (user, port, err))
    ids = usb_ids(port)
    if ids:
        with open(UDEV_PATH, "w") as f:
            f.write(UDEV_RULE.format(vid=ids[0], pid=ids[1]))
        print("règle udev écrite : %s (USB %s:%s -> dialout, 0660)" % (UDEV_PATH, ids[0], ids[1]))
        subprocess.call(["udevadm", "control", "--reload-rules"])
        subprocess.call(["udevadm", "trigger", "--subsystem-match=tty"])
    else:
        os.chown(port, -1, grp.getgrnam("dialout").gr_gid)
        os.chmod(port, 0o660)
        print("groupe/mode corrigés directement (pas d'infos USB pour une règle udev)")
    time.sleep(0.5)
    ok, err = port_access_ok(port, user)
    if ok:
        print("accès au port OK pour l'utilisateur %r" % user)
    else:
        print("ÉCHEC : %s toujours inaccessible (%s). Repli : relancer install avec --service-user root"
              % (port, err))


def cmd_install(args):
    import pwd
    import shutil
    import subprocess
    if os.geteuid() != 0:
        sys.exit("install : à lancer en root (sudo)")
    if not args.account or ":" not in args.account:
        sys.exit("install : --account user:password requis")
    user = args.service_user
    if user == SVC_USER:
        try:
            pwd.getpwnam(SVC_USER)
        except KeyError:
            subprocess.check_call(["useradd", "--system", "--no-create-home", "--shell", "/usr/sbin/nologin",
                                   "-G", "dialout", SVC_USER])
            print("utilisateur système %r créé (groupe dialout)" % SVC_USER)
    else:
        pwd.getpwnam(user)        # KeyError si inexistant
    ensure_port_access(args.port, user)
    try:
        import serial  # noqa: F401  (doit être installé système : apt install python3-serial)
    except ImportError:
        print("attention : pyserial introuvable pour python3 système -> apt install python3-serial")
    src = os.path.abspath(__file__)
    if src != BIN_PATH:
        shutil.copyfile(src, BIN_PATH)
        os.chmod(BIN_PATH, 0o755)
    with open(ENV_PATH, "w") as f:
        f.write("# identifiants HTTP de pbmaster (auth basic) — lu par systemd\n")
        f.write("PBMASTER_ACCOUNT=%s\n" % args.account)
    os.chmod(ENV_PATH, 0o600)
    extra = ""
    if args.no_wd:
        extra += " --no-wd"
    if args.http_quiet:
        extra += " --http-quiet"
    if args.cfg:
        extra += " --cfg %s" % args.cfg
    if args.ident is not None:
        extra += " --ident 0x%04X" % args.ident
    if args.rts:
        extra += " --rts"
    unit = UNIT_TEMPLATE.format(user=user, env=ENV_PATH, bin=BIN_PATH, port=args.port, baud=args.baud,
                                slave=args.slave, master=args.master, prm=args.prm, wd=args.wd,
                                bind=args.http_bind, extra=extra,
                                prefix=" --json-key-prefix %s" % args.json_key_prefix if args.json_key_prefix else "")
    with open(UNIT_PATH, "w") as f:
        f.write(unit)
    subprocess.check_call(["systemctl", "daemon-reload"])
    subprocess.check_call(["systemctl", "enable", "--now", "pbmaster.service"])
    subprocess.call(["systemctl", "restart", "pbmaster.service"])
    time.sleep(1.5)
    print("--- installé :")
    print("  service     : utilisateur %r" % user)
    print("  script      :", BIN_PATH)
    print("  unité       :", UNIT_PATH)
    print("  identifiants:", ENV_PATH, "(mode 600)")
    print("  API         : http://%s/  (utilisateur %r)" % (args.http_bind, args.account.split(":")[0]))
    print("  journal     : journalctl -u pbmaster -f")
    subprocess.call(["systemctl", "--no-pager", "--lines=5", "status", "pbmaster.service"])


def cmd_uninstall(args):
    import subprocess
    if os.geteuid() != 0:
        sys.exit("uninstall : à lancer en root (sudo)")
    subprocess.call(["systemctl", "disable", "--now", "pbmaster.service"])
    for p in (UNIT_PATH, ENV_PATH, BIN_PATH, UDEV_PATH):
        if os.path.exists(p):
            os.remove(p)
            print("supprimé :", p)
    subprocess.call(["systemctl", "daemon-reload"])
    print("(l'utilisateur système %r est conservé ; `userdel %s` pour le retirer)" % (SVC_USER, SVC_USER))


def main():
    ap = argparse.ArgumentParser(description="mini maître PROFIBUS-DP (pyserial)")
    ap.add_argument("cmd", choices=["probe", "scan", "diag", "identify", "run", "httpd", "install", "uninstall"])
    ap.add_argument("--bauds", type=int, nargs="+", default=None,
                    help="vitesses à essayer pour probe (défaut : %s)" % " ".join(map(str, PROBE_BAUDS)))
    ap.add_argument("--all-bauds", action="store_true",
                    help="probe : ne pas s'arrêter à la première vitesse qui répond")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600,
                    help="9600 ou 19200 conseillé depuis un PC (défaut 9600)")
    ap.add_argument("--master", type=int, default=1, help="adresse du maître (défaut 1)")
    ap.add_argument("--slave", type=int, default=5, help="adresse de l'esclave (roues codeuses)")
    ap.add_argument("--range", type=int, nargs=2, default=(0, 126), metavar=("MIN", "MAX"),
                    help="plage d'adresses pour scan")
    ap.add_argument("--ident", type=lambda s: int(s, 0), default=None,
                    help="forcer l'Ident_Number (défaut : lu dans le diag)")
    ap.add_argument("--cfg", default=None,
                    help="octets Chk_Cfg en hex (défaut : Get_Cfg de l'esclave). Ex: 50600060")
    ap.add_argument("--prm", default="0000",
                    help="user_prm_data en hex (défaut 0000 : octet ASIC + flags auto-configure, "
                         "validé sur 1794-APB/B ; un APB/A attend 00)")
    ap.add_argument("--out", default=None, help="sorties initiales en hex")
    ap.add_argument("--wd", type=int, default=300, help="watchdog en unités de 10 ms (défaut 300 = 3 s)")
    ap.add_argument("--no-wd", action="store_true", help="désactiver le watchdog esclave")
    ap.add_argument("--cycle", type=float, default=0.05, help="pause entre cycles (s)")
    ap.add_argument("--timeout", type=float, default=0.15, help="attente réponse (s)")
    ap.add_argument("--rts", action="store_true",
                    help="piloter la direction RS-485 par RTS (adaptateurs sans auto-direction)")
    ap.add_argument("--debug", action="store_true", help="afficher les trames TX/RX")
    ap.add_argument("--account", metavar="USER:PASS", default=None,
                    help="identifiants HTTP (auth basic) ; requis pour httpd/install, optionnel pour run")
    ap.add_argument("--http-bind", default="127.0.0.1:8080", metavar="HOST:PORT",
                    help="adresse d'écoute HTTP (défaut 127.0.0.1:8080)")
    ap.add_argument("--json-key-prefix", default="", metavar="P",
                    help="préfixe des clés JSON : avec P=F -> FE1.0 / FS2.3")
    ap.add_argument("--http-quiet", action="store_true", help="ne pas journaliser les requêtes HTTP")
    ap.add_argument("--service-user", default=SVC_USER, metavar="USER",
                    help="install : utilisateur du service systemd (défaut %s ; root en dernier recours)" % SVC_USER)
    args = ap.parse_args()

    if args.cmd == "install":
        return cmd_install(args)
    if args.cmd == "uninstall":
        return cmd_uninstall(args)
    if args.cmd == "httpd":
        args.account = args.account or os.environ.get("PBMASTER_ACCOUNT")
        if not args.account or ":" not in args.account:
            sys.exit("httpd : --account user:password requis (ou variable PBMASTER_ACCOUNT)")
    try:
        m = PBMaster(args.port, args.baud, args.master, args.timeout, args.debug, args.rts)
    except serial.SerialException as e:
        sys.exit("port série : %s" % e)
    try:
        {"probe": cmd_probe, "scan": cmd_scan, "diag": cmd_diag, "identify": cmd_identify,
         "run": cmd_run, "httpd": cmd_httpd}[args.cmd](m, args)
    except PBError as e:
        sys.exit("PROFIBUS : %s" % e)


if __name__ == "__main__":
    main()
