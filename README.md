# pbmaster

A minimal PROFIBUS-DP class 1 master (DP-V0) in a single Python file, for driving
Allen-Bradley **1794-APB** FLEX I/O adapters (and their OEM rebadges, e.g. Invensys
I/A Series / SPX-APV) from a Linux box with a cheap USB/RS-485 dongle.

Features: bus probe (baud rates, addresses, slave diagnostics), card identification,
interactive console, JSON HTTP API with a small web page, and a one-shot systemd
installer. Dependencies: Python 3 and [pyserial](https://pypi.org/project/pyserial/) only.

Tested with a 1794-APB/B (Ident 0x1010) + 1794-IB16 + 2× 1794-OB16, through an
Innomaker isolated USB-RS485 adapter (FTDI FT230X) at 9600 baud.

*Version française plus bas.*

---

## English

### Hardware

| Item | Notes |
|---|---|
| USB/RS-485 adapter | Must be a real RS-485 transceiver (not a plain TTL/RS-232 UART). Isolated FTDI-based adapters work well. |
| Wiring | PROFIBUS DB9 on the adapter: pin 3 = B (red), pin 8 = A (green), pin 5 = GND. On the dongle, **green → A**, **red → B**, cable shield → GND. If nothing answers, swap A/B. |
| Termination | Enable the terminating resistor on the PROFIBUS connector at the adapter end (red switch "On") and the 120 Ω jumper on the dongle. |
| Power | 24 V DC on the adapter **and** field power on the terminal bases of output modules, otherwise outputs stay dark even though the bus is happy. |
| Baud rate | 9600 (default) or 19200 from a PC. The APB auto-detects the rate at power-up: if you change it, power-cycle the adapter. |
| Node address | Thumbwheels on the adapter (01…99). |

Serial format is always 8E1; there is nothing to configure there.

### Install

```bash
sudo apt install python3-serial          # or: pip install pyserial
sudo usermod -aG dialout $USER           # then log out / in
```

### Commands

```
pbmaster.py probe                         # sweep baud rates + addresses, inspect every slave found
pbmaster.py scan      [--baud 9600]       # who answers at this rate?
pbmaster.py diag      --slave 3           # diagnostics, config, state of one slave
pbmaster.py identify  --slave 3           # exact FLEX I/O card reference per slot
pbmaster.py run       --slave 3           # cyclic data exchange + interactive console
pbmaster.py httpd     --slave 3 --account user:pass   # headless daemon with HTTP API
sudo pbmaster.py install  --slave 3 --account user:pass  # install + start the systemd service
sudo pbmaster.py uninstall
```

Common options: `--port /dev/ttyUSB0`, `--baud 9600`, `--master 1`, `--slave N`,
`--debug` (dump frames), `--rts` (drive RS-485 direction with RTS for dongles without
auto-direction), `--prm 0000` (user parameter bytes: `0000` works for APB series B,
series A expects `00`), `--cfg <hex>` (force the Chk_Cfg bytes), `--wd 300` (slave
watchdog in 10 ms units), `--no-wd`.

#### `probe`

Tries 9600 → 1.5 M baud, polls addresses 0..126 at each rate, then for every slave
found prints FDL ident, decoded diagnostics (parameterised by whom, `Prm_Req`,
`Prm_Fault`/`Cfg_Fault`, watchdog), the current config (`Get_Cfg`) and a guess of the
FLEX cards per slot. Also reports "noise" bytes (polarity/baud problems) and whether the
dongle echoes its own frames.

#### `identify`

Uses the APB "condensed" parameter format to test the known module keys (IB16, OB16,
IA8, OA8, IB8S, OW8, IE8, OE4, IE4XOE2, IT8, IR8, FM1) on every occupied slot and
prints the exact card reference. The adapter is put back in auto-configure afterwards.
A card inserted after power-up is only seen after power-cycling the APB.

#### `run` — interactive console

Inputs are printed whenever they change, per slot, with active bit numbers and a ruler
(bit 0 on the left):

```
18:34:18 slot 1 IN  : 5,10           |.....X....X.....|
```

Keyboard commands (`<mod>` = slot `s2` or logical module `#5`):

```
b <mod> <bit> <0|1>      set one output bit          (b s2 3 1)
w <mod> <hex>            write the module's bytes    (w s2 00FF0000)
on <mod> / off <mod>     all 16 outputs on / off
o <hex>                  replace the whole output buffer
chrismas <mod>[,<mod>] [step] [duration]
                         animated light chaser, e.g. chrismas s2,s3 1s 60s
stop                     stop the animation (outputs to 0)
i / d / ? / q            show inputs / re-read diag / help / quit
```

Ctrl-C or `q` zero the outputs before exiting. The slave watchdog (3 s by default)
zeroes them anyway if the process dies.

### HTTP API

```
pbmaster.py run   --slave 3 --account admin:secret        # console + API
pbmaster.py httpd --slave 3 --account admin:secret [--http-bind 127.0.0.1:8080] [--json-key-prefix F]
```

Basic auth on every route. Points are named `E<slot>.<bit>` (inputs) and
`S<slot>.<bit>` (outputs), slot = physical position of the card (1..8), bit = 0..15.
With one IB16 in slot 1 and OB16s in slots 2 and 3 you get `E1`, `S2`, `S3`.

| Route | Description |
|---|---|
| `GET /` | Web page: one table per card, LEDs for inputs, checkboxes for outputs, Xmas bar (cards, step, duration, start/stop, countdown), refreshed every 500 ms |
| `GET /in` | `{"E1.0":0, …, "E1.5":1, …}` |
| `GET /out` | current output state `{"S2.0":0, …}` |
| `POST /out` | body `{"S2.3":1, "S3.0":0}` — fully validated before anything is written |
| `GET /io` | cards + inputs + outputs + age of the last slave data |
| `GET /cards` | `{"E1":"inputs (16 bits)", "S2":"outputs (16 bits)", …}` |
| `POST /xmas` | body `{"cards":["S2","S3"], "step":"50ms", "duration":"2m"}` — start the light chaser (cards default to all output cards, duration optional) |
| `POST /xmas/stop` | stop it, outputs to 0 |

A `POST /out` is atomic: an unknown slot, an `E` key, a slot of the wrong type
(`S1` on an input card, `E2` on an output card), a bit > 15 or a value other than
0/1/true/false/on/off returns `400` and nothing is written. Leading zeros are accepted
(`S2.03`). `--json-key-prefix F` turns the keys into `FE1.5` / `FS2.3` everywhere.

```bash
curl -u admin:secret http://127.0.0.1:8080/in
curl -u admin:secret -X POST -d '{"S2.0":1,"S3.15":"on"}' http://127.0.0.1:8080/out
```

Plain HTTP with basic auth: keep it on localhost, behind Tailscale or a TLS reverse
proxy.

### systemd service

```bash
sudo python3 pbmaster.py install --slave 3 --account admin:secret [--http-bind 127.0.0.1:8080] [--json-key-prefix F]
```

`install` creates a system user `pbmaster` (group `dialout`), checks that this user
can actually open the serial port (adds a udev rule for the dongle if not; `--service-user
root` as a last resort), copies the script to `/usr/local/bin/pbmaster.py`, writes the
credentials to `/etc/pbmaster.conf` (mode 600, read via `EnvironmentFile`, so never
visible in `ps`), writes `/etc/systemd/system/pbmaster.service` (`Restart=always`,
hardened), then `daemon-reload` + `enable --now`. Re-run `install` to change a
parameter. Logs: `journalctl -u pbmaster -f`. `uninstall` removes all of it.

### Protocol notes

Pure DP-V0 over pyserial: `FDL Status` / `Request Ident`, `Slave_Diag` (SAP 60),
`Set_Prm` (61), `Chk_Cfg` (62), `Get_Cfg` (59), then cyclic `Data_Exchange` with
FCB/FCV handling, half-duplex echo suppression and noise counting. The Ident number is
read from the slave's diagnostics, so series A/B and OEM rebadges work unchanged.
Chk_Cfg defaults to what the slave reports in `Get_Cfg`. On a 1794-APB each slot uses
two logical modules (input, output); logical modules #0/#1 are the adapter status word.

Troubleshooting: no answer and zero noise → wrong wiring, no RS-485 transceiver or
adapter unpowered; exactly one noise byte per frame at every rate → A/B swapped;
`Prm_Fault` after `Set_Prm` → try `--prm 00` (series A) or `--prm 0000` (series B);
`défaut slot N` in the status word → card inserted after power-up, power-cycle the APB.

References: Rockwell 1794-APB user manual (publication 1794-6.5.6 / 1794-UM057),
PROFIBUS DP (IEC 61158 / DIN 19245-3).

---

## Français

### Matériel

| Élément | Notes |
|---|---|
| Adaptateur USB/RS-485 | Un vrai transceiver RS-485 (pas un simple UART TTL/RS-232). Les adaptateurs isolés à base FTDI conviennent. |
| Câblage | DB9 PROFIBUS de l'adaptateur : pin 3 = B (rouge), pin 8 = A (vert), pin 5 = GND. Sur le dongle : **vert → A**, **rouge → B**, blindage → GND. Si rien ne répond, inverser A/B. |
| Terminaison | Activer la résistance de terminaison du connecteur PROFIBUS côté adaptateur (bouton rouge sur "On") et le jumper 120 Ω du dongle. |
| Alimentation | 24 V DC sur l'adaptateur **et** alimentation terrain sur les embases des cartes de sortie, sinon les sorties restent éteintes même si le bus fonctionne. |
| Vitesse | 9600 (défaut) ou 19200 depuis un PC. L'APB détecte la vitesse au démarrage : après un changement, couper/remettre le 24 V. |
| Adresse | Roues codeuses sur l'adaptateur (01…99). |

Le format série est toujours 8E1, rien à régler de ce côté.

### Installation

```bash
sudo apt install python3-serial          # ou : pip install pyserial
sudo usermod -aG dialout $USER           # puis se reconnecter
```

### Commandes

```
pbmaster.py probe                         # balaye vitesses + adresses, inspecte chaque esclave
pbmaster.py scan      [--baud 9600]       # qui répond à cette vitesse ?
pbmaster.py diag      --slave 3           # diagnostic, config, état d'un esclave
pbmaster.py identify  --slave 3           # référence exacte des cartes Flex I/O par slot
pbmaster.py run       --slave 3           # échange cyclique + console interactive
pbmaster.py httpd     --slave 3 --account user:pass   # daemon sans clavier avec API HTTP
sudo pbmaster.py install  --slave 3 --account user:pass  # installe + démarre le service systemd
sudo pbmaster.py uninstall
```

Options communes : `--port /dev/ttyUSB0`, `--baud 9600`, `--master 1`, `--slave N`,
`--debug` (trames TX/RX), `--rts` (direction RS-485 par RTS pour les dongles sans
auto-direction), `--prm 0000` (octets de paramètres utilisateur : `0000` pour un APB
série B, `00` pour une série A), `--cfg <hex>` (forcer les octets Chk_Cfg), `--wd 300`
(watchdog esclave en unités de 10 ms), `--no-wd`.

#### `probe`

Essaie 9600 → 1,5 M bauds, interroge les adresses 0..126 à chaque vitesse, puis pour
chaque esclave trouvé affiche l'ident FDL, le diagnostic décodé (paramétré par qui,
`Prm_Req`, `Prm_Fault`/`Cfg_Fault`, watchdog), la config courante (`Get_Cfg`) et une
estimation des cartes Flex par slot. Signale aussi les octets de « bruit » (polarité ou
vitesse fausse) et si le dongle renvoie l'écho de ses propres trames.

#### `identify`

Utilise le format « condensé » des paramètres de l'APB pour tester les clés module
connues (IB16, OB16, IA8, OA8, IB8S, OW8, IE8, OE4, IE4XOE2, IT8, IR8, FM1) sur chaque
slot occupé et affiche la référence exacte. L'adaptateur est remis en auto-configure
ensuite. Une carte insérée après la mise sous tension n'est vue qu'après un cycle
d'alimentation de l'APB.

#### `run` — console interactive

Les entrées s'affichent à chaque changement, par slot, avec les numéros actifs et une
réglette (bit 0 à gauche) :

```
18:34:18 slot 1 IN  : 5,10           |.....X....X.....|
```

Commandes clavier (`<mod>` = slot `s2` ou module logique `#5`) :

```
b <mod> <bit> <0|1>      met un bit de sortie             (b s2 3 1)
w <mod> <hex>            écrit les octets du module       (w s2 00FF0000)
on <mod> / off <mod>     les 16 sorties à 1 / 0
o <hex>                  remplace tout le buffer de sortie
chrismas <mod>[,<mod>] [pas] [durée]
                         guirlande animée, ex. chrismas s2,s3 1s 60s
stop                     arrête la guirlande (sorties à 0)
i / d / ? / q            entrées / relire le diag / aide / quitter
```

Ctrl-C ou `q` remettent les sorties à 0 avant de quitter. Le watchdog esclave (3 s par
défaut) les remet à 0 de toute façon si le process meurt.

### API HTTP

```
pbmaster.py run   --slave 3 --account admin:secret        # console + API
pbmaster.py httpd --slave 3 --account admin:secret [--http-bind 127.0.0.1:8080] [--json-key-prefix F]
```

Auth basic sur toutes les routes. Les points sont nommés `E<slot>.<bit>` (entrées) et
`S<slot>.<bit>` (sorties), slot = position physique de la carte (1..8), bit = 0..15.
Avec une IB16 en slot 1 et des OB16 en slots 2 et 3 on obtient `E1`, `S2`, `S3`.

| Route | Description |
|---|---|
| `GET /` | Page web : un tableau par carte, LED pour les entrées, cases à cocher pour les sorties, barre Xmas (cartes, pas, durée, démarrer/stop, compte à rebours), rafraîchie toutes les 500 ms |
| `GET /in` | `{"E1.0":0, …, "E1.5":1, …}` |
| `GET /out` | état courant des sorties `{"S2.0":0, …}` |
| `POST /out` | corps `{"S2.3":1, "S3.0":0}` — tout est validé avant écriture |
| `GET /io` | cartes + entrées + sorties + âge des dernières données esclave |
| `GET /cards` | `{"E1":"entrées (16 bits)", "S2":"sorties (16 bits)", …}` |
| `POST /xmas` | corps `{"cards":["S2","S3"], "step":"50ms", "duration":"2m"}` — lance la guirlande (cartes : toutes les cartes de sorties par défaut, durée optionnelle) |
| `POST /xmas/stop` | l'arrête, sorties à 0 |

Un `POST /out` est atomique : slot inconnu, clé `E`, slot du mauvais type (`S1` sur une
carte d'entrées, `E2` sur une carte de sorties), bit > 15 ou valeur autre que
0/1/true/false/on/off → `400` et rien n'est écrit. Les zéros devant sont acceptés
(`S2.03`). `--json-key-prefix F` transforme les clés en `FE1.5` / `FS2.3` partout.

```bash
curl -u admin:secret http://127.0.0.1:8080/in
curl -u admin:secret -X POST -d '{"S2.0":1,"S3.15":"on"}' http://127.0.0.1:8080/out
```

HTTP en clair avec auth basic : à garder sur localhost, derrière Tailscale ou un reverse
proxy TLS.

### Service systemd

```bash
sudo python3 pbmaster.py install --slave 3 --account admin:secret [--http-bind 127.0.0.1:8080] [--json-key-prefix F]
```

`install` crée un utilisateur système `pbmaster` (groupe `dialout`), vérifie qu'il peut
réellement ouvrir le port série (ajoute une règle udev pour le dongle sinon ;
`--service-user root` en dernier recours), copie le script vers
`/usr/local/bin/pbmaster.py`, écrit les identifiants dans `/etc/pbmaster.conf` (mode
600, lu via `EnvironmentFile`, donc jamais visibles dans `ps`), écrit
`/etc/systemd/system/pbmaster.service` (`Restart=always`, durci), puis `daemon-reload` +
`enable --now`. Relancer `install` pour changer un paramètre. Journal :
`journalctl -u pbmaster -f`. `uninstall` retire le tout.

### Notes protocole

DP-V0 pur sur pyserial : `FDL Status` / `Request Ident`, `Slave_Diag` (SAP 60),
`Set_Prm` (61), `Chk_Cfg` (62), `Get_Cfg` (59), puis `Data_Exchange` cyclique avec
gestion FCB/FCV, suppression de l'écho half-duplex et comptage du bruit. L'Ident est lu
dans le diagnostic de l'esclave, donc séries A/B et rebadges OEM fonctionnent sans
changement. Chk_Cfg prend par défaut ce que l'esclave renvoie dans `Get_Cfg`. Sur un
1794-APB chaque slot occupe deux modules logiques (entrée, sortie) ; les modules #0/#1
sont le mot de statut de l'adaptateur.

Dépannage : aucune réponse et zéro bruit → câblage, pas de transceiver RS-485 ou
adaptateur non alimenté ; exactement un octet de bruit par trame à toutes les vitesses →
A/B inversés ; `Prm_Fault` après `Set_Prm` → essayer `--prm 00` (série A) ou
`--prm 0000` (série B) ; `défaut slot N` dans le mot de statut → carte insérée après la
mise sous tension, couper/remettre le 24 V de l'APB.

Références : manuel utilisateur Rockwell 1794-APB (publication 1794-6.5.6 /
1794-UM057), PROFIBUS DP (IEC 61158 / DIN 19245-3).
